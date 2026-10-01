using System;
using System.IO;
using FlaxEngine;

namespace MotionMatching;

/// <summary>
/// Scene-detached runtime for the native motion-matching system.
///
/// P3: plain C# class, zero scene presence. The game-side facade
/// (MotionMatchingPlayer) owns this object, feeds it a prebuilt
/// <see cref="MotionMatchingFrameInput"/> every tick (it calls the provider's
/// BuildInput itself) and drives the overlay post-pass + pose write. This
/// class owns the database, policy/playback lifecycle and core-only tuning.
/// Everything locomotion (gait scope, trajectory prediction, gait speeds)
/// arrives through the frame argument. This file must never reference game
/// types (PlayerState, coordinators, automation, overlays).
/// </summary>
public class MotionMatchingController : IDisposable
{
    /// <summary>
    /// Bake contract authored by the host. A snapshot is stored inside the database on bake;
    /// </summary>
    public PoseSearchSchema Schema = CreateDefaultSchema();

    public SkinnedModel PlayerModel;

    public string DatabasePath = "MotionMatching/MotionMatchingDatabase.flax";

    public MotionMatchingDatabase Database;

    public int AnimationClipsCount;

    public int SampleCount;

    public bool PersistenceValid;

    public string[] AnimationTags = Array.Empty<string>();

    public int[] AnimationClipCountsByTag = Array.Empty<int>();

    // Diagnostics (debug only, off by default): records every query into the
    // bounded trace ring (Trace) and collects per-class search diagnostics.
    // Never changes which clip wins. Export the trace with Trace.ExportJson /
    // ExportCsv to a path the host chooses.
    public bool Diagnostics = false;

    public AnimatedModel CharacterBody;

    public float QueryInterval = 0.1f;

    // Per-frame search: the query gate runs every warmed tick instead of on
    // the QueryInterval cadence, so input latency never waits for a tick
    // boundary. Full scans stay budgeted (FrameSearchBudget) with a rotating
    // stride + continuation exemption, and intent-change queries always scan
    // unstrided — same total CPU as cadence, minimal worst-case latency.
    public bool SearchEveryFrame = true;

    // Samples scored per full scan (rotating stride covers the rest over
    // following frames). 12000 ≈ 1ms on the reference machine. 0 = unstrided.
    public int FrameSearchBudget = 12000;

    // Phase 3 (KDTREE_PLAN.md): indexed path is live by default (phase-2
    // audit: 0.33% same-clip-twin divergence, suite parity gate below).
    // Brute force stays as fallback + permanent verify harness.
    public bool UseIndexedSearch = true;

    // Candidates retrieved per tag before exact rerank. 16 (measured
    // 2026-09-14: sub-1ms scans, 0/1500 verify divergence, 12/12 suite).
    public int TopKCandidates = 16;

    // Phase 2 (KDTREE_PLAN.md): run brute force alongside the indexed path,
    // count divergences, keep the brute-force winner. Permanent debug harness.
    public bool VerifyIndexedSearch = false;

    // P2: foot-IK solver deleted with the plugin lock machine (field died
    // with it). Game-side FootIKEngine (no separate script) owns all foot
    // solving via the facade single-publish pipeline; search contact
    // features (bake + query) are untouched.

    // NOTE: layer decisions (procedural angle, bone gate) live
    // game-side now (P2 post-pass in the game facade): the provider
    // snapshot still carries the host-authored angle/gate for the game
    // pass, but nothing is forwarded into the core anymore.

    // Switch penalty added to any different-clip winner (continuity block).
    // Rev-3 cost units (N1b shrank dissimilarity ~100x): 0.0025 restores the
    // old ratio against typical winner costs (~0.01-0.2 measured).
    public float ContinuityWeight = 0.0025f;

    public float MinSwitchTime = 0.12f;

    // Continuation stickiness (live tuning, no rebake): bias rewards the
    // natural next frames on the current clip, margin forces other clips to
    // beat them clearly. Rev-3 cost units: N1b shrank dissimilarity ~100x
    // (measured mean winner cost 0.095), so both scale down equally to keep
    // the old hold/switch ratios. UE philosophy is the same direction (tiny
    // -0.01 bias + interrupt logic instead of giant margins).
    public float ContinuingBias = 0.012f;

    public float ContinuingWindow = 0.15f;

    public float SwitchMargin = 0.008f;

    // foot-ik branch flip work: P2 DELETED (FairFightAccum/Window died with
    // the native intent accumulator; the Game selection policy owns onset/
    // fair-fight and passes only the resulting filter per tick). Values
    // preserved Game-side in LocomotionSelectionConfig (40/0.25).

    // Heuristic thresholds below were hardcoded constants (codex review):
    // exposed live so tuning moves numbers instead of code. No rebake needed.
    // P2: SettleToLoopDelay died with the native settle branch (Game policy
    // owns settle; value preserved Game-side in LocomotionSelectionConfig).

    public float SameClipExclusion = 0.25f;

    public float SameClipSeekThreshold = 0.25f;

    // P3: Turn* game-driver config moved to the game facade
    // (MotionMatchingPlayer.TurnDriveAngle/Lookahead/SettleRate/PatientAngle/
    // PatientTime). The native turn machine is deleted and never read these;
    // the game-side TurnDriver is the only reader. TurnMinSwitchTime died
    // with them: it was never pushed into the native tuning (no reader in
    // the P2 headers) and no driver reads it.

    // Cost fuse: a switched winner above this is treated as a corrupted query
    // (observed: single-query pose spike mid steady walk, self-heals next
    // query) — hold current instead of cascading. Rev-3 units: legit switches
    // peak ~0.9 measured, so 2.0 keeps headroom without ever firing legally.
    public float CostFuseThreshold = 2.0f;

    // P2 straightness prior (was hardcoded 0.03/4.0 in MotionMatchingSearch):
    // Game-owned tuning, values preserved from the calibration. Recorded in
    // the trace so brute-force, indexed and replay share one formula.
    public float TurnPenaltyWeight = 0.03f;
    public float TurnPenaltyYawScale = 4.0f;

    // Speed matching: clip time bends toward FrameInput.TargetSpeed within
    // [PlayRateMin, PlayRateMax]; off at or below PlayRateMinSpeed.
    public float PlayRateMin = 0.85f;
    public float PlayRateMax = 1.15f;
    public float PlayRateMinSpeed = 10.0f;

    // Structural limits of the search policy (world units follow the rig).
    // Actor displacement per tick treated as a teleport and reseeded.
    public float TeleportDistance = 2000.0f;
    // Seconds after init or reseed before searches start.
    public float WarmupTime = 0.5f;
    // Continuation hold: seconds, and minimum frame cap (scaled for high fps).
    public float FastPathMaxAge = 0.5f;
    public int FastPathMaxFrames = 5;
    // A just-left clip cannot win again for ReselectBanHistory seconds (0 =
    // off); ReselectBanWindow is the clip-time half-width of the ban.
    public float ReselectBanHistory = 0.6f;
    public float ReselectBanWindow = 0.35f;
    // Samples this close (seconds) to the playing pose of the same clip.
    public float SameSegmentTwinBan = 0.25f;
    // Seconds at the end of non-loop clips excluded from search (0 = off).
    public float LoopTailExclusion = 0.0f;

    // Largest skeleton (bone count) the zero-allocation pose read accepts.
    public const int MaxPoseBones = 512;

    // Inertialization decay time (replaces linear-crossfade BlendDuration;
    // single path, no parallel blend modes).
    public float InertializationDuration = 0.25f;

    // Observation mirrors (written only by Update below, read by the host,
    // editor service and tools).
    public string PolicyState { get; private set; } = "Disabled";
    public int WinnerSample { get; private set; } = -1;
    public int WinnerClip { get; private set; } = -1;
    public string WinnerClipName { get; private set; } = string.Empty;
    public float WinnerCost { get; private set; } = float.MaxValue;
    public float SecondBestCost { get; private set; } = float.MaxValue;
    public float WinnerPoseCost { get; private set; } = float.MaxValue;
    public float WinnerTrajectoryCost { get; private set; } = float.MaxValue;
    public float WinnerContinuityCost { get; private set; }
    public float WinnerLoopCost { get; private set; }
    public int SwitchCount { get; private set; }
    public int SeekCount { get; private set; }
    public float LastQueryMs { get; private set; }
    public string ActiveTag { get; private set; } = string.Empty;

    // P2: TurnDriveActive/Yaw mirrors deleted with the native turn machine.
    // Committed-turn facing is owned game-side (facade driver facing); the
    // core never steers facing anymore.

    public bool WinnerIsLoop { get; private set; }
    public float LoopPreferenceApplied { get; private set; }
    public float BestLoopCost { get; private set; } = float.MaxValue;
    public string BestLoopClipName { get; private set; } = string.Empty;
    public float BestOneShotCost { get; private set; } = float.MaxValue;
    public string BestOneShotClipName { get; private set; } = string.Empty;
    public int LoopScoredCount { get; private set; }
    public int OneShotScoredCount { get; private set; }
    public int LoopTotalCount { get; private set; }
    public int OneShotTotalCount { get; private set; }
    public string LastSwitchReason { get; private set; } = string.Empty;
    public string ChooserStatus { get; private set; } = string.Empty;
    public int TraceEntries { get; private set; }
    // Profiling mirrors (record-only; copied from the native policy after
    // each Update, never steer anything).
    public float LastTickMs { get; private set; }
    public float LastFastPathMs { get; private set; }
    public int TickCount { get; private set; }
    public int FullSearchCount { get; private set; }
    public float TotalTickMs { get; private set; }
    public float TotalFullSearchMs { get; private set; }
    public float TotalFastPathMs { get; private set; }

    public int FastPathHits { get; private set; }

    private MotionMatchingRuntimePolicy _policy;
    private bool _diagnosticsWarned;
    private MotionMatchingDatabase _boundDatabase;
    private AnimatedModel _boundBody;
    private SkinnedModel _boundModel;
    private PoseSearchSchema _boundSchema;
    private string _loadedDatabaseFile = string.Empty;

    public float[] TrajectorySampleTimes
    {
        get
        {
            ReleaseStaleBinding();
            // P5: the Database getter is a native binding that allocates a
            // managed float[] per call; the baked times never change under a
            // loaded database in play, so cache the array and refresh only
            // when the database reference changes. Callers (facade tick +
            // Update, 2x per tick) share the instance; nobody mutates it.
            // The null-database fallback (unbaked: the policy never ticks)
            // reads live with no caching.
            if (!Database)
                return Schema.TrajectorySampleTimes ?? Array.Empty<float>();
            if (!ReferenceEquals(Database, _sampleTimesDb))
            {
                _sampleTimesDb = Database;
                _sampleTimesCache = Database.TrajectorySampleTimes ?? Schema.TrajectorySampleTimes;
            }
            return _sampleTimesCache;
        }
    }

    private MotionMatchingDatabase _sampleTimesDb;
    private float[] _sampleTimesCache = Array.Empty<float>();

    public bool IsReady
    {
        get
        {
            ReleaseStaleBinding();
            return _policy && _policy.IsReady;
        }
    }

    /// <summary>
    /// Live playback handle for game-side integrations (e.g. pose overlays
    /// attach here). Null until the first update initializes the policy.
    /// Borrowed handle: reacquire after every rebind/shutdown; never destroy it.
    /// </summary>
    public MotionMatchingPlayback Playback => IsReady ? _policy.GetPlayback() : null;

    /// <summary>
    /// P0 additive-only: publish the existing search-pose path without
    /// touching Update. Returns the playback's current search locals plus the
    /// current winner (false + defaults when the policy is not ready).
    /// </summary>
    public bool TryGetBasePose(out Matrix[] locals, out int winnerClip, out float winnerCost)
    {
        locals = System.Array.Empty<Matrix>();
        winnerClip = -1;
        winnerCost = float.MaxValue;
        if (!IsReady)
            return false;
        var playback = _policy.GetPlayback();
        if (playback == null)
            return false;
        locals = playback.GetSearchLocals() ?? System.Array.Empty<Matrix>();
        winnerClip = _policy.WinnerClip;
        winnerCost = _policy.WinnerCost;
        return true;
    }

    /// <summary>
    /// P5 zero-alloc base-pose read: copies the playback search locals into
    /// the caller-owned buffer (no managed array allocation, unlike
    /// TryGetBasePose which allocates via GetSearchLocals). The caller
    /// reuses its buffer and resizes only when the bone count changes
    /// (read it via Playback.GetSearchLocalsCount first). Bound: model bone
    /// count (91 on the reference rig, callers cap at 512). Same snapshot
    /// data as TryGetBasePose (post-inertial search pose); the native
    /// history is never aliased with the caller buffer (per-bone copy).
    /// Returns false (no copy) when the policy is not ready, the buffer is
    /// null, or its length differs from the bone count.
    /// </summary>
    public bool TryGetBasePoseInto(Matrix[] buffer, out int winnerClip, out float winnerCost)
    {
        winnerClip = -1;
        winnerCost = float.MaxValue;
        if (!IsReady || buffer == null)
            return false;
        var playback = _policy.GetPlayback();
        if (playback == null)
            return false;
        int n = playback.GetSearchLocalsCount();
        if (n <= 0 || n > MaxPoseBones || buffer.Length != n)
            return false;
        for (int i = 0; i < n; i++)
        {
            if (!playback.GetSearchLocal(i, out buffer[i]))
                return false;
        }
        winnerClip = _policy.WinnerClip;
        winnerCost = _policy.WinnerCost;
        return true;
    }

    /// <summary>
    /// Explicit audit command: writes the per-clip travel/turn/speed table
    /// to <paramref name="filePath"/>. Static native export: runs in edit
    /// mode with no play/policy instance. Runtime init/play never scans the
    /// database or writes files.
    /// </summary>
    public bool GenerateClipAudit(string filePath, out string error)
    {
        error = string.Empty;
        if (Database == null || !Database.IsBaked)
        {
            error = "Clip audit stopped: database is not baked.";
            return false;
        }
        try
        {
            return MotionMatchingRuntimePolicy.ExportClipAudit(
                Database, filePath ?? string.Empty, Schema.SampleRate, out error);
        }
        catch (System.Exception ex)
        {
            error = "Clip audit stopped: " + ex.Message;
            return false;
        }
    }

    /// <summary>
    /// P3 explicit init (replaces the Script OnEnable): the game facade
    /// assigns Schema/PlayerModel/DatabasePath/CharacterBody + session
    /// config first, then calls this. Content.Load works outside Scripts —
    /// Globals remain usable here.
    /// </summary>
    public void Initialize()
    {
        ReleaseStaleBinding();

        // Unified load path (no editor partial): Content.Load works in both
        // editor and game builds; the editor service refreshes explicitly
        // after bake/registry edits.
        EnsureDatabaseLoadedForGame();
    }

    /// <summary>
    /// Releases the owned native policy, playback and trace immediately, then
    /// schedules destruction of the policy object. Idempotent; call on the
    /// Flax main thread on host disable/destroy. Export the trace first.
    /// Configuration and host-owned assets are retained for editor tooling and
    /// reuse: a subsequent Update starts a fresh session without requiring Initialize.
    /// Previously borrowed Playback/Trace handles are invalid after this call.
    /// </summary>
    public void Shutdown()
    {
        var policy = _policy;
        _policy = null;
        _boundDatabase = null;
        _boundBody = null;
        _boundModel = null;
        _boundSchema = default;
        _sampleTimesDb = null;
        _sampleTimesCache = Array.Empty<float>();
        ResetRuntimeStatus();
        if (policy)
        {
            try
            {
                policy.Shutdown();
            }
            finally
            {
                FlaxEngine.Object.Destroy(policy);
            }
        }
    }

    /// <summary>Equivalent to Shutdown; the configured controller remains reusable.</summary>
    public void Dispose() => Shutdown();

    private void ReleaseStaleBinding()
    {
        // Keep public fields (including nested struct edits) compatible with
        // existing hosts/tools. Validate before any native read or command.
        if (ReferenceEquals(_policy, null))
            return;
        if (!_policy || !Database || !CharacterBody ||
            !ReferenceEquals(Database, _boundDatabase) ||
            !ReferenceEquals(CharacterBody, _boundBody) ||
            !ReferenceEquals(CharacterBody.SkinnedModel, _boundModel) ||
            (!string.IsNullOrEmpty(_loadedDatabaseFile) &&
             !string.Equals(DatabaseFile, _loadedDatabaseFile, StringComparison.OrdinalIgnoreCase)) ||
            !SchemaBindingEquals(Schema, _boundSchema) || !_policy.IsReady)
            Shutdown();
    }

    private static bool SchemaBindingEquals(in PoseSearchSchema a, in PoseSearchSchema b)
    {
        // Weights are query-time tuning, not binding identity.
        var identity = a;
        identity.PoseWeight = b.PoseWeight;
        identity.TrajectoryWeight = b.TrajectoryWeight;
        return SchemaEquals(identity, b);
    }

    private void ResetRuntimeStatus()
    {
        PolicyState = "Disabled";
        WinnerSample = WinnerClip = -1;
        WinnerClipName = ActiveTag = BestLoopClipName = BestOneShotClipName = string.Empty;
        LastSwitchReason = ChooserStatus = string.Empty;
        WinnerCost = SecondBestCost = WinnerPoseCost = WinnerTrajectoryCost = float.MaxValue;
        BestLoopCost = BestOneShotCost = float.MaxValue;
        WinnerContinuityCost = WinnerLoopCost = LoopPreferenceApplied = 0.0f;
        WinnerIsLoop = false;
        SwitchCount = SeekCount = LoopScoredCount = OneShotScoredCount = 0;
        LoopTotalCount = OneShotTotalCount = TraceEntries = FastPathHits = 0;
        LastQueryMs = LastTickMs = LastFastPathMs = 0.0f;
        TickCount = FullSearchCount = 0;
        TotalTickMs = TotalFullSearchMs = TotalFastPathMs = 0.0f;
    }

    // P3: the P2 rebuild-compat reflection shim for the legacy native scope
    // gate is deleted with the facade (the P2 headers no longer carry it).

    /// <summary>
    /// Clears native playback state. The game facade resets its provider
    /// (scope + predictor) and drivers separately — this class no longer
    /// owns any provider reference. Call after teleport/respawn.
    /// </summary>
    public void ResetPlayback()
    {
        ReleaseStaleBinding();
        _policy?.ResetOnTeleport();
    }

    /// <summary>
    /// P3 policy + playback tick. The game facade builds the provider frame
    /// itself (provider.BuildInput) and passes it in with the actor world
    /// transform — this class never calls a provider. Loads the database on
    /// first tick if Initialize found none.
    /// </summary>
    public void Update(float deltaTime, Transform actorWorld, MotionMatchingFrameInput input)
    {
        ReleaseStaleBinding();
        if (!CharacterBody)
            return;

        if (_policy == null)
        {
            EnsureDatabaseLoadedForGame();
            if (!Database || !Database.IsBaked || !CharacterBody.SkinnedModel)
                return;
            _policy = new MotionMatchingRuntimePolicy();
            _policy.Initialize(Database, CharacterBody, Schema, InertializationDuration);
            if (!_policy.IsReady)
            {
                Shutdown();
                return;
            }
            _boundDatabase = Database;
            _boundBody = CharacterBody;
            _boundModel = CharacterBody.SkinnedModel;
            _boundSchema = Schema;
            // Snapshot array contents too: hosts can edit sample times in place.
            _boundSchema.TrajectorySampleTimes = Schema.TrajectorySampleTimes != null
                ? (float[])Schema.TrajectorySampleTimes.Clone() : Array.Empty<float>();
        }
        if (!_policy.IsReady)
            return;

        if (_policy.GetTrace() != null)
            _policy.GetTrace().Enabled = Diagnostics;
        if (Diagnostics && !_diagnosticsWarned)
        {
            _diagnosticsWarned = true;
            Debug.LogWarning("Motion matching diagnostics are ACTIVE (debug only: every query is recorded into the trace ring). Turn off MotionMatchingController.Diagnostics for shipping builds.");
        }

        // P2: the snapshot carries the caller scope only (movement fields
        // died with the native gameplay state). LoopFilter/CandidateBand/
        // YawRate ride explicit Tick params (never inferred); the core uses
        // them verbatim. No new native struct field (bindings cap 64).
        var snapshot = new MotionMatchingPlayerSnapshot
        {
            AllowedTagSet = input.AllowedTagSet ?? string.Empty,
            ExcludedTagSet = input.ExcludedTagSet ?? string.Empty,
        };
        var tuning = new MotionMatchingPolicyTuning
        {
            PoseWeight = Schema.PoseWeight,
            TrajectoryWeight = Schema.TrajectoryWeight,
            SampleRate = Schema.SampleRate,
            MinSwitchTime = MinSwitchTime,
            SameClipExclusion = SameClipExclusion,
            SameClipSeekThreshold = SameClipSeekThreshold,
            ContinuingBias = ContinuingBias,
            ContinuingWindow = ContinuingWindow,
            SwitchMargin = SwitchMargin,
            FrameSearchBudget = FrameSearchBudget,
            SearchEveryFrame = SearchEveryFrame,
            QueryInterval = QueryInterval,
            UseIndexedSearch = UseIndexedSearch,
            TopKCandidates = TopKCandidates,
            VerifyIndexedSearch = VerifyIndexedSearch,
            Diagnostics = Diagnostics,
            ContinuityWeight = ContinuityWeight,
            CostFuseThreshold = CostFuseThreshold,
            TurnPenaltyWeight = TurnPenaltyWeight,
            TurnPenaltyYawScale = TurnPenaltyYawScale,
            PlayRateMin = PlayRateMin,
            PlayRateMax = PlayRateMax,
            PlayRateMinSpeed = PlayRateMinSpeed,
            TeleportDistance = TeleportDistance,
            WarmupTime = WarmupTime,
            FastPathMaxAge = FastPathMaxAge,
            FastPathMaxFrames = FastPathMaxFrames,
            ReselectBanHistory = ReselectBanHistory,
            ReselectBanWindow = ReselectBanWindow,
            SameSegmentTwinBan = SameSegmentTwinBan,
            LoopTailExclusion = LoopTailExclusion,
        };
        int callerLoop = input.LoopFilter >= 0 && input.LoopFilter <= 2 ? input.LoopFilter : 0;
        int callerBand = input.CandidateBand >= 0 && input.CandidateBand <= 3 ? input.CandidateBand : 0;
        float callerYaw = input.QueryYawRate >= 0.0f && input.QueryYawRate < 1e29f ? input.QueryYawRate : 0.0f;
        float targetSpeed = input.TargetSpeed >= 0.0f && input.TargetSpeed < 1e29f ? input.TargetSpeed : 0.0f;
        _policy.Tick(deltaTime, snapshot,
            input.Points ?? Array.Empty<TrajectoryPoint>(),
            actorWorld.GetWorld(), TrajectorySampleTimes, tuning,
            callerLoop, callerBand, callerYaw, targetSpeed, input.HoldReseedFilter);

        PolicyState = _policy.PolicyState;
        WinnerSample = _policy.WinnerSample;
        WinnerClip = _policy.WinnerClip;
        WinnerClipName = _policy.WinnerClipName;
        WinnerCost = _policy.WinnerCost;
        SecondBestCost = _policy.SecondBestCost;
        WinnerPoseCost = _policy.WinnerPoseCost;
        WinnerTrajectoryCost = _policy.WinnerTrajectoryCost;
        WinnerContinuityCost = _policy.WinnerContinuityCost;
        SwitchCount = _policy.SwitchCount;
        SeekCount = _policy.SeekCount;
        LastQueryMs = _policy.LastQueryMs;
        ActiveTag = _policy.ActiveTag;
        WinnerLoopCost = _policy.WinnerLoopCost;
        WinnerIsLoop = _policy.WinnerIsLoop;
        LoopPreferenceApplied = _policy.LoopPreferenceApplied;
        BestLoopCost = _policy.BestLoopCost;
        BestLoopClipName = _policy.BestLoopClipName;
        BestOneShotCost = _policy.BestOneShotCost;
        BestOneShotClipName = _policy.BestOneShotClipName;
        LoopScoredCount = _policy.LoopScoredCount;
        OneShotScoredCount = _policy.OneShotScoredCount;
        LoopTotalCount = _policy.LoopTotalCount;
        OneShotTotalCount = _policy.OneShotTotalCount;
        LastSwitchReason = _policy.LastSwitchReason;
        ChooserStatus = _policy.ChooserStatusText;
        TraceEntries = _policy.GetTrace() != null ? _policy.GetTrace().Count : 0;
        FastPathHits = _policy.FastPathHits;
        // P0 profiling mirrors (record-only).
        LastTickMs = _policy.LastTickMs;
        LastFastPathMs = _policy.LastFastPathMs;
        TickCount = _policy.TickCount;
        FullSearchCount = _policy.FullSearchCount;
        TotalTickMs = _policy.TotalTickMs;
        TotalFullSearchMs = _policy.TotalFullSearchMs;
        TotalFastPathMs = _policy.TotalFastPathMs;
    }

    // M6: the policy's query trace (null until the first play tick). The
    // tuning window reads it for top-N, export, and replay.
    // Borrowed handle, with the same lifetime as Playback.
    public MotionMatchingTraceLog Trace => IsReady ? _policy.GetTrace() : null;
    public MotionMatchingTraceEntry LastTraceEntry => IsReady ? _policy.GetLastEntry() : new MotionMatchingTraceEntry();

    private void EnsureDatabaseLoadedForGame()
    {
        string databaseFile = DatabaseFile;
        // A host may inject an already-loaded database (editor tooling and
        // isolated contract controllers do this). Once that reference is
        // adopted, the normal binding checks below own rebind detection.
        if (Database && (string.IsNullOrEmpty(_loadedDatabaseFile) ||
            string.Equals(_loadedDatabaseFile, databaseFile, StringComparison.OrdinalIgnoreCase)))
            return;
        if (_policy)
            Shutdown();
        Database = Content.Load<MotionMatchingDatabase>(databaseFile);
        _loadedDatabaseFile = Database ? databaseFile : string.Empty;
        RefreshDatabaseStatus();
    }

    /// <summary>
    /// Resolved database asset path. Public so editor services and host
    /// tooling can operate on the same asset without duplicating the
    /// fallback logic.
    /// </summary>
    public string DatabaseFile
    {
        get
        {
            string path = string.IsNullOrWhiteSpace(DatabasePath)
                ? "MotionMatching/MotionMatchingDatabase.flax"
                : DatabasePath.Trim();
            return StringUtils.NormalizePath(Path.IsPathRooted(path)
                ? path
                : Path.Combine(Globals.ProjectContentFolder, path));
        }
    }

    public static PoseSearchSchema CreateDefaultSchema()
    {
        // Skeleton ships EMPTY on purpose (no mannequin names): the host
        // owning this scene overrides every bone name for its own rig
        // (Schema → Skeleton). PoseSearchSchema.Default already carries the
        // empty profile, so nothing is filled in here.
        var schema = PoseSearchSchema.Default;
        schema.TrajectorySampleTimes = new float[]{ -0.4f, -0.2f, 0.0f, 0.2f, 0.4f, 0.6f};
        return schema;
    }

    public static bool SchemaEquals(in PoseSearchSchema a, in PoseSearchSchema b)
    {
        float AxisTolerance = 0.001f;

        // The baker stores normalized axes, so compare directions rather than raw vectors.
        if (a.SampleRate != b.SampleRate ||
            !Float3.NearEqual(Float3.Normalize(a.RootForwardAxis), Float3.Normalize(b.RootForwardAxis), AxisTolerance) ||
            !Float3.NearEqual(Float3.Normalize(a.RootUpAxis), Float3.Normalize(b.RootUpAxis), AxisTolerance) ||
            a.PoseWeight != b.PoseWeight ||
            a.TrajectoryWeight != b.TrajectoryWeight)
            return false;

        if (a.Skeleton.Root != b.Skeleton.Root ||
            a.Skeleton.Pelvis != b.Skeleton.Pelvis ||
            a.Skeleton.Spine != b.Skeleton.Spine ||
            a.Skeleton.Head != b.Skeleton.Head ||
            a.Skeleton.LeftFoot != b.Skeleton.LeftFoot ||
            a.Skeleton.RightFoot != b.Skeleton.RightFoot ||
            a.Skeleton.LeftHand != b.Skeleton.LeftHand ||
            a.Skeleton.RightHand != b.Skeleton.RightHand)
            return false;

        var ta = a.TrajectorySampleTimes ?? Array.Empty<float>();
        var tb = b.TrajectorySampleTimes ?? Array.Empty<float>();
        if (ta.Length != tb.Length)
            return false;
        for (int i = 0; i < ta.Length; i++)
        {
            if (ta[i] != tb[i])
                return false;
        }
        return true;
    }

    public void RefreshDatabaseStatus()
    {
        // Explicit editor reload/bake notification, including same-asset reloads.
        Shutdown();
        _loadedDatabaseFile = Database ? DatabaseFile : string.Empty;
        // Non-blocking check: WaitForLoaded defaults to a 30s main-thread
        // block, and Content.Load above already waited for the settled state.
        if (!Database || Database.WaitForLoaded(0))
        {
            ClearDatabaseStatus();
            return;
        }

        AnimationClipsCount = Database.AnimationClipsCount;
        SampleCount = Database.SampleCount;
        AnimationTags = Database.AnimationTags ?? Array.Empty<string>();
        AnimationClipCountsByTag = Database.AnimationClipCountsByTag ?? Array.Empty<int>();
        PersistenceValid = Database.ValidateData();
        // Gait-config validation lives in the provider now: it validates
        // against the inventory handed over in the tick context.
    }

    public void ClearDatabaseStatus()
    {
        Shutdown();
        _loadedDatabaseFile = string.Empty;
        Database = null;
        AnimationClipsCount = 0;
        SampleCount = 0;
        AnimationTags = Array.Empty<string>();
        AnimationClipCountsByTag = Array.Empty<int>();
        PersistenceValid = false;
    }
}
