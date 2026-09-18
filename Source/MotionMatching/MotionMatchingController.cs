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
public class MotionMatchingController
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

    // Host-assigned session config (set by the game facade, e.g. from
    // automation flags — this class never reads automation itself).
    public string CacheFolder = string.Empty;
    public bool TelemetryOverride = false;
    public bool IndexedVerifyAuto = false;

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
    // with it). Game-side FootIKController owns all foot solving; search
    // contact features (bake + query) are untouched.

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

    // foot-ik branch flip work: fair-fight intent accumulator trip level
    // (u/s accumulated with 0.7/tick decay) and unbiased-scan latch window
    // (seconds). A 45deg walk flip slews ~20u/s per tick and trips in 2-3
    // ticks; steady-motion noise stays an order of magnitude below.
    public float FairFightAccum = 40.0f;
    public float FairFightWindow = 0.25f;

    // Heuristic thresholds below were hardcoded constants (codex review):
    // exposed live so tuning moves numbers instead of code. No rebake needed.
    public float SettleToLoopDelay = 0.3f;

    // Rest-loop preference (loops win ties at rest so one-shot breaks/stops
    // cannot re-win forever on exact pose match). Rev-3 units: rest gaps run
    // ~0.005-0.02 measured, so 0.02 settles rest without touching transitions
    // (preference is zero unless restCalm). Raised from 0.005 after measured
    // Idle_Break churn in suite phase 10.
    public float RestLoopPreference = 0.02f;

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

    // Inertialization decay time (replaces linear-crossfade BlendDuration;
    // single path, no parallel blend modes).
    public float InertializationDuration = 0.25f;

    public bool TelemetryLog = false;

    // Effective switch: the host field above never changes at runtime —
    // the host flips TelemetryOverride (e.g. from file-driven automation),
    // so the scene is never dirtied by a play session.
    public bool EffectiveTelemetry => TelemetryLog || TelemetryOverride;

    // Telemetry mirrors (getters: written only by Update below, read by the
    // game facade / editor service / bridge).
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

    // P2: machine/overlay/IK telemetry deleted with their producers
    // (HoldCount, TurnReplayCount, SettleCount, RecoveryCount, JumpCount,
    // LayerAngle/Primary/Secondary/GateApplied, FootLockL/R, FootCorrL/R).
    // Search telemetry above (switches, seeks, query ms, fast path) stays.
    public int FastPathHits { get; private set; }

    private MotionMatchingRuntimePolicy _policy;
    private bool _telemetryWarned;

    public float[] TrajectorySampleTimes => Database?.TrajectorySampleTimes ?? Schema.TrajectorySampleTimes;

    public bool IsReady => _policy != null && _policy.IsReady;

    /// <summary>
    /// Live playback handle for game-side integrations (e.g. pose overlays
    /// attach here). Null until the first update initializes the policy.
    /// </summary>
    public MotionMatchingPlayback Playback => _policy != null ? _policy.GetPlayback() : null;

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
        if (_policy == null || !_policy.IsReady)
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
    /// P0 additive-only force-clip hatch for future game drivers. Forwards
    /// to playback Play with forceSeek:true and logs the reason on EVERY
    /// call (anti-abuse). No production callers yet.
    /// </summary>
    public void ForcePlayClip(int clipIndex, float startTime, string reason)
    {
        Debug.Log("MotionMatchingController.ForcePlayClip: clip=" + clipIndex
            + " startTime=" + startTime + " reason=" + (reason ?? string.Empty));
        var playback = Playback;
        if (playback == null)
            return;
        playback.Play(clipIndex, startTime, true);
    }

    /// <summary>
    /// P3 explicit init (replaces the Script OnEnable): the game facade
    /// assigns Schema/PlayerModel/DatabasePath/CharacterBody + session
    /// config first, then calls this. Content.Load works outside Scripts —
    /// Globals remain usable here.
    /// </summary>
    public void Initialize()
    {
        _telemetryWarned = false;

        // Unified load path (no editor partial): Content.Load works in both
        // editor and game builds; the editor service refreshes explicitly
        // after bake/registry edits.
        EnsureDatabaseLoadedForGame();
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
        if (CharacterBody == null)
            return;

        if (_policy == null)
            _policy = new MotionMatchingRuntimePolicy();
        if (!_policy.IsReady)
        {
            EnsureDatabaseLoadedForGame();
            // Telemetry file reset (was inside the C# policy init): fresh
            // file per play session, deleted only when logging.
            if (EffectiveTelemetry)
            {
                try
                {
                    if (!string.IsNullOrEmpty(CacheFolder))
                    {
                        string telemetryPath = Path.Combine(CacheFolder, "Cache", "MMTelemetry.jsonl");
                        if (File.Exists(telemetryPath))
                            File.Delete(telemetryPath);
                    }
                }
                catch
                {
                }
            }
            _policy.Initialize(Database, CharacterBody, Schema,
                CacheFolder ?? string.Empty, InertializationDuration);
            if (_policy.GetPlayback() != null)
                _policy.GetPlayback().InertializationDuration = InertializationDuration;
        }
        if (!_policy.IsReady)
            return;

        // Debug-only trace gate with a visible warning (codex review): the
        // host TelemetryLog field is never written at runtime; the host
        // flips only TelemetryOverride above.
        if (_policy.GetTrace() != null)
            _policy.GetTrace().Enabled = EffectiveTelemetry;
        if (EffectiveTelemetry && !_telemetryWarned)
        {
            _telemetryWarned = true;
            Debug.LogWarning("Motion matching telemetry/trace is ACTIVE (debug-only: per-query clones + file logging). Turn off TelemetryLog for shipping builds.");
        }

        var snapshot = new MotionMatchingPlayerSnapshot
        {
            CurrentVelocity = input.Velocity,
            DesiredVelocity = input.DesiredVelocity,
            AngularVelocity = input.AngularVelocity,
            TargetYaw = input.TargetYaw,
            CurrentYaw = input.CurrentYaw,
            IsGrounded = input.IsGrounded,
            IsSprinting = input.IsSprinting,
            AllowedTagSet = input.AllowedTagSet ?? string.Empty,
            ExcludedTagSet = input.ExcludedTagSet ?? string.Empty,
        };
        var tuning = new MotionMatchingPolicyTuning
        {
            PoseWeight = Schema.PoseWeight,
            TrajectoryWeight = Schema.TrajectoryWeight,
            SampleRate = Schema.SampleRate,
            FairFightAccum = FairFightAccum,
            FairFightWindow = FairFightWindow,
            SettleToLoopDelay = SettleToLoopDelay,
            MinSwitchTime = MinSwitchTime,
            SameClipExclusion = SameClipExclusion,
            SameClipSeekThreshold = SameClipSeekThreshold,
            ContinuingBias = ContinuingBias,
            ContinuingWindow = ContinuingWindow,
            SwitchMargin = SwitchMargin,
            RestLoopPreference = RestLoopPreference,
            FrameSearchBudget = FrameSearchBudget,
            SearchEveryFrame = SearchEveryFrame,
            QueryInterval = QueryInterval,
            UseIndexedSearch = UseIndexedSearch,
            TopKCandidates = TopKCandidates,
            VerifyIndexedSearch = VerifyIndexedSearch,
            IndexedVerifyAuto = IndexedVerifyAuto,
            EffectiveTelemetry = EffectiveTelemetry,
            ContinuityWeight = ContinuityWeight,
            CostFuseThreshold = CostFuseThreshold,
        };
        _policy.Tick(deltaTime, snapshot,
            input.Points ?? Array.Empty<TrajectoryPoint>(),
            actorWorld.GetWorld(), TrajectorySampleTimes, tuning);

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
    }

    // M6: the policy's query trace (null until the first play tick). The
    // tuning window reads it for top-N, export, and replay.
    public MotionMatchingTraceLog Trace => _policy != null ? _policy.GetTrace() : null;
    public MotionMatchingTraceEntry LastTraceEntry => _policy != null ? _policy.GetLastEntry() : new MotionMatchingTraceEntry();

    private void EnsureDatabaseLoadedForGame()
    {
        if (Database)
            return;
        Database = Content.Load<MotionMatchingDatabase>(DatabaseFile);
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
        Database = null;
        AnimationClipsCount = 0;
        SampleCount = 0;
        AnimationTags = Array.Empty<string>();
        AnimationClipCountsByTag = Array.Empty<int>();
        PersistenceValid = false;
    }
}
