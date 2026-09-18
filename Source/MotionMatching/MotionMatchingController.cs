using System;
using System.Collections.Generic;
using System.IO;
using FlaxEngine;

namespace MotionMatching;

/// <summary>
/// Scene-facing settings and status for the native motion-matching system.
///
/// Plugin boundary (MM_PLUGIN_PLAN.md): this shell owns the database,
/// policy/playback lifecycle and core-only tuning. Everything locomotion
/// (gait scope, trajectory prediction, gait speeds) arrives through the
/// single IMotionMatchingInputProvider slot below. This file must never
/// reference game types (PlayerState, coordinators, automation, overlays).
/// </summary>
public class MotionMatchingController : Script
{

    /// <summary>
    /// Bake contract authored in the scene. A snapshot is stored inside the database on bake;
    /// </summary>
    [EditorOrder(0), EditorDisplay("Schema"),
     Tooltip("Search/bake contract. Skeleton bone names ship EMPTY by design: configure them for your rig (Schema → Skeleton). An empty profile fails fast with the exact scene path instead of silently using mannequin names.")]
    public PoseSearchSchema Schema = CreateDefaultSchema();

    [ExpandGroups]
    [EditorOrder(10), EditorDisplay("Model")]
    public SkinnedModel PlayerModel;

    [ExpandGroups]
    [EditorOrder(20), EditorDisplay("Database")]
    public string DatabasePath = "MotionMatching/MotionMatchingDatabase.flax";

    [EditorOrder(21), EditorDisplay("Database"), ReadOnly]
    public MotionMatchingDatabase Database;

    [EditorOrder(22), EditorDisplay("Database"), ReadOnly]
    public int AnimationClipsCount;

    [EditorOrder(23), EditorDisplay("Database"), ReadOnly]
    public int SampleCount;

    [EditorOrder(24), EditorDisplay("Database"), ReadOnly]
    public bool PersistenceValid;

    [HideInEditor]
    public string[] AnimationTags = Array.Empty<string>();

    [HideInEditor]
    public int[] AnimationClipCountsByTag = Array.Empty<int>();

    [ExpandGroups]
    [EditorOrder(30), EditorDisplay("Input"),
     Tooltip("Game-side input provider (Flax inspector cannot show interface fields, so this is a Script slot cast to IMotionMatchingInputProvider). Null = tick skipped with a warning; the game driver assigns its provider here.")]
    public Script InputProvider;

    // Host-assigned session config (set by the game driver, e.g. from
    // automation flags — the shell never reads automation itself).
    [HideInEditor]
    public string CacheFolder = string.Empty;
    [HideInEditor]
    public bool TelemetryOverride = false;
    [HideInEditor]
    public bool IndexedVerifyAuto = false;

    [ExpandGroups]
    [EditorOrder(40), EditorDisplay("Playback")]
    public AnimatedModel CharacterBody;

    [EditorOrder(41), EditorDisplay("Playback")]
    public float QueryInterval = 0.1f;

    // Per-frame search: the query gate runs every warmed tick instead of on
    // the QueryInterval cadence, so input latency never waits for a tick
    // boundary. Full scans stay budgeted (FrameSearchBudget) with a rotating
    // stride + continuation exemption, and intent-change queries always scan
    // unstrided — same total CPU as cadence, minimal worst-case latency.
    [EditorOrder(41), EditorDisplay("Playback")]
    public bool SearchEveryFrame = true;

    // Samples scored per full scan (rotating stride covers the rest over
    // following frames). 12000 ≈ 1ms on the reference machine. 0 = unstrided.
    [EditorOrder(41), EditorDisplay("Playback")]
    public int FrameSearchBudget = 12000;

    // Phase 3 (KDTREE_PLAN.md): indexed path is live by default (phase-2
    // audit: 0.33% same-clip-twin divergence, suite parity gate below).
    // Brute force stays as fallback + permanent verify harness.
    [EditorOrder(41), EditorDisplay("Playback")]
    public bool UseIndexedSearch = true;

    // Candidates retrieved per tag before exact rerank. 16 (measured
    // 2026-09-14: sub-1ms scans, 0/1500 verify divergence, 12/12 suite).
    [EditorOrder(41), EditorDisplay("Playback")]
    public int TopKCandidates = 16;

    // Phase 2 (KDTREE_PLAN.md): run brute force alongside the indexed path,
    // count divergences, keep the brute-force winner. Permanent debug harness.
    [EditorOrder(41), EditorDisplay("Playback")]
    public bool VerifyIndexedSearch = false;

    // Single foot-IK switch (positive logic): ticked = IK may engage when
    // moving (Wave C motion gate still applies). Replaces the old
    // DisableFootIK kill-switch + the foot-ik branch force-enable override.
    [EditorOrder(45), EditorDisplay("Playback"), Tooltip("Enable foot IK (locks engage only while moving/turning).")]
    public bool EnableFootIK = true;

    // NOTE: layer decisions (procedural angle, bone gate) live
    // in the input provider now (P4/P6). The shell forwards its decided
    // LayerAngleDeg/GateBone verbatim; see MotionMatchingFrameInput.

    // Switch penalty added to any different-clip winner (continuity block).
    // Rev-3 cost units (N1b shrank dissimilarity ~100x): 0.0025 restores the
    // old ratio against typical winner costs (~0.01-0.2 measured).
    [EditorOrder(47), EditorDisplay("Playback")]
    public float ContinuityWeight = 0.0025f;

    [EditorOrder(48), EditorDisplay("Playback")]
    public float MinSwitchTime = 0.12f;

    // Continuation stickiness (live tuning, no rebake): bias rewards the
    // natural next frames on the current clip, margin forces other clips to
    // beat them clearly. Rev-3 cost units: N1b shrank dissimilarity ~100x
    // (measured mean winner cost 0.095), so both scale down equally to keep
    // the old hold/switch ratios. UE philosophy is the same direction (tiny
    // -0.01 bias + interrupt logic instead of giant margins).
    [EditorOrder(49), EditorDisplay("Playback")]
    public float ContinuingBias = 0.012f;

    [EditorOrder(50), EditorDisplay("Playback")]
    public float ContinuingWindow = 0.15f;

    [EditorOrder(51), EditorDisplay("Playback")]
    public float SwitchMargin = 0.008f;

    // foot-ik branch flip work: fair-fight intent accumulator trip level
    // (u/s accumulated with 0.7/tick decay) and unbiased-scan latch window
    // (seconds). A 45deg walk flip slews ~20u/s per tick and trips in 2-3
    // ticks; steady-motion noise stays an order of magnitude below.
    [EditorOrder(51), EditorDisplay("Playback")]
    public float FairFightAccum = 40.0f;
    [EditorOrder(51), EditorDisplay("Playback")]
    public float FairFightWindow = 0.25f;

    // Heuristic thresholds below were hardcoded constants (codex review):
    // exposed live so tuning moves numbers instead of code. No rebake needed.
    [EditorOrder(52), EditorDisplay("Playback")]
    public float SettleToLoopDelay = 0.3f;

    // Rest-loop preference (loops win ties at rest so one-shot breaks/stops
    // cannot re-win forever on exact pose match). Rev-3 units: rest gaps run
    // ~0.005-0.02 measured, so 0.02 settles rest without touching transitions
    // (preference is zero unless restCalm). Raised from 0.005 after measured
    // Idle_Break churn in suite phase 10.
    [EditorOrder(53), EditorDisplay("Playback")]
    public float RestLoopPreference = 0.02f;

    [EditorOrder(57), EditorDisplay("Playback")]
    public float SameClipExclusion = 0.25f;

    [EditorOrder(58), EditorDisplay("Playback")]
    public float SameClipSeekThreshold = 0.25f;

    // Longer switch lock while turn tags are active: turn entry trades
    // near-twin one-shots every query, so the first turn win holds longer.
    // First response stays immediate (lock only bites AFTER a switch).
    [EditorOrder(59), EditorDisplay("Playback")]
    public float TurnMinSwitchTime = 0.3f;

    // Turn-drive trigger: facing error (degrees) past this starts a committed
    // in-place turn. Low enough that the spring-skate window (idle loop
    // yawing under a planted stance) stays small; early exit caps overplay.
    [EditorOrder(60), EditorDisplay("Playback")]
    public float TurnDriveAngle = 25.0f;

    // Reactive bucket: need = |error| + cameraRate * lookahead, matched to
    // the clip whose measured turn angle is closest (fast flicks pick bigger
    // turns). Camera rate, not body: body lags, camera is the intent.
    [EditorOrder(61), EditorDisplay("Playback")]
    public float TurnLookahead = 0.2f;

    // Flying-vs-settled split (rad/s on the CAMERA yaw): while flying, error
    // growth is expected, so the wrong-way fuse stays off; once settled the
    // fuse arms and drives finish exactly.
    [EditorOrder(62), EditorDisplay("Playback")]
    public float TurnSettleRate = 2.0f;

    // Patient trigger for slow sustained turns (steady error never reaches
    // TurnDriveAngle): fire once the error holds past the small angle this long.
    [EditorOrder(63), EditorDisplay("Playback")]
    public float TurnPatientAngle = 15.0f;

    [EditorOrder(64), EditorDisplay("Playback")]
    public float TurnPatientTime = 1.0f;

    // Cost fuse: a switched winner above this is treated as a corrupted query
    // (observed: single-query pose spike mid steady walk, self-heals next
    // query) — hold current instead of cascading. Rev-3 units: legit switches
    // peak ~0.9 measured, so 2.0 keeps headroom without ever firing legally.
    [EditorOrder(65), EditorDisplay("Playback")]
    public float CostFuseThreshold = 2.0f;

    // Inertialization decay time (replaces linear-crossfade BlendDuration;
    // single path, no parallel blend modes).
    [EditorOrder(49), EditorDisplay("Playback")]
    public float InertializationDuration = 0.25f;

    [EditorOrder(50), EditorDisplay("Playback")]
    public bool TelemetryLog = false;

    // Effective switch: the scene field above never changes at runtime —
    // the host flips TelemetryOverride (e.g. from file-driven automation),
    // so the scene is never dirtied by a play session.
    public bool EffectiveTelemetry => TelemetryLog || TelemetryOverride;

    [EditorOrder(60), EditorDisplay("Telemetry"), ReadOnly]
    public string PolicyState = "Disabled";

    [EditorOrder(61), EditorDisplay("Telemetry"), ReadOnly]
    public int WinnerSample = -1;

    [EditorOrder(62), EditorDisplay("Telemetry"), ReadOnly]
    public int WinnerClip = -1;

    [EditorOrder(63), EditorDisplay("Telemetry"), ReadOnly]
    public string WinnerClipName = string.Empty;

    [EditorOrder(63), EditorDisplay("Telemetry"), ReadOnly]
    public float WinnerCost = float.MaxValue;

    [EditorOrder(64), EditorDisplay("Telemetry"), ReadOnly]
    public float SecondBestCost = float.MaxValue;

    [EditorOrder(65), EditorDisplay("Telemetry"), ReadOnly]
    public float WinnerPoseCost = float.MaxValue;

    [EditorOrder(66), EditorDisplay("Telemetry"), ReadOnly]
    public float WinnerTrajectoryCost = float.MaxValue;

    [EditorOrder(67), EditorDisplay("Telemetry"), ReadOnly]
    public float WinnerContinuityCost;

    [EditorOrder(68), EditorDisplay("Telemetry"), ReadOnly]
    public float WinnerLoopCost;

    [EditorOrder(69), EditorDisplay("Telemetry"), ReadOnly]
    public int SwitchCount;

    [EditorOrder(70), EditorDisplay("Telemetry"), ReadOnly]
    public int SeekCount;

    [EditorOrder(71), EditorDisplay("Telemetry"), ReadOnly]
    public float LastQueryMs;

    [EditorOrder(72), EditorDisplay("Telemetry"), ReadOnly]
    public string ActiveTag = string.Empty;

    [EditorOrder(72), EditorDisplay("Telemetry"), ReadOnly]
    public bool TurnDriveActive;

    [EditorOrder(72), EditorDisplay("Telemetry"), ReadOnly]
    public float TurnDriveYaw;

    [EditorOrder(73), EditorDisplay("Telemetry"), ReadOnly]
    public bool WinnerIsLoop;

    [EditorOrder(74), EditorDisplay("Telemetry"), ReadOnly]
    public float LoopPreferenceApplied;

    [EditorOrder(75), EditorDisplay("Telemetry"), ReadOnly]
    public float BestLoopCost = float.MaxValue;

    [EditorOrder(76), EditorDisplay("Telemetry"), ReadOnly]
    public string BestLoopClipName = string.Empty;

    [EditorOrder(77), EditorDisplay("Telemetry"), ReadOnly]
    public float BestOneShotCost = float.MaxValue;

    [EditorOrder(78), EditorDisplay("Telemetry"), ReadOnly]
    public string BestOneShotClipName = string.Empty;

    [EditorOrder(79), EditorDisplay("Telemetry"), ReadOnly]
    public int LoopScoredCount;

    [EditorOrder(80), EditorDisplay("Telemetry"), ReadOnly]
    public int OneShotScoredCount;

    [EditorOrder(81), EditorDisplay("Telemetry"), ReadOnly]
    public int LoopTotalCount;

    [EditorOrder(82), EditorDisplay("Telemetry"), ReadOnly]
    public int OneShotTotalCount;

    [EditorOrder(83), EditorDisplay("Telemetry"), ReadOnly]
    public string LastSwitchReason = string.Empty;

    [EditorOrder(84), EditorDisplay("Telemetry"), ReadOnly]
    public string ChooserStatus = string.Empty;

    [EditorOrder(85), EditorDisplay("Telemetry"), ReadOnly]
    public int TraceEntries;

    [EditorOrder(86), EditorDisplay("Telemetry"), ReadOnly]
    public int HoldCount;

    [EditorOrder(87), EditorDisplay("Telemetry"), ReadOnly]
    public int TurnReplayCount;

    [EditorOrder(88), EditorDisplay("Telemetry"), ReadOnly]
    public int SettleCount;

    [EditorOrder(89), EditorDisplay("Telemetry"), ReadOnly]
    public int RecoveryCount;

    [EditorOrder(90), EditorDisplay("Telemetry"), ReadOnly]
    public int JumpCount;

    [EditorOrder(91), EditorDisplay("Telemetry"), ReadOnly]
    public float LayerAngle;

    [EditorOrder(92), EditorDisplay("Telemetry"), ReadOnly]
    public float LayerPrimary;

    [EditorOrder(93), EditorDisplay("Telemetry"), ReadOnly]
    public float LayerSecondary;

    [EditorOrder(93), EditorDisplay("Telemetry"), ReadOnly]
    public bool LayerGateApplied;

    [EditorOrder(94), EditorDisplay("Telemetry"), ReadOnly]
    public bool FootLockL;

    [EditorOrder(95), EditorDisplay("Telemetry"), ReadOnly]
    public bool FootLockR;

    [EditorOrder(96), EditorDisplay("Telemetry"), ReadOnly]
    public float FootCorrL;

    [EditorOrder(97), EditorDisplay("Telemetry"), ReadOnly]
    public float FootCorrR;

    [EditorOrder(98), EditorDisplay("Telemetry"), ReadOnly]
    public int FastPathHits;

    private MotionMatchingRuntimePolicy _policy;
    private bool _telemetryWarned;
    private bool _providerWarned;
    private int _frameId;

    public float[] TrajectorySampleTimes => Database?.TrajectorySampleTimes ?? Schema.TrajectorySampleTimes;

    /// <summary>
    /// Live playback handle for game-side integrations (e.g. pose overlays
    /// attach here). Null until the first tick initializes the policy.
    /// </summary>
    public MotionMatchingPlayback Playback => _policy != null ? _policy.GetPlayback() : null;

    public override void OnEnable()
    {
        base.OnEnable();

        _telemetryWarned = false;

        // Unified load path (no editor partial): Content.Load works in both
        // editor and game builds; the editor service refreshes explicitly
        // after bake/registry edits.
        EnsureDatabaseLoadedForGame();
    }

    /// <summary>
    /// Clears native playback state and snaps the provider (scope + predictor
    /// reseed). Call after teleport/respawn.
    /// </summary>
    public void ResetPlayback()
    {
        _policy?.ResetOnTeleport();
        (InputProvider as IMotionMatchingInputProvider)?.Reset();
    }

    /// <summary>
    /// Policy + playback tick. The game driver pushes fresh player state into
    /// the provider first, then calls this. The provider's frame is the only
    /// input: movement, trajectory, scope and locomotion tuning. Loads the
    /// database on first tick if OnEnable found none.
    /// </summary>
    public void TickPolicy(float deltaTime)
    {
        if (CharacterBody == null)
            return;

        var provider = InputProvider as IMotionMatchingInputProvider;
        // A destroyed script reads as null; a disabled one must not be
        // ticked even though the slot still references it (lifecycle D1:
        // never drive animation from a provider the host switched off).
        bool providerOff = InputProvider != null &&
            InputProvider is Script providerScript && !providerScript.Enabled;
        if (provider == null || providerOff)
        {
            if (!_providerWarned)
            {
                _providerWarned = true;
                Debug.LogWarning(providerOff
                    ? "MotionMatchingController: InputProvider is disabled; animation tick skipped until it is re-enabled or replaced."
                    : "MotionMatchingController: InputProvider slot is empty or does not implement IMotionMatchingInputProvider. Assign a provider (e.g. the locomotion sample); animation tick skipped.");
            }
            return;
        }
        _providerWarned = false;

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
        // scene TelemetryLog field is never written at runtime; the host
        // flips only TelemetryOverride above.
        if (_policy.GetTrace() != null)
            _policy.GetTrace().Enabled = EffectiveTelemetry;
        if (EffectiveTelemetry && !_telemetryWarned)
        {
            _telemetryWarned = true;
            Debug.LogWarning("Motion matching telemetry/trace is ACTIVE (debug-only: per-query clones + file logging). Turn off TelemetryLog for shipping builds.");
        }

        // The single input call: provider owns scope, trajectory and
        // locomotion tuning; the shell only forwards plain data.
        var ctx = new MotionMatchingInputContext
        {
            ActorWorld = Actor.Transform,
            DeltaTime = deltaTime,
            FrameId = ++_frameId,
            DatabaseTags = AnimationTags,
            SampleTimes = TrajectorySampleTimes,
        };
        MotionMatchingFrameInput input = provider.BuildInput(ctx);

        var snapshot = new MotionMatchingPlayerSnapshot
        {
            CurrentVelocity = input.Velocity,
            DesiredVelocity = input.DesiredVelocity,
            AngularVelocity = input.AngularVelocity,
            TargetYaw = input.TargetYaw,
            CurrentYaw = input.CurrentYaw,
            IsGrounded = input.IsGrounded,
            IsSprinting = input.IsSprinting,
            LayerAngleDeg = input.LayerAngleDeg,
            GateBone = input.GateBone,
            ScopeConstrained = input.HasScope,
            AllowedTagSet = input.AllowedTagSet ?? string.Empty,
            ExcludedTagSet = input.ExcludedTagSet ?? string.Empty,
        };
        var tuning = new MotionMatchingPolicyTuning
        {
            PoseWeight = Schema.PoseWeight,
            TrajectoryWeight = Schema.TrajectoryWeight,
            SampleRate = Schema.SampleRate,
            IdleSpeedMax = input.RestSpeedMax,
            WalkSpeedMax = input.JumpWalkSpeedMax,
            RunSpeedMax = input.JumpRunSpeedMax,
            YawRateThreshold = input.YawRateThreshold,
            FairFightAccum = FairFightAccum,
            FairFightWindow = FairFightWindow,
            TagConfirmRepeats = input.TagConfirmRepeats,
            SettleToLoopDelay = SettleToLoopDelay,
            TurnMinSwitchTime = TurnMinSwitchTime,
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
            EnableFootIK = EnableFootIK,
            EnableJumpDriver = input.EnableJumpDriver,
            EnableTurnDrive = input.EnableTurnDrive,
            ContinuityWeight = ContinuityWeight,
            CostFuseThreshold = CostFuseThreshold,
            TurnDriveAngle = TurnDriveAngle,
            TurnLookahead = TurnLookahead,
            TurnPatientAngle = TurnPatientAngle,
            TurnPatientTime = TurnPatientTime,
            TurnSettleRate = TurnSettleRate,
            BackwardThreshold = input.BackwardThreshold,
            LateralThreshold = input.LateralThreshold,
            LateralZRatio = input.LateralZRatio,
            JumpTag = input.JumpTag ?? string.Empty,
            TurnInPlaceTag = input.TurnInPlaceTag ?? string.Empty,
            TurnTravelTag = input.TurnTravelTag ?? string.Empty,
            RestTag = input.RestTag ?? string.Empty,
        };
        _policy.Tick(deltaTime, snapshot,
            input.Points ?? Array.Empty<TrajectoryPoint>(),
            Actor.Transform.GetWorld(), TrajectorySampleTimes, tuning);

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
        TurnDriveActive = _policy.TurnDriveActive;
        TurnDriveYaw = _policy.TurnDriveYaw;
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
        HoldCount = _policy.HoldCount;
        TurnReplayCount = _policy.TurnReplayCount;
        SettleCount = _policy.SettleCount;
        RecoveryCount = _policy.RecoveryCount;
        JumpCount = _policy.JumpCount;
        LayerAngle = _policy.LastLayerRequested;
        LayerPrimary = _policy.LastLayerPrimary;
        LayerSecondary = _policy.LastLayerSecondary;
        LayerGateApplied = _policy.LastLayerGateApplied;
        FootLockL = _policy.LastFootLockL;
        FootLockR = _policy.LastFootLockR;
        FootCorrL = _policy.LastFootCorrL;
        FootCorrR = _policy.LastFootCorrR;
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
