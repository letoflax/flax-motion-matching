using System;
using System.Collections.Generic;
using FlaxEngine;

namespace MotionMatching;

/// <summary>
/// Default locomotion implementation of <see cref="IMotionMatchingInputProvider"/>.
/// Integration sample (NOT part of the motion-matching core): owns a neutral
/// movement snapshot, the gait scope decision (editable bands/roles) and the
/// spring trajectory predictor. Has no player types, camera objects or
/// automation: any host fills <see cref="LocomotionSnapshot"/> from its own
/// state. An empty resolved scope is an explicit no-match (empty scope),
/// never a silent legacy-native fallback.
/// </summary>
public class DefaultLocomotionProvider : Script, IMotionMatchingInputProvider
{
    [ExpandGroups]
    [EditorOrder(0), EditorDisplay("MM Input"),
     Tooltip("Gait bands by planar speed (u/s). Sorted ascending at use; first entry is the rest band. Defaults mirror measured clip speeds: walk loops 198, run loops 493, sprint loops 688.")]
    public GaitBand[] GaitBands = new GaitBand[]
    {
        new GaitBand { Tag = "idle", MaxSpeed = 5.0f },
        new GaitBand { Tag = "walk", MaxSpeed = 200.0f },
        new GaitBand { Tag = "run", MaxSpeed = 490.0f },
        new GaitBand { Tag = "sprint", MaxSpeed = float.MaxValue },
    };

    [EditorOrder(1), EditorDisplay("MM Input"), Tooltip("Airborne scope tag. Must exist in the baked database.")]
    public string JumpTag = "jump";

    [EditorOrder(1), EditorDisplay("MM Input"), Tooltip("Retreat scope tag (direction intent beats speed bands).")]
    public string BackTag = "back";

    [EditorOrder(1), EditorDisplay("MM Input"), Tooltip("Sideways scope tag (sideways-dominated diagonals only; 45° diagonals stay in gait bands).")]
    public string StrafeTag = "strafe";

    [EditorOrder(1), EditorDisplay("MM Input"), Tooltip("Stationary-turn scope tag, appended when yaw exceeds YawRateThreshold at rest.")]
    public string TurnInPlaceTag = "turn_inplace";

    [EditorOrder(1), EditorDisplay("MM Input"), Tooltip("Traveling-turn scope tag, appended when yaw exceeds YawRateThreshold while moving.")]
    public string TurnTravelTag = "turn_travel";

    [EditorOrder(2), EditorDisplay("MM Input"),
     Tooltip("Gameplay scope from game code, '+'-separated (e.g. injured). Empty = none. Final scope = locomotion scope ∩ this; an empty intersection keeps the locomotion scope with a warning. Player scripts set this at runtime.")]
    public string GameplayTags = "";

    [EditorOrder(2), EditorDisplay("MM Input"),
     Tooltip("Gameplay exclusions, '+'-separated. Passed straight to the search every query.")]
    public string GameplayExclude = "";

    [EditorOrder(3), EditorDisplay("MM Input"), Tooltip("Rest ceiling: speeds at/below this latch the rest band and gate coast/hold logic.")]
    public float IdleSpeedMax = 5.0f;

    [EditorOrder(3), EditorDisplay("MM Input"), Tooltip("Game-driver Walk ceiling (game JumpDriver bucket). Warns when it drifts from the bands.")]
    public float JumpWalkSpeedMax = 200.0f;

    [EditorOrder(3), EditorDisplay("MM Input"), Tooltip("Game-driver Run ceiling (game JumpDriver bucket). Warns when it drifts from the bands.")]
    public float JumpRunSpeedMax = 490.0f;

    [EditorOrder(4), EditorDisplay("MM Input")]
    public float YawRateThreshold = 0.5f;

    [EditorOrder(4), EditorDisplay("MM Input")]
    public int TagConfirmRepeats = 2;

    [EditorOrder(5), EditorDisplay("MM Input")]
    public float BackwardThreshold = 30.0f;

    [EditorOrder(5), EditorDisplay("MM Input")]
    public float LateralThreshold = 30.0f;

    [EditorOrder(5), EditorDisplay("MM Input")]
    public float LateralZRatio = 0.58f;

    // P2: overlay inputs are consumed game-side by the facade post-pass
    // (the game player reads them from its own per-frame input copy),
    // never forwarded into the core. The fields below died with the core
    // overlay seam: EnablePoseLayer, LayerLowSpeedOnly, LayerSpeedGateMult,
    // EnableBoneGate, EnableJumpDriver, EnableTurnDrive.

    [EditorOrder(7), EditorDisplay("MM Input")]
    public bool DrawDebugTrajectory = true;

    // Latest movement snapshot, pushed host-side every frame before the tick.
    private LocomotionSnapshot _snap;
    private bool _hasSnap;

    private MotionMatchingTrajectory _predictor;
    private TrajectoryPoint[] _points = Array.Empty<TrajectoryPoint>();
    private float[] _lastSampleTimes = Array.Empty<float>();
    private Transform _lastTransform;

    // Chooser state (mirrors the legacy native SelectTags hysteresis exactly,
    // but resolves through the editable GaitBands/roles above).
    private string _scopeStable = "idle";
    private bool _scopeStableYaw;
    private string _scopePendingTag = "idle";
    private bool _scopePendingYaw;
    private int _scopePendingCount = 99;
    private bool _scopeReseed;
    private bool _scopeWasAirborne;
    private bool _scopeLatchedBackward;
    private bool _scopeLatchedLateral;
    private Float3 _scopeLastActorPos = Float3.Zero;
    private bool _scopeHasLastActorPos;
    private bool _predictorReseed;
    // P5 steady-state reuse (all bounded, scope semantics unchanged):
    // gait/inventory/role caches below are rebuilt only when their source
    // config changes (ordinal/value compares, no allocation on compare);
    // steady-state ticks reuse them with no List/HashSet/Split/Join/ToLower.
    // The scope string itself is cached by validated set: same set returns
    // the same string instance (no Join per tick); a changed set Joins once.
    private GaitBand[] _bandsNorm = Array.Empty<GaitBand>();
    private string[] _inventoryLower = Array.Empty<string>();
    private string[] _inventorySrcRef;
    private int _inventorySrcLen = -1;
    private string _roleJumpLower = "jump";
    private string _roleJumpSrc = "jump";
    private string _roleBackLower = "back";
    private string _roleBackSrc = "back";
    private string _roleStrafeLower = "strafe";
    private string _roleStrafeSrc = "strafe";
    private string _roleTurnInPlaceLower = "turn_inplace";
    private string _roleTurnInPlaceSrc = "turn_inplace";
    private string _roleTurnTravelLower = "turn_travel";
    private string _roleTurnTravelSrc = "turn_travel";
    private string _gameplaySrc;
    private string[] _gameplayWant = Array.Empty<string>();
    private string _excludeSrc;
    private string _excludeLower = string.Empty;
    private readonly HashSet<string> _wantScratch = new HashSet<string>(StringComparer.Ordinal);
    private readonly string[] _setScratch = new string[2];
    private readonly string[] _keptScratch = new string[2];
    private string _scopeCache = string.Empty;
    private readonly string[] _scopeKeptCache = new string[2];
    private int _scopeKeptCount = -1;
    private readonly HashSet<string> _scopeWarned = new HashSet<string>();
    private string[] _validatedInventoryRef;
    private int _validatedInventoryLen = -1;

    /// <summary>
    /// Pushes this frame's movement snapshot. Called by the host driver
    /// before the shell ticks. Single writer: the shell never stores
    /// movement itself.
    /// </summary>
    public void UpdateSnapshot(LocomotionSnapshot snapshot)
    {
        _snap = snapshot;
        _hasSnap = true;
    }

    /// <inheritdoc/>
    public MotionMatchingFrameInput BuildInput(MotionMatchingInputContext ctx)
    {
        var frame = new MotionMatchingFrameInput();
        if (!_hasSnap)
            return BuildIdleFrame(ctx);

        // R6 order: trajectory first (scope direction detectors read it),
        // then scope, then the assembled frame.
        PredictTrajectory(ctx);
        ValidateInventory(ctx.DatabaseTags);
        string scope = ComputeTagScope(ctx);

        frame.Points = _points;
        // P2: scope + caller yaw only. Roles/speeds/thresholds/drivers/overlay
        // inputs died with the core machines: the provider keeps them as
        // game-side config for its own scope decision above, but nothing
        // crosses into the core anymore besides trajectory + scope + caller
        // filter. LoopFilter/CandidateBand default Any (the Game selection
        // policy overwrites them per tick in the facade); QueryYawRate rides
        // the snapshot yaw verbatim for the turn prior (Game owns the meaning).
        frame.AllowedTagSet = scope ?? string.Empty;
        // P5: change-only lowered exclusion (no Trim/ToLower per tick).
        frame.ExcludedTagSet = GetExcludedLower();
        frame.LoopFilter = 0;
        frame.CandidateBand = 0;
        float yaw = _snap.AngularVelocity.Y;
        frame.QueryYawRate = yaw >= 0.0f ? yaw : -yaw;
        return frame;
    }

    /// <inheritdoc/>
    public void Reset()
    {
        // Snap the scope on the next tick; the predictor reseeds lazily in
        // BuildInput where a fresh world transform is available.
        _scopeReseed = true;
        _scopeHasLastActorPos = false;
        _predictorReseed = true;
    }

    // Pre-snapshot idle: the provider owns the rest decision from the first
    // tick, so the core never falls back to its legacy chooser. Movement is
    // zero and grounded; scope/roles/speeds/drivers come from config exactly
    // like the normal path. Empty points hold the query (same 6-point guard
    // as a failed prediction) until the first snapshot arrives. Only an
    // empty gait config returns an intentional empty no-match + warning.
    private MotionMatchingFrameInput BuildIdleFrame(MotionMatchingInputContext ctx)
    {
        var frame = new MotionMatchingFrameInput();
        frame.Points = Array.Empty<TrajectoryPoint>();
        frame.LoopFilter = 0;
        frame.CandidateBand = 0;
        frame.QueryYawRate = 0.0f;
        // P5: rest band from the normalized cache (no per-tick scan/alloc).
        EnsureNormalizedBands();
        EnsureInventoryCache(ctx.DatabaseTags);
        string rest = _bandsNorm.Length > 0 ? _bandsNorm[0].Tag : string.Empty;
        if (string.IsNullOrEmpty(rest))
        {
            frame.AllowedTagSet = string.Empty;
            ScopeWarnOnce("idle-no-bands",
                "Motion matching provider: no gait bands configured; returning explicit no-match (empty scope).");
        }
        else
        {
            frame.AllowedTagSet = FilterScopeSingle(rest, ctx.DatabaseTags, "idle-default");
        }
        frame.ExcludedTagSet = GetExcludedLower();
        return frame;
    }

    public override void OnEnable()
    {
        base.OnEnable();
        InitPredictor();
    }

    public override void OnDisable()
    {
        CleanupPredictor();
        base.OnDisable();
    }

    public override void OnDestroy()
    {
        CleanupPredictor();
        base.OnDestroy();
    }

    private void InitPredictor()
    {
        if (_predictor == null && Actor != null)
        {
            _predictor = new MotionMatchingTrajectory();
            _predictor.Reset(Actor.Transform);
        }
    }

    private void CleanupPredictor()
    {
        if (_predictor)
        {
            Destroy(_predictor);
            _predictor = null;
        }
    }

    private void PredictTrajectory(MotionMatchingInputContext ctx)
    {
        if (_predictor == null)
            InitPredictor();
        if (_predictor == null)
        {
            _points = Array.Empty<TrajectoryPoint>();
            return;
        }
        if (_predictorReseed)
        {
            _predictorReseed = false;
            _predictor.Reset(ctx.ActorWorld);
        }

        // Desired facing comes from look yaw so strafing keeps facing forward.
        // TargetYaw leads CurrentYaw, giving the spring a goal ahead of the current pose.
        Quaternion desiredRotation = Quaternion.Euler(0.0f, _snap.TargetYaw, 0.0f);

        float[] sampleOffsets = ctx.SampleTimes;
        _lastTransform = ctx.ActorWorld;
        _lastSampleTimes = sampleOffsets ?? Array.Empty<float>();
        // P5 zero-alloc trajectory read: UpdateTrajectory runs the same
        // prediction as CalculateTrajectory but returns void (no managed
        // array allocation at the binding). Points are copied back into the
        // reused _points buffer; resize only when the schema point count
        // changes. Bound: schema point count (6, hard cap 16). Semantics
        // identical (same native core, same _points content/order).
        _predictor.UpdateTrajectory(
            _lastTransform,
            _snap.CurrentVelocity,
            _snap.Acceleration,
            _snap.AngularVelocity,
            _snap.DesiredVelocity,
            desiredRotation,
            sampleOffsets,
            _snap.PositionHalflife,
            _snap.FacingHalflife,
            ctx.DeltaTime);
        int n = _predictor.GetPredictedCount();
        if (n < 0 || n > 16)
            n = 0;
        if (_points.Length != n)
            _points = new TrajectoryPoint[n];
        for (int i = 0; i < n; i++)
        {
            TrajectoryPoint p;
            if (_predictor.GetPredictedPoint(i, out p))
                _points[i] = p;
            else
                _points[i] = new TrajectoryPoint();
        }
    }

    // P5: normalized band cache (trimmed + lowered, ascending MaxSpeed).
    // Rebuilt only when the GaitBands content changes. Change detection is
    // allocation-free steady-state: array reference + length + per-item
    // string-reference/MaxSpeed compares (strings are immutable, so a
    // reference change IS a content change; rebuilt on any difference).
    private GaitBand[] _bandsSrcRef;
    private int _bandsNormSrcLen = -1;
    private string[] _bandsRawTags = Array.Empty<string>();
    private float[] _bandsRawMax = Array.Empty<float>();

    private void EnsureNormalizedBands()
    {
        GaitBand[] src = GaitBands;
        if (src != null && ReferenceEquals(src, _bandsSrcRef) && src.Length == _bandsNormSrcLen && BandsItemsEqual(src))
            return;
        RebuildNormalizedBands(src);
    }

    private bool BandsItemsEqual(GaitBand[] src)
    {
        if (_bandsRawTags.Length != src.Length)
            return false;
        for (int i = 0; i < src.Length; i++)
        {
            if (!ReferenceEquals(src[i].Tag, _bandsRawTags[i]) || src[i].MaxSpeed != _bandsRawMax[i])
                return false;
        }
        return true;
    }

    private void RebuildNormalizedBands(GaitBand[] src)
    {
        _bandsSrcRef = src;
        _bandsNormSrcLen = src != null ? src.Length : -1;
        if (src != null)
        {
            if (_bandsRawTags.Length != src.Length)
            {
                _bandsRawTags = new string[src.Length];
                _bandsRawMax = new float[src.Length];
            }
            for (int i = 0; i < src.Length; i++)
            {
                _bandsRawTags[i] = src[i].Tag;
                _bandsRawMax[i] = src[i].MaxSpeed;
            }
        }
        int n = CountValidBands(src);
        if (_bandsNorm.Length != n)
            _bandsNorm = new GaitBand[n];
        if (src == null)
        {
            _scopeKeptCount = -1;
            return;
        }
        int w = 0;
        for (int i = 0; i < src.Length; i++)
        {
            string tag = src[i].Tag;
            if (string.IsNullOrWhiteSpace(tag))
                continue;
            _bandsNorm[w++] = new GaitBand { Tag = tag.Trim().ToLowerInvariant(), MaxSpeed = src[i].MaxSpeed };
        }
        // Insertion sort by MaxSpeed (band counts are tiny: 4 by default).
        for (int i = 1; i < n; i++)
        {
            GaitBand key = _bandsNorm[i];
            int j = i - 1;
            while (j >= 0 && _bandsNorm[j].MaxSpeed > key.MaxSpeed)
            {
                _bandsNorm[j + 1] = _bandsNorm[j];
                j--;
            }
            _bandsNorm[j + 1] = key;
        }
        _scopeKeptCount = -1;
    }

    private static int CountValidBands(GaitBand[] src)
    {
        if (src == null)
            return 0;
        int n = 0;
        for (int i = 0; i < src.Length; i++)
        {
            if (!string.IsNullOrWhiteSpace(src[i].Tag))
                n++;
        }
        return n;
    }

    // P5: lowered inventory cache. Change detection is reference + length
    // (the inventory array is set once per database load, never mutated in
    // place). Rebuild also drops the scope-string cache: the validated set
    // may change with the inventory. Returns true when rebuilt.
    private bool EnsureInventoryCache(string[] inventory)
    {
        if (ReferenceEquals(inventory, _inventorySrcRef) &&
            (inventory == null || inventory.Length == _inventorySrcLen))
            return false;
        _inventorySrcRef = inventory;
        _inventorySrcLen = inventory != null ? inventory.Length : -1;
        int n = 0;
        if (inventory != null)
        {
            for (int i = 0; i < inventory.Length; i++)
            {
                if (!string.IsNullOrWhiteSpace(inventory[i]))
                    n++;
            }
        }
        if (_inventoryLower.Length != n)
            _inventoryLower = new string[n];
        int w = 0;
        if (inventory != null)
        {
            for (int i = 0; i < inventory.Length; i++)
            {
                if (string.IsNullOrWhiteSpace(inventory[i]))
                    continue;
                _inventoryLower[w++] = inventory[i].Trim().ToLowerInvariant();
            }
        }
        _scopeKeptCount = -1;
        return true;
    }

    // P5: lowered role cache (Jump/Back/Strafe/TurnInPlace/TurnTravel).
    // Rebuilt only when a role source string changes (value compare, no
    // allocation); steady-state ticks reuse the lowered copies.
    private void EnsureRoles()
    {
        if (JumpTag != _roleJumpSrc)
        {
            _roleJumpSrc = JumpTag;
            _roleJumpLower = string.IsNullOrWhiteSpace(JumpTag) ? string.Empty : JumpTag.Trim().ToLowerInvariant();
        }
        if (BackTag != _roleBackSrc)
        {
            _roleBackSrc = BackTag;
            _roleBackLower = string.IsNullOrWhiteSpace(BackTag) ? string.Empty : BackTag.Trim().ToLowerInvariant();
        }
        if (StrafeTag != _roleStrafeSrc)
        {
            _roleStrafeSrc = StrafeTag;
            _roleStrafeLower = string.IsNullOrWhiteSpace(StrafeTag) ? string.Empty : StrafeTag.Trim().ToLowerInvariant();
        }
        if (TurnInPlaceTag != _roleTurnInPlaceSrc)
        {
            _roleTurnInPlaceSrc = TurnInPlaceTag;
            _roleTurnInPlaceLower = string.IsNullOrWhiteSpace(TurnInPlaceTag) ? string.Empty : TurnInPlaceTag.Trim().ToLowerInvariant();
        }
        if (TurnTravelTag != _roleTurnTravelSrc)
        {
            _roleTurnTravelSrc = TurnTravelTag;
            _roleTurnTravelLower = string.IsNullOrWhiteSpace(TurnTravelTag) ? string.Empty : TurnTravelTag.Trim().ToLowerInvariant();
        }
    }

    // P5: parsed gameplay-want cache. Split/trim/lower run only when the
    // GameplayTags source string changes (rare: driver-forced scopes).
    private void EnsureGameplayWant()
    {
        if (GameplayTags == _gameplaySrc)
            return;
        _gameplaySrc = GameplayTags;
        if (string.IsNullOrWhiteSpace(GameplayTags))
        {
            _gameplayWant = Array.Empty<string>();
            return;
        }
        string[] parts = GameplayTags.Split('+');
        int n = 0;
        for (int i = 0; i < parts.Length; i++)
        {
            if (!string.IsNullOrWhiteSpace(parts[i]))
                n++;
        }
        string[] want = new string[n];
        int w = 0;
        for (int i = 0; i < parts.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(parts[i]))
                continue;
            want[w++] = parts[i].Trim().ToLowerInvariant();
        }
        _gameplayWant = want;
    }

    // P5: lowered exclusion cache for the frame ExcludedTagSet. Rebuilt
    // only when GameplayExclude changes; blank stays string.Empty.
    private string GetExcludedLower()
    {
        if (GameplayExclude == _excludeSrc)
            return _excludeLower;
        _excludeSrc = GameplayExclude;
        _excludeLower = string.IsNullOrWhiteSpace(GameplayExclude)
            ? string.Empty
            : GameplayExclude.Trim().ToLowerInvariant();
        return _excludeLower;
    }

    /// <summary>
    /// Provider-decided tag scope for one tick, '+'-joined. Mirrors the
    /// legacy native chooser exactly (direction latch, band lookup, yaw bit,
    /// hysteresis) but resolves through the editable GaitBands/roles above
    /// instead of hardcoded literals. Empty = explicit no-match downstream.
    /// Unknown tags are dropped with a warning (validate-then-drop).
    /// </summary>
    private string ComputeTagScope(MotionMatchingInputContext ctx)
    {
        // P5: normalized bands/roles/inventory come from the change-only
        // caches above (no List/HashSet/Split/Join/ToLower per tick).
        EnsureNormalizedBands();
        EnsureRoles();
        EnsureInventoryCache(ctx.DatabaseTags);
        if (_bandsNorm.Length == 0)
            return string.Empty;

        string rest = _bandsNorm[0].Tag;
        float restMax = _bandsNorm[0].MaxSpeed;
        string top = _bandsNorm[_bandsNorm.Length - 1].Tag;

        float planarSpeed = (float)Math.Sqrt(
            _snap.CurrentVelocity.X * _snap.CurrentVelocity.X +
            _snap.CurrentVelocity.Z * _snap.CurrentVelocity.Z);
        float desSpeed = (float)Math.Sqrt(
            _snap.DesiredVelocity.X * _snap.DesiredVelocity.X +
            _snap.DesiredVelocity.Z * _snap.DesiredVelocity.Z);
        float yawRate = Math.Abs(_snap.AngularVelocity.Y);
        bool airborne = !_snap.IsGrounded;

        bool edge = airborne != _scopeWasAirborne;
        _scopeWasAirborne = airborne;

        // Teleport self-detect (mirrors the native 2000u guard): a position
        // jump snaps the scope instead of waiting out the hysteresis.
        bool reseed = _scopeReseed;
        Float3 actorPos = Actor != null ? Actor.Transform.Translation : Float3.Zero;
        if (_scopeHasLastActorPos)
        {
            Float3 jump = actorPos - _scopeLastActorPos;
            if (jump.LengthSquared > 2000.0f * 2000.0f)
            {
                reseed = true;
                ResetScopeState(rest);
            }
        }
        _scopeLastActorPos = actorPos;
        _scopeHasLastActorPos = true;

        if (airborne)
        {
            _scopeStable = _roleJumpLower;
            _scopeStableYaw = false;
            _scopePendingTag = _roleJumpLower;
            _scopePendingYaw = false;
            _scopePendingCount = 99;
            _scopeReseed = false;
            return FilterScopeSingle(_roleJumpLower, ctx.DatabaseTags, "airborne");
        }

        bool backward = ScopeDecelLatchedDirection(true, airborne, planarSpeed, desSpeed, ctx.SampleTimes);
        bool lateral = ScopeDecelLatchedDirection(false, airborne, planarSpeed, desSpeed, ctx.SampleTimes);

        // Direction intent beats speed bands (a fast retreat is "back",
        // never "run"). Sprint intent takes the top band. Otherwise the
        // first band whose ceiling covers the speed wins.
        string linear;
        bool linearIsBand;
        int linearBandIndex;
        if (backward && planarSpeed > restMax && IsKnownRoleLower(_roleBackLower, ctx.DatabaseTags))
        {
            linear = _roleBackLower;
            linearIsBand = false;
            linearBandIndex = -1;
        }
        else if (lateral && planarSpeed > restMax && IsKnownRoleLower(_roleStrafeLower, ctx.DatabaseTags))
        {
            linear = _roleStrafeLower;
            linearIsBand = false;
            linearBandIndex = -1;
        }
        else if (_snap.IsSprinting && planarSpeed > restMax)
        {
            linear = top;
            linearIsBand = true;
            linearBandIndex = _bandsNorm.Length - 1;
        }
        else
        {
            linearBandIndex = _bandsNorm.Length - 1;
            for (int i = 0; i < _bandsNorm.Length; i++)
            {
                if (planarSpeed <= _bandsNorm[i].MaxSpeed)
                {
                    linearBandIndex = i;
                    break;
                }
            }
            linear = _bandsNorm[linearBandIndex].Tag;
            linearIsBand = true;
        }

        bool yaw = yawRate >= YawRateThreshold;
        // P5: hysteresis keys compared as (tag, yaw-bit) pairs instead of
        // joined "tag|0/1" strings (the old key concat allocated per tick).
        // _scopeStable/_scopePendingTag hold normalized tags, so ordinal
        // value compares are exact with no allocation.
        bool sameAsStable = yaw == _scopeStableYaw &&
            string.Equals(linear, _scopeStable, StringComparison.Ordinal);
        // Instant adoption on edges and pure band crossings (speed never
        // flickers); everything else waits out TagConfirmRepeats. Non-rest
        // bands only: entering rest stays delayed (natural settle tail).
        int stableBandIndex = BandIndexOf(_scopeStable);
        bool leavingRest = string.Equals(_scopeStable, rest, StringComparison.Ordinal) &&
            !string.Equals(linear, rest, StringComparison.Ordinal);
        bool bandChange = linearIsBand && linearBandIndex > 0 &&
            stableBandIndex > 0;
        if (edge || sameAsStable || reseed || leavingRest || bandChange)
        {
            _scopeStable = linear;
            _scopeStableYaw = yaw;
            _scopePendingTag = linear;
            _scopePendingYaw = yaw;
            _scopePendingCount = 99;
            _scopeReseed = false;
        }
        else if (yaw == _scopePendingYaw &&
            string.Equals(linear, _scopePendingTag, StringComparison.Ordinal))
        {
            _scopePendingCount++;
            if (_scopePendingCount >= Math.Max(TagConfirmRepeats, 1))
            {
                _scopeStable = linear;
                _scopeStableYaw = yaw;
            }
        }
        else
        {
            _scopePendingTag = linear;
            _scopePendingYaw = yaw;
            _scopePendingCount = 1;
        }

        // P5: two-slot scratch instead of List<string> (stable + one turn
        // role max). Turn roles ride the pre-lowered cache (no per-tick
        // Trim/ToLower); known-check is an ordinal scan of the lowered
        // inventory (no HashSet per tick).
        int setCount = 1;
        _setScratch[0] = _scopeStable;
        if (_scopeStableYaw)
        {
            string turnRole = string.Equals(_scopeStable, rest, StringComparison.Ordinal)
                ? _roleTurnInPlaceLower : _roleTurnTravelLower;
            if (!string.IsNullOrEmpty(turnRole) && IsKnownRoleLower(turnRole, ctx.DatabaseTags))
                _setScratch[setCount++] = turnRole;
        }

        // Gameplay scope from game code intersects the locomotion scope. An
        // empty intersection keeps locomotion (a full-pool fallback would be
        // worse: it could play a clip the game explicitly forbade).
        // P5: the want-set is parsed only when GameplayTags changes (cached);
        // intersection filters the scratch in place (no HashSet/Split per
        // tick; the reused _wantScratch HashSet is Cleared, never rebuilt).
        EnsureGameplayWant();
        if (_gameplayWant.Length > 0)
        {
            _wantScratch.Clear();
            for (int i = 0; i < _gameplayWant.Length; i++)
                _wantScratch.Add(_gameplayWant[i]);
            int keptCount = 0;
            for (int i = 0; i < setCount; i++)
            {
                if (_wantScratch.Contains(_setScratch[i]))
                    _setScratch[keptCount++] = _setScratch[i];
            }
            if (keptCount > 0)
            {
                setCount = keptCount;
            }
            else
            {
                string wantKey = GameplayTags.Trim().ToLowerInvariant();
                string have = setCount > 1
                    ? _setScratch[0] + "+" + _setScratch[1]
                    : (setCount > 0 ? _setScratch[0] : string.Empty);
                ScopeWarnOnce("gameplay-miss:" + wantKey,
                    "Motion matching scope: gameplay tags '" + GameplayTags.Trim() +
                    "' exclude the locomotion scope '" + have +
                    "'; keeping locomotion.");
            }
        }

        return FilterScopeSet(setCount, ctx.DatabaseTags, "scope");
    }

    private void ResetScopeState(string rest)
    {
        _scopeStable = rest;
        _scopeStableYaw = false;
        _scopePendingTag = rest;
        _scopePendingYaw = false;
        _scopePendingCount = 99;
        _scopeReseed = true;
        _scopeHasLastActorPos = false;
    }

    private int BandIndexOf(string tag)
    {
        for (int i = 0; i < _bandsNorm.Length; i++)
        {
            if (string.Equals(_bandsNorm[i].Tag, tag, StringComparison.Ordinal))
                return i;
        }
        return -1;
    }

    // Role known = present in the baked inventory, or inventory unavailable
    // (database not loaded: allow everything, validation happens per-scope).
    // P5: ordinal scan of the lowered inventory cache (no Trim/ToLower per
    // item per tick). The role must already be lowered by the caller.
    private bool IsKnownRoleLower(string loweredRole, string[] inventory)
    {
        if (string.IsNullOrEmpty(loweredRole))
            return false;
        if (inventory == null || inventory.Length == 0)
            return true;
        for (int i = 0; i < _inventoryLower.Length; i++)
        {
            if (string.Equals(_inventoryLower[i], loweredRole, StringComparison.Ordinal))
                return true;
        }
        return false;
    }

    // Single-tag validate path (airborne scope): no array allocation.
    private string FilterScopeSingle(string loweredTag, string[] inventory, string context)
    {
        _setScratch[0] = loweredTag;
        return FilterScopeSet(1, inventory, context);
    }

    // Drops scope tags missing from the baked inventory (with a warning).
    // P5: validates the scratch in place (items are already normalized by
    // the callers) and caches the joined scope string by validated set: an
    // unchanged set returns the same string instance (no Join per tick); a
    // changed set Joins once (max 2 tags: stable + one turn role).
    // Content-identical to the old FilterScopeTags for the same input.
    private string FilterScopeSet(int count, string[] inventory, string context)
    {
        int kept = 0;
        if (inventory == null || inventory.Length == 0)
        {
            for (int i = 0; i < count && kept < 2; i++)
            {
                string tag = _setScratch[i];
                if (string.IsNullOrEmpty(tag))
                    continue;
                bool dup = false;
                for (int k = 0; k < kept; k++)
                {
                    if (string.Equals(_keptScratch[k], tag, StringComparison.Ordinal))
                    {
                        dup = true;
                        break;
                    }
                }
                if (!dup)
                    _keptScratch[kept++] = tag;
            }
        }
        else
        {
            for (int i = 0; i < count && kept < 2; i++)
            {
                string tag = _setScratch[i];
                if (string.IsNullOrEmpty(tag))
                    continue;
                bool dup = false;
                for (int k = 0; k < kept; k++)
                {
                    if (string.Equals(_keptScratch[k], tag, StringComparison.Ordinal))
                    {
                        dup = true;
                        break;
                    }
                }
                if (dup)
                    continue;
                if (!IsKnownRoleLower(tag, inventory))
                {
                    ScopeWarnOnce("unknown-tag:" + tag,
                        "Motion matching scope (" + context + "): tag '" + tag +
                        "' is not in the baked database; dropping it.");
                    continue;
                }
                _keptScratch[kept++] = tag;
            }
        }
        if (kept == _scopeKeptCount)
        {
            bool same = true;
            for (int k = 0; k < kept; k++)
            {
                if (!string.Equals(_keptScratch[k], _scopeKeptCache[k], StringComparison.Ordinal))
                {
                    same = false;
                    break;
                }
            }
            if (same)
                return _scopeCache;
        }
        string scope;
        if (kept == 0)
            scope = string.Empty;
        else if (kept == 1)
            scope = _keptScratch[0];
        else
            scope = _keptScratch[0] + "+" + _keptScratch[1];
        _scopeKeptCount = kept;
        for (int k = 0; k < kept; k++)
            _scopeKeptCache[k] = _keptScratch[k];
        _scopeCache = scope;
        return scope;
    }

    private void ScopeWarnOnce(string key, string message)
    {
        if (_scopeWarned.Contains(key))
            return;
        _scopeWarned.Add(key);
        Debug.LogWarning(message);
    }

    // Validates the editable gait config against the baked database
    // inventory whenever it changes: every band and role tag should exist,
    // or the scope it feeds silently loses candidates.
    private void ValidateInventory(string[] inventory)
    {
        // P5: identity is reference + length (the inventory array is set
        // once per database load and never mutated in place), so the
        // steady-state check allocates nothing (the old string.Join key
        // built a string every tick).
        if (ReferenceEquals(inventory, _validatedInventoryRef) &&
            (inventory == null || inventory.Length == _validatedInventoryLen))
            return;
        _validatedInventoryRef = inventory;
        _validatedInventoryLen = inventory != null ? inventory.Length : -1;
        if (inventory == null || inventory.Length == 0)
            return;
        var known = new HashSet<string>();
        foreach (string tag in inventory)
        {
            if (!string.IsNullOrWhiteSpace(tag))
                known.Add(tag.Trim().ToLowerInvariant());
        }
        if (GaitBands != null)
        {
            foreach (var band in GaitBands)
            {
                string tag = (band.Tag ?? string.Empty).Trim().ToLowerInvariant();
                if (!string.IsNullOrEmpty(tag) && !known.Contains(tag))
                    ScopeWarnOnce("config-band:" + tag,
                        "Motion matching gait config: band tag '" + tag +
                        "' is not in the baked database; its speeds fall through to the next band.");
            }
        }
        CheckRoleConfig(known, JumpTag, "JumpTag");
        CheckRoleConfig(known, BackTag, "BackTag");
        CheckRoleConfig(known, StrafeTag, "StrafeTag");
        CheckRoleConfig(known, TurnInPlaceTag, "TurnInPlaceTag");
        CheckRoleConfig(known, TurnTravelTag, "TurnTravelTag");
    }

    private void CheckRoleConfig(HashSet<string> known, string role, string field)
    {
        string tag = (role ?? string.Empty).Trim().ToLowerInvariant();
        if (!string.IsNullOrEmpty(tag) && !known.Contains(tag))
            ScopeWarnOnce("config-role:" + tag,
                "Motion matching gait config: " + field + " '" + tag +
                "' is not in the baked database; related clips never match.");
    }

    // Direction detector (native IsMovingBackward/IsMovingLateral mirror +
    // the coasting latch): future trajectory points past 0.01s vote; while
    // coasting (no live intent, body still fast) the latched value holds.
    private bool ScopeDecelLatchedDirection(bool backwardAxis, bool airborne, float planarSpeed, float desSpeed, float[] offsets)
    {
        bool coasting = !airborne && desSpeed <= IdleSpeedMax && planarSpeed > IdleSpeedMax;
        if (coasting)
            return backwardAxis ? _scopeLatchedBackward : _scopeLatchedLateral;
        bool value = backwardAxis ? ScopeMovingBackward(offsets) : ScopeMovingLateral(offsets);
        if (!airborne)
        {
            if (backwardAxis)
                _scopeLatchedBackward = value;
            else
                _scopeLatchedLateral = value;
        }
        return value;
    }

    private bool ScopeMovingBackward(float[] offsets)
    {
        if (_points == null || _points.Length == 0)
            return false;
        if (offsets == null || offsets.Length == 0)
            return false;
        float sum = 0.0f;
        int n = 0;
        int count = Math.Min(_points.Length, offsets.Length);
        for (int i = 0; i < count; i++)
        {
            if (offsets[i] > 0.01f)
            {
                sum += _points[i].Position.Z;
                n++;
            }
        }
        return n > 0 && sum / n < -BackwardThreshold;
    }

    private bool ScopeMovingLateral(float[] offsets)
    {
        if (_points == null || _points.Length == 0)
            return false;
        if (offsets == null || offsets.Length == 0)
            return false;
        float sumX = 0.0f;
        float sumZ = 0.0f;
        int n = 0;
        int count = Math.Min(_points.Length, offsets.Length);
        for (int i = 0; i < count; i++)
        {
            if (offsets[i] > 0.01f)
            {
                sumX += _points[i].Position.X;
                sumZ += _points[i].Position.Z;
                n++;
            }
        }
        if (n == 0)
            return false;
        float meanX = Math.Abs(sumX / n);
        float meanZ = sumZ / n;
        if (meanX <= LateralThreshold)
            return false;
        return meanZ <= meanX * Math.Max(LateralZRatio, 0.05f);
    }

    public override void OnDebugDraw()
    {
        base.OnDebugDraw();

        if (!DrawDebugTrajectory || _points == null || _points.Length == 0)
            return;

        var transform = _lastTransform;
        Float3 prevWorldPos = Float3.Zero;
        float[] sampleOffsets = _lastSampleTimes;
        for (int i = 0; i < _points.Length; i++)
        {
            Float3 worldPos = transform.LocalToWorld(_points[i].Position);
            // Ground projection: future points keep the actor's Y by
            // construction and past points ride root height, so raw markers
            // sink into (or float above) terrain. Drop each marker onto the
            // ground; no hit keeps the raw position (never hide the marker).
            worldPos = ProjectDebugToGround(worldPos);

            float offset = (sampleOffsets != null && i < sampleOffsets.Length) ? sampleOffsets[i] : 0.0f;
            Color pointColor = offset < -0.0001f ? Color.Yellow : (Mathf.Abs(offset) <= 0.0001f ? Color.White : Color.Green);
            DebugDraw.DrawSphere(new BoundingSphere(worldPos, 6.0f), pointColor);

            // Facing direction
            Float3 worldFacing = transform.LocalToWorldVector(_points[i].Facing);
            DebugDraw.DrawLine(worldPos, worldPos + worldFacing * 36.0f, Color.Red);

            if (i > 0)
            {
                DebugDraw.DrawLine(prevWorldPos, worldPos, Color.Cyan);
            }
            prevWorldPos = worldPos;
        }
    }

    // Debug-draw only (6 raycasts/frame max): drop a trajectory marker onto
    // the ground plane under it. Self hits (player capsule/mannequin) are
    // rejected by ancestry so the @0 marker doesn't stick to the body.
    private Float3 ProjectDebugToGround(Float3 worldPos)
    {
        try
        {
            Vector3 origin = new Vector3(worldPos.X, worldPos.Y + 40.0f, worldPos.Z);
            Vector3 down = new Vector3(0.0f, -1.0f, 0.0f);
            RayCastHit hit;
            if (Physics.RayCast(origin, down, out hit, 200.0f) && hit.Collider != null && !IsSelfCollider(hit.Collider))
            {
                Vector3 p = hit.Point;
                return new Float3(p.X, p.Y + 4.0f, p.Z);
            }
        }
        catch
        {
            // Debug draw must never break the frame; fall through to raw pos.
        }
        return worldPos;
    }

    private bool IsSelfCollider(PhysicsColliderActor collider)
    {
        try
        {
            // The body is a descendant of this actor, so one ancestry walk
            // covers both (the old CharacterBody check was redundant).
            for (Actor c = collider; c != null; c = c.Parent)
            {
                if (Actor != null && c.ID == Actor.ID)
                    return true;
            }
        }
        catch
        {
        }
        return false;
    }
}
