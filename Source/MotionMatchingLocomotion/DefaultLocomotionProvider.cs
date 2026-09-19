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
    private string _scopePendingKey = "idle|0";
    private int _scopePendingCount = 99;
    private bool _scopeReseed;
    private bool _scopeWasAirborne;
    private bool _scopeLatchedBackward;
    private bool _scopeLatchedLateral;
    private Float3 _scopeLastActorPos = Float3.Zero;
    private bool _scopeHasLastActorPos;
    private bool _predictorReseed;
    private readonly HashSet<string> _scopeWarned = new HashSet<string>();
    private string _validatedInventoryKey;

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
        frame.ExcludedTagSet = string.IsNullOrWhiteSpace(GameplayExclude)
            ? string.Empty
            : GameplayExclude.Trim().ToLowerInvariant();
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
        string rest = RestBandTag();
        if (string.IsNullOrEmpty(rest))
        {
            frame.AllowedTagSet = string.Empty;
            ScopeWarnOnce("idle-no-bands",
                "Motion matching provider: no gait bands configured; returning explicit no-match (empty scope).");
        }
        else
        {
            frame.AllowedTagSet = FilterScopeTags(new string[] { rest }, ctx.DatabaseTags, "idle-default");
        }
        frame.ExcludedTagSet = string.IsNullOrWhiteSpace(GameplayExclude)
            ? string.Empty
            : GameplayExclude.Trim().ToLowerInvariant();
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
        _points = _predictor.CalculateTrajectory(
            _lastTransform,
            _snap.CurrentVelocity,
            _snap.Acceleration,
            _snap.AngularVelocity,
            _snap.DesiredVelocity,
            desiredRotation,
            sampleOffsets,
            _snap.PositionHalflife,
            _snap.FacingHalflife,
            ctx.DeltaTime
        ) ?? Array.Empty<TrajectoryPoint>();
    }

    // Rest-band tag (first band by MaxSpeed) for the native idle watchdog.
    private string RestBandTag()
    {
        if (GaitBands == null || GaitBands.Length == 0)
            return string.Empty;
        string rest = string.Empty;
        float best = float.MaxValue;
        foreach (var band in GaitBands)
        {
            if (string.IsNullOrWhiteSpace(band.Tag))
                continue;
            if (band.MaxSpeed < best)
            {
                best = band.MaxSpeed;
                rest = band.Tag.Trim().ToLowerInvariant();
            }
        }
        return rest;
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
        if (GaitBands == null || GaitBands.Length == 0)
            return string.Empty;

        var bands = new List<GaitBand>();
        foreach (var band in GaitBands)
        {
            if (string.IsNullOrWhiteSpace(band.Tag))
                continue;
            bands.Add(new GaitBand { Tag = band.Tag.Trim().ToLowerInvariant(), MaxSpeed = band.MaxSpeed });
        }
        if (bands.Count == 0)
            return string.Empty;
        bands.Sort((a, b) => a.MaxSpeed.CompareTo(b.MaxSpeed));
        string rest = bands[0].Tag;
        float restMax = bands[0].MaxSpeed;
        string top = bands[bands.Count - 1].Tag;

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
            _scopeStable = JumpTag;
            _scopeStableYaw = false;
            _scopePendingKey = JumpTag + "|0";
            _scopePendingCount = 99;
            _scopeReseed = false;
            return FilterScopeTags(new string[] { JumpTag }, ctx.DatabaseTags, "airborne");
        }

        bool backward = ScopeDecelLatchedDirection(true, airborne, planarSpeed, desSpeed, ctx.SampleTimes);
        bool lateral = ScopeDecelLatchedDirection(false, airborne, planarSpeed, desSpeed, ctx.SampleTimes);

        // Direction intent beats speed bands (a fast retreat is "back",
        // never "run"). Sprint intent takes the top band. Otherwise the
        // first band whose ceiling covers the speed wins.
        string linear;
        bool linearIsBand;
        int linearBandIndex;
        if (backward && planarSpeed > restMax && IsKnownRole(BackTag, ctx.DatabaseTags))
        {
            linear = BackTag.Trim().ToLowerInvariant();
            linearIsBand = false;
            linearBandIndex = -1;
        }
        else if (lateral && planarSpeed > restMax && IsKnownRole(StrafeTag, ctx.DatabaseTags))
        {
            linear = StrafeTag.Trim().ToLowerInvariant();
            linearIsBand = false;
            linearBandIndex = -1;
        }
        else if (_snap.IsSprinting && planarSpeed > restMax)
        {
            linear = top;
            linearIsBand = true;
            linearBandIndex = bands.Count - 1;
        }
        else
        {
            linearBandIndex = bands.Count - 1;
            for (int i = 0; i < bands.Count; i++)
            {
                if (planarSpeed <= bands[i].MaxSpeed)
                {
                    linearBandIndex = i;
                    break;
                }
            }
            linear = bands[linearBandIndex].Tag;
            linearIsBand = true;
        }

        bool yaw = yawRate >= YawRateThreshold;
        string key = linear + "|" + (yaw ? "1" : "0");
        string stableKey = _scopeStable + "|" + (_scopeStableYaw ? "1" : "0");
        // Instant adoption on edges and pure band crossings (speed never
        // flickers); everything else waits out TagConfirmRepeats. Non-rest
        // bands only: entering rest stays delayed (natural settle tail).
        int stableBandIndex = BandIndexOf(bands, _scopeStable);
        bool leavingRest = _scopeStable == rest && linear != rest;
        bool bandChange = linearIsBand && linearBandIndex > 0 &&
            stableBandIndex > 0;
        if (edge || key == stableKey || reseed || leavingRest || bandChange)
        {
            _scopeStable = linear;
            _scopeStableYaw = yaw;
            _scopePendingKey = key;
            _scopePendingCount = 99;
            _scopeReseed = false;
        }
        else if (key == _scopePendingKey)
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
            _scopePendingKey = key;
            _scopePendingCount = 1;
        }

        var set = new List<string> { _scopeStable };
        if (_scopeStableYaw)
        {
            string turnRole = _scopeStable == rest ? TurnInPlaceTag : TurnTravelTag;
            if (IsKnownRole(turnRole, ctx.DatabaseTags))
                set.Add(turnRole.Trim().ToLowerInvariant());
        }

        // Gameplay scope from game code intersects the locomotion scope. An
        // empty intersection keeps locomotion (a full-pool fallback would be
        // worse: it could play a clip the game explicitly forbade).
        if (!string.IsNullOrWhiteSpace(GameplayTags))
        {
            var want = new HashSet<string>();
            foreach (string raw in GameplayTags.Split('+'))
            {
                string tag = raw.Trim().ToLowerInvariant();
                if (!string.IsNullOrEmpty(tag))
                    want.Add(tag);
            }
            if (want.Count > 0)
            {
                var both = new List<string>();
                foreach (string tag in set)
                {
                    if (want.Contains(tag))
                        both.Add(tag);
                }
                if (both.Count > 0)
                    set = both;
                else
                    ScopeWarnOnce("gameplay-miss:" + GameplayTags.Trim().ToLowerInvariant(),
                        "Motion matching scope: gameplay tags '" + GameplayTags.Trim() +
                        "' exclude the locomotion scope '" + string.Join("+", set) +
                        "'; keeping locomotion.");
            }
        }

        return FilterScopeTags(set.ToArray(), ctx.DatabaseTags, "scope");
    }

    private void ResetScopeState(string rest)
    {
        _scopeStable = rest;
        _scopeStableYaw = false;
        _scopePendingKey = rest + "|0";
        _scopePendingCount = 99;
        _scopeReseed = true;
        _scopeHasLastActorPos = false;
    }

    private int BandIndexOf(List<GaitBand> bands, string tag)
    {
        for (int i = 0; i < bands.Count; i++)
        {
            if (bands[i].Tag == tag)
                return i;
        }
        return -1;
    }

    // Role known = present in the baked inventory, or inventory unavailable
    // (database not loaded: allow everything, validation happens per-scope).
    private bool IsKnownRole(string role, string[] inventory)
    {
        if (string.IsNullOrWhiteSpace(role))
            return false;
        if (inventory == null || inventory.Length == 0)
            return true;
        string tag = role.Trim().ToLowerInvariant();
        foreach (string known in inventory)
        {
            if (known != null && known.Trim().ToLowerInvariant() == tag)
                return true;
        }
        return false;
    }

    // Drops scope tags missing from the baked inventory (with a warning).
    private string FilterScopeTags(string[] set, string[] inventory, string context)
    {
        if (set == null || set.Length == 0)
            return string.Empty;
        if (inventory == null || inventory.Length == 0)
            return string.Join("+", set);
        var known = new HashSet<string>();
        foreach (string tag in inventory)
        {
            if (!string.IsNullOrWhiteSpace(tag))
                known.Add(tag.Trim().ToLowerInvariant());
        }
        var kept = new List<string>();
        foreach (string raw in set)
        {
            string tag = (raw ?? string.Empty).Trim().ToLowerInvariant();
            if (string.IsNullOrEmpty(tag))
                continue;
            if (kept.Contains(tag))
                continue;
            if (!known.Contains(tag))
            {
                ScopeWarnOnce("unknown-tag:" + tag,
                    "Motion matching scope (" + context + "): tag '" + tag +
                    "' is not in the baked database; dropping it.");
                continue;
            }
            kept.Add(tag);
        }
        return string.Join("+", kept.ToArray());
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
        string key = inventory == null ? "null" : string.Join("+", inventory);
        if (key == _validatedInventoryKey)
            return;
        _validatedInventoryKey = key;
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
