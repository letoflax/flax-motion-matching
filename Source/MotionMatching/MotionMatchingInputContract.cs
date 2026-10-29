using FlaxEngine;

namespace MotionMatching;

// Plugin-owned input contract (docs/PLUGIN_CONTRACT.md).
//
// Ownership: everything in this file belongs to the motion-matching side.
// The host owns and calls IMotionMatchingInputProvider, then passes the frame
// to MotionMatchingController.Update. The controller has no provider slot and
// never calls BuildInput or Reset on the host's behalf.
//
// Managed DTOs only: MotionMatchingInputContext never crosses the native
// boundary (C# -> C#), so managed arrays are fine here. Only the plain
// String/POD fields copied into MotionMatchingPlayerSnapshot /
// MotionMatchingPolicyTuning cross into C++.

/// <summary>
/// The single call the motion-matching side needs from the host,
/// once per tick. The provider owns the player snapshot (pushed host-side
/// via its own Update method), all hysteresis/latch state, and the
/// trajectory predictor. ctx carries only what the MM side provides:
/// time, world transform, schema inventory and database inventory.
/// Order inside BuildInput is fixed (R6): snapshot -> trajectory ->
/// scope/driver reads the fresh points -> frame.
/// </summary>
public interface IMotionMatchingInputProvider
{
    MotionMatchingFrameInput BuildInput(MotionMatchingInputContext ctx);

    /// <summary>
    /// Snap scope hysteresis and reseed the predictor. The host calls this
    /// alongside controller.ResetPlayback (teleport/respawn). Must not require a transform:
    /// predictor reseed is lazy on the next BuildInput.
    /// </summary>
    void Reset();
}

/// <summary>
/// MM-side input to the provider. No raw movement here on purpose (R5):
/// movement arrives host-side through the provider's own Update method,
/// so there is exactly one writer and no push-then-tick staleness.
/// </summary>
public struct MotionMatchingInputContext
{
    // Actor WORLD transform (Flax Actor.Transform is world space; parent-local
    // would be Actor.LocalTransform). The predictor and the query run in this
    // frame; predicted Points are actor-local offsets. Verified against
    // Engine/Level/Actor.h (Transform = world, LocalTransform = parent-local).
    public Transform ActorWorld;
    public float DeltaTime;
    public int FrameId;
    // Baked tag inventory for validate-then-drop (null = skip validation).
    public string[] DatabaseTags;
    // Trajectory sample offsets, seconds, strictly ascending.
    public float[] SampleTimes;
}

/// <summary>
/// Everything motion matching needs for one tick, produced in one call.
/// Trajectory + scope + caller filter only. All gameplay decisions (gait
/// roles, speed ceilings, yaw/direction thresholds, overlay angles/gates,
/// velocities, grounded/sprint) live host-side in the provider/selection
/// policy and never cross into the core. Includes use tag overlap;
/// exclusions veto any overlap. Empty AllowedTagSet = explicit no-match.
/// Normal queries use the caller filter. Structural reseed/end-recovery
/// searches override LoopFilter to LoopOnly within the same tag scope and
/// exclusions, even for OneShotOnly callers (docs/PLUGIN_CONTRACT.md).
/// </summary>
public struct MotionMatchingFrameInput
{
    // 1. Predicted trajectory, character-local, aligned 1:1 with the
    // sample times from the context. Empty/null = prediction failed;
    // the core skips the query exactly like today's trajError path.
    public TrajectoryPoint[] Points;

    // 2. Scope: '+'-joined include/exclude tag sets decided provider-side.
    // Empty AllowedTagSet = explicit no-match (hold current, never fall
    // back to any pool). Every query is provider-scoped.
    public string AllowedTagSet;
    public string ExcludedTagSet;

    // 3. Caller filter, taken verbatim (the core never infers any of it):
    // LoopFilter mirrors MotionLoopFilter (0 Any, 1 LoopOnly, 2 OneShotOnly);
    // ClipGroup is the one-shot clip group filter: one-shot clips of another
    // non-zero group are skipped, loops and group-0 clips never are (0 = any;
    // groups are authored in the clip registry and the host defines what the
    // ids mean); QueryYawRate (rad/s, >= 0) scales the straightness prior
    // (weight 0 turns the prior off).
    public int LoopFilter;
    public int ClipGroup;
    public float QueryYawRate;

    // 4. Speed matching and reseed hold, both host decisions:
    // TargetSpeed (world units/s, >= 0) is the speed the character really
    // moves at; the clip time bends to match it within the play-rate range of
    // the tuning (0 or below PlayRateMinSpeed = authored speed).
    // HoldReseedFilter keeps the structural reseed recovery (LoopOnly within
    // the same tag scope) armed while true; it is released by the first scan
    // after the host clears it. Set it while the host's own motion state is
    // still settling after ResetPlayback or a teleport.
    public float TargetSpeed;
    public bool HoldReseedFilter;

    // 5. Root event window, a host decision like the caller filter (default
    // off): RootEvent mirrors MotionRootEvent (0 none, 1 takeoff, 2 landing,
    // 3 turn start, 4 stop),
    // RootEventTime is the time left until that event (seconds, negative once
    // it has passed), RootEventTolerance the half-width (seconds, 0 = off).
    // Only candidates whose clip has that root event at the same distance
    // rank, so a landing clip can start where its own touchdown meets the
    // predicted one; a window that admits nothing falls back to the plain
    // query. Event times are derived from the baked root height
    // (MotionMatchingDatabase.GetClipRootTakeoffTime / GetClipRootLandTime).
    public int RootEvent;
    public float RootEventTime;
    public float RootEventTolerance;
}

// Scope crosses into the core only through MotionMatchingFrameInput above;
// intent, loop and group mirrors stay host-side. No native struct field is
// added for them (MotionMatchingTraceEntry stays under the Flax 1.12 bindings
// cap of 64).
