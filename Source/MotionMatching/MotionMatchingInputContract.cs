using FlaxEngine;

namespace MotionMatching;

// Plugin-owned input contract (docs/PLUGIN_CONTRACT.md).
//
// Ownership: everything in this file belongs to the motion-matching side.
// Game code implements IMotionMatchingInputProvider; the shell
// (MotionMatchingController) only ever calls through the interface and
// never references game types (PlayerState, coordinators, automation).
//
// Managed DTOs only: MotionMatchingInputContext never crosses the native
// boundary (C# -> C#), so managed arrays are fine here. Only the plain
// String/POD fields copied into MotionMatchingPlayerSnapshot /
// MotionMatchingPolicyTuning cross into C++.

/// <summary>
/// The single call the motion-matching side needs from game-side code,
/// once per tick. The provider owns the player snapshot (pushed game-side
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
    /// Snap scope hysteresis and reseed the predictor. Called by the shell
    /// from ResetPlayback (teleport/respawn). Must not require a transform:
    /// predictor reseed is lazy on the next BuildInput.
    /// </summary>
    void Reset();
}

/// <summary>
/// MM-side input to the provider. No raw movement here on purpose (R5):
/// movement arrives game-side through the provider's own Update method,
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
/// Movement is copied straight into the native snapshot; scope strings are
/// '+'-joined and split natively (the scripting marshaller cannot move
/// Array-String struct fields across the boundary).
/// P2: movement + trajectory + scope only. All gameplay decisions (gait
/// roles, speed ceilings, yaw/direction thresholds, driver flags, overlay
/// angles/gates) live game-side in the provider/snapshot and never cross
/// into the core. Empty AllowedTagSet = explicit no-match.
/// </summary>
public struct MotionMatchingFrameInput
{
    // 1. Movement (snapshot movement block).
    public Float3 Velocity;
    public Float3 DesiredVelocity;
    public Float3 Acceleration;
    public Float3 AngularVelocity;
    public float TargetYaw;
    public float CurrentYaw;
    public bool IsGrounded;
    public bool IsSprinting;

    // 2. Predicted trajectory, character-local, aligned 1:1 with the
    // sample times from the context. Empty/null = prediction failed;
    // the core skips the query exactly like today's trajError path.
    public TrajectoryPoint[] Points;

    // 3. Scope: '+'-joined include/exclude tag sets decided provider-side.
    // Empty AllowedTagSet = explicit no-match (hold current, never fall
    // back to any pool). The native legacy chooser is deleted, so every
    // query is provider-scoped.
    public string AllowedTagSet;
    public string ExcludedTagSet;
}

// P0 additive-only DTOs for future game drivers (never read by Tick yet).
// Neutral Layer-style naming: no gait/jump/turn/camera vocabulary.

/// <summary>
/// P0: neutral query filter for a future game driver. Plain data only;
/// not consumed by the runtime yet (no Tick change).
/// </summary>
public struct MotionMatchingQueryFilter
{
    public TrajectoryPoint[] Points;
    public string AllowedTags;
    public string ExcludedTags;
}

/// <summary>
/// P0: neutral search tuning for a future game driver. Plain data only;
/// not consumed by the runtime yet (no Tick change).
/// </summary>
public struct MotionMatchingSearchTuning
{
    public float PoseWeight;
    public float TrajectoryWeight;
    public float InertializationDuration;
    public int FrameSearchBudget;
}
