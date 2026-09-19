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
    /// Snap scope hysteresis and reseed the predictor. The host calls this
    /// alongside controller.ResetPlayback (teleport/respawn). Must not require a transform:
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
/// P2: trajectory + scope + caller filter only. All gameplay decisions (gait
/// roles, speed ceilings, yaw/direction thresholds, driver flags, overlay
/// angles/gates, velocities, grounded/sprint) live game-side in the provider/
/// selection policy and never cross into the core. Includes use tag overlap;
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
    // back to any pool). The native legacy chooser is deleted, so every
    // query is provider-scoped.
    public string AllowedTagSet;
    public string ExcludedTagSet;

    // 3. P2 caller filter (Game LocomotionSelectionPolicy output):
    // LoopFilter mirrors MotionLoopFilter (0 Any, 1 LoopOnly, 2 OneShotOnly);
    // CandidateBand mirrors BakedClipSpeedBand DB ids (0 Any, 1 Walk, 2 Run,
    // 3 Sprint); QueryYawRate (rad/s, >= 0) scales the straightness prior.
    // The core never infers these from movement.
    public int LoopFilter;
    public int CandidateBand;
    public float QueryYawRate;
}

// P1: the P0 additive-only DTOs (MotionMatchingQueryFilter,
// MotionMatchingSearchTuning) were never read by Tick and are deleted. The
// live Game-side request is LocomotionSelectionRequest
// (Source/Game/Player/LocomotionSelectionPolicy.cs): scope crosses into the
// core through MotionMatchingFrameInput above (direct path), intent/loop/
// band mirrors stay Game-side until P2. No native struct field is added
// (MotionMatchingTraceEntry stays 61/64 for the Flax 1.12 bindings cap).
