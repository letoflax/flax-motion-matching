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

    // 3. Scope. HasScope=false = provider does not decide (legacy native
    // decision, bare callers). HasScope=true with an empty AllowedTagSet
    // = explicit no-match: no candidate may win, the current clip holds.
    public bool HasScope;
    public string AllowedTagSet;
    public string ExcludedTagSet;

    // 4. Locomotion-owned tuning the core still consumes (roles replace
    // the old literals; speeds replace the old shell scene fields).
    public string JumpTag;
    public string TurnInPlaceTag;
    public string TurnTravelTag;
    public string RestTag;
    public float RestSpeedMax;
    public float JumpWalkSpeedMax;
    public float JumpRunSpeedMax;
    public float YawRateThreshold;
    public int TagConfirmRepeats;
    public float BackwardThreshold;
    public float LateralThreshold;
    public float LateralZRatio;

    // 5. Overlay inputs, decided game-side (P4/P6): procedural pitch for the
    // overlay layer + head visibility. The core forwards them verbatim and
    // never sees camera state.
    public float AimPitchDeg;
    public bool HideHead;

    // 6. Driver ownership (P2/P3): this provider enables the core's
    // airborne-transition machine and standing-turn machine. False = pure
    // search from that machine (no direct-plays, holds or facing overrides).
    // Defaults are false in a bare struct: providers must opt in explicitly.
    public bool EnableJumpDriver;
    public bool EnableTurnDrive;
}
