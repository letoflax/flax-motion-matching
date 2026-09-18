using FlaxEngine;

namespace MotionMatching;

/// <summary>
/// Neutral per-frame movement snapshot for <see cref="DefaultLocomotionProvider"/>.
/// Plain data, no logic, no player types: any host (player rig, cutscene,
/// test driver) fills this from its own state. Layer authorship stays
/// generic — <see cref="LayerAngleDeg"/> and <see cref="RequestBoneGate"/>
/// are decided host-side, never derived from host objects in here.
/// </summary>
[System.Serializable]
public struct LocomotionSnapshot
{
    public Float3 CurrentVelocity;
    public Float3 DesiredVelocity;
    public Float3 Acceleration;
    public Float3 AngularVelocity;
    public float TargetYaw;
    public float CurrentYaw;
    public bool IsGrounded;
    public bool IsSprinting;
    public float PositionHalflife;
    public float FacingHalflife;
    public float LayerAngleDeg;
    public bool RequestBoneGate;
}
