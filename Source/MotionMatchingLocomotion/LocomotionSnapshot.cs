using FlaxEngine;

namespace MotionMatching;

/// <summary>
/// Neutral per-frame movement snapshot for <see cref="DefaultLocomotionProvider"/>.
/// Plain data, no logic, no player types: any host (player rig, cutscene,
/// test driver) fills this from its own state. Overlay authorship stays
/// generic — <see cref="LookPitchDeg"/> and <see cref="RequestHeadHide"/>
/// are decided host-side (e.g. by a first-person adapter), never derived
/// from camera objects in here.
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
    public float LookPitchDeg;
    public bool RequestHeadHide;
}
