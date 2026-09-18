#pragma once

#include "Engine/Core/Collections/Array.h"
#include "Engine/Core/Math/Matrix.h"
#include "Engine/Core/Math/Vector3.h"
#include "Engine/Core/Math/Quaternion.h"
#include "Engine/Core/Types/String.h"
#include "Engine/Scripting/ScriptingObject.h"
#include "../Schema/MotionMatchingTypes.h"

class AnimatedModel;
class SkinnedModel;
class Animation;
class MotionMatchingDatabase;
class PhysicsColliderActor;

/// <summary>
/// PLAN §5.1 Foot-lock lifecycle state per foot:
/// Free -> Planting -> Locked -> Releasing -> Free.
/// </summary>
API_ENUM()
enum class LockState
{
    Free = 0,
    Planting = 1,
    Locked = 2,
    Releasing = 3,
};

/// <summary>
/// Generic per-tick pose overlay seam (keeps non-MM processing OUT of the
/// motion-matching folder): after inertial decay + search-pose save, before
/// FK compose, playback hands the bone-LOCAL pose to the overlay function
/// (e.g. the first-person aim/head-hide in Source/Game/Player, wired via
/// its static AttachTo). The overlay owns all of its math; playback only
/// mirrors the reported telemetry. Plain function pointer (no inheritance)
/// so neither side constrains the other's scripting shape.
/// </summary>
typedef void (*MotionMatchingPoseOverlayFn)(
    Matrix* locals,
    int32 boneCount,
    const NodeIndices& nodes,
    float overlayPitchDeg,
    float spineShare,
    bool hideHead,
    float& outSpineDeg,
    float& outHeadDeg,
    bool& outHeadHidden,
    void* userData);

/// <summary>
/// Manual clip playback (plan Milestone 5). Advances the winning clip at the
/// baked sample rate, wraps loop clips, clamps one-shots, and blends between
/// clips with quintic inertialization (UE AnimNode_Inertialization port, see
/// CalcInertialFloat): offset = oldPose - newPose decayed per bone with zero
/// end value/velocity/acceleration, so phase-mismatched switches settle
/// without the foot-skate of linear crossfade. Root motion stays Disabled:
/// CharacterController owns movement, this only shows poses via
/// AnimatedModel.SetCurrentPose.
/// Requires the body to have NO AnimGraph assigned — the graph would
/// overwrite the manually written pose every frame.
///
/// Hierarchy correctness: SetCurrentPose(worldSpace: false) takes final
/// actor-space (model-space) matrices, but stripping and blending MUST happen
/// in bone-LOCAL space before forward kinematics. Editing only the root of a
/// composed pose leaves every descendant carrying the old root travel/yaw,
/// and lerping composed matrices independently stretches bones. This class
/// therefore samples locals, strips the root local to its bind reference,
/// blends locals (native TRS blend), then composes to model space.
///
/// Wave C foot IK (legs only, playback-output-only, never search): after
/// compose, planted feet get an analytic 2-bone solve (thigh->calf->foot)
/// in body space toward a world-pinned + raycast-grounded ankle target.
/// The solve writes back ONLY thigh/calf local rotations (positions/scales
/// untouched, bone lengths exact) and recomposes with the same native FK,
/// so the local-space discipline above is preserved and SetCurrentPose is
/// still called once with a consistent pose.
/// </summary>
API_CLASS(Namespace="MotionMatching")
class MOTIONMATCHING_API MotionMatchingPlayback : public ScriptingObject
{
    DECLARE_SCRIPTING_TYPE_WITH_CONSTRUCTOR_IMPL(MotionMatchingPlayback, ScriptingObject);

public:
    // PLAN §6.1 / §18.10 play-rate ownership: Playback is the ONLY place that
    // applies time. Rate defaults to 1.0, clamped to [0.85, 1.15].
    API_PROPERTY() float GetPlayRate() const;
    API_PROPERTY() void SetPlayRate(float value);
    API_PROPERTY() float GetPlayRateBefore() const;
    API_PROPERTY() float GetPlayRateAfter() const;
    API_FIELD() float LastEffectiveDt = 0.0f;
    API_FIELD() int32 ClipCycle = 0;
    // Inertialization blend time (seconds, replaces linear-crossfade
    // BlendDuration — single path, no parallel blend modes). The policy
    // overrides this per jump-landing severity, same pattern as before.
    API_FIELD() float InertializationDuration = 0.25f;
    // True while a decay offset is active (telemetry only).
    API_PROPERTY() bool GetInertialActive() const;

    // Procedural overlay INPUTS (overlay seam — the bone math lives in the
    // overlay impl, playback only feeds + mirrors telemetry). Pitch bends
    // spine + head in bone-LOCAL space after root-strip and blend, before
    // FK compose. Yaw is intentionally skipped: turn-drive owns facing.
    // Values arrive decided (the provider owns camera/locomotion rules);
    // playback clamps + forwards, never interprets.
    API_FIELD() float OverlayPitchDeg = 0.0f;
    API_FIELD() float OverlayClampDeg = 60.0f;
    API_FIELD() float OverlaySpineShare = 0.5f;
    // Last tick's applied angles (degrees, telemetry only).
    API_FIELD() float LastAimRequested = 0.0f;
    API_FIELD() float LastAimSpine = 0.0f;
    API_FIELD() float LastAimHead = 0.0f;
    // Head-hide input + mirror (overlay-owned math, see seam above).
    API_FIELD() bool HideHead = false;
    API_FIELD() bool LastHeadHidden = false;

    // Wave C foot IK (legs only, playback-output-only, never search).
    // OWNERSHIP (P5): the solver + lock state machine below are CORE-OWNED
    // generic animation: bone indices come from the schema profile (feet
    // found by parent-walk, no bone-name or tag literals anywhere), ground
    // contact from physics sweeps, all behavior from the tunables below.
    // Game-specific policy stays OUTSIDE: the engage gate (movement
    // speeds + EnableFootIK switch) is computed by the policy from plain
    // snapshot/tuning data, and the search-cost prior only reads the firm
    // lock pins. No locomotion vocabulary in this block.
    API_FIELD() bool FootIKEnabled = true;
    API_FIELD() float FootLockAuthority = 0.6f;
    API_FIELD() float FootMaxCorrection = 15.0f;
    API_FIELD() float AnkleToSoleOffset = 11.0f;
    API_FIELD() float FootBlendTime = 0.12f;
    API_FIELD() float FootReleaseDelay = 0.06f;
    API_FIELD() float FootPlantingDelay = 0.05f;
    API_FIELD() int32 FootPlantingMinFrames = 3;
    API_FIELD() int32 FootReleasingMinFrames = 2;
    API_FIELD() int32 FootLockedMinFrames = 5;
    API_FIELD() float FootLockEnterSpeed = 300.0f;
    API_FIELD() float FootLockExitSpeed = 250.0f;
    API_FIELD() float FootLockEnterHeight = 12.0f;
    API_FIELD() float FootLockExitHeight = 14.0f;
    API_PROPERTY() float GetFootLockMaxSpeed() const;
    API_PROPERTY() void SetFootLockMaxSpeed(float value);
    API_PROPERTY() float GetFootLockMaxHeight() const;
    API_PROPERTY() void SetFootLockMaxHeight(float value);
    // Foot-radius for the ground sweep (units, cm).
    API_FIELD() float FootSweepRadius = 5.0f;

    API_FIELD() LockState LockStateL = LockState::Free;
    API_FIELD() LockState LockStateR = LockState::Free;
    API_FIELD() bool FirmLockL = false;
    API_FIELD() bool FirmLockR = false;
    API_FIELD() float LastFootYL = 0.0f;
    API_FIELD() float LastFootYR = 0.0f;
    API_FIELD() bool LastFootLockL = false;
    API_FIELD() bool LastFootLockR = false;
    API_FIELD() float LastFootWeightL = 0.0f;
    API_FIELD() float LastFootWeightR = 0.0f;
    API_FIELD() float LastFootCorrL = 0.0f;
    API_FIELD() float LastFootCorrR = 0.0f;
    API_FIELD() float LastFootVelocityWorldL = 0.0f;
    API_FIELD() float LastFootVelocityWorldR = 0.0f;
    API_FIELD() float LastPoseVelocityRootRelativeL = 0.0f;
    API_FIELD() float LastPoseVelocityRootRelativeR = 0.0f;
    API_FIELD() float LastContactHeightL = 0.0f;
    API_FIELD() float LastContactHeightR = 0.0f;

    API_FIELD() int32 CurrentClip = -1;
    API_FIELD() float CurrentTime = 0.0f;
    API_FIELD() bool IsAtEnd = false;
    API_PROPERTY() float GetCurrentClipLength() const;

    API_FIELD() int64 SampleFrameId = -1;
    API_FIELD() int64 SearchFrameId = -1;
    API_FIELD() int64 SaveSearchPoseFrameId = -1;
    API_FIELD() int64 ApplyAimFrameId = -1;
    API_FIELD() int64 ApplyIKFrameId = -1;

public:
    API_FUNCTION() void Setup(
        MotionMatchingDatabase* database,
        SkinnedModel* fallbackModel,
        const SkeletonProfile& skeleton,
        const Float3& controllerUpAxis);
    API_FUNCTION() void Stop();
    API_FUNCTION() void Play(int32 clipIndex, float startTime, bool forceSeek);
    API_FUNCTION() void ResetFootLock();
    API_FUNCTION() void ResetFootLockSide(int32 side);
    API_FUNCTION() void ResetOnTeleport();
    API_FUNCTION() void NotifySearchExecuted(int64 tickFrameId);
    API_FUNCTION() Array<Matrix> GetSearchLocals() const;
    API_FUNCTION() void Tick(float deltaTime, AnimatedModel* body, bool grounded);

private:
    MotionMatchingDatabase* _database = nullptr;
    SkinnedModel* _fallbackModel = nullptr;
    SkinnedModel* _hierarchyModel = nullptr;
    int32 _rootNodeIndex = -1;
    Array<Animation*> _clips;
    Array<float> _lengths;
    Array<bool> _loops;

    Array<int32> _parents;
    Array<Matrix> _referenceLocals;
    Matrix _referenceRootLocal = Matrix::Identity;
    bool _hasReferenceRoot = false;

    float _playRate = 1.0f;
    float _playRateBefore = 1.0f;

    int32 _aimSpineIndex = -1;
    int32 _aimHeadIndex = -1;
    bool _aimLogged = false;

    Float3 _rootUpAxis = Float3(0.0f, 0.0f, -1.0f);
    Float3 _controllerUpAxis = Float3(0.0f, 0.0f, -1.0f);

    int32 _footLIndex = -1;
    int32 _footRIndex = -1;
    int32 _thighLIndex = -1;
    int32 _calfLIndex = -1;
    int32 _thighRIndex = -1;
    int32 _calfRIndex = -1;
    bool _legsResolved = false;
    bool _legsResolveFailed = false;
    bool _plantL = false;
    bool _plantR = false;
    float _weightL = 0.0f;
    float _weightR = 0.0f;
    Vector3 _pinWL = Vector3::Zero;
    Vector3 _pinWR = Vector3::Zero;
    bool _hasPinL = false;
    bool _hasPinR = false;
    Vector3 _prevFootWL = Vector3::Zero;
    Vector3 _prevFootWR = Vector3::Zero;
    bool _hasPrevFootW = false;
    bool _footRebasePending = false;
    Float3 _prevRelL = Float3::Zero;
    Float3 _prevRelR = Float3::Zero;
    bool _hasPrevRel = false;
    float _releaseL = 0.0f;
    float _releaseR = 0.0f;
    float _plantingGraceL = 0.0f;
    float _plantingGraceR = 0.0f;
    int32 _stateFramesL = 0;
    int32 _stateFramesR = 0;

    Array<Matrix> _displayedLocals;
    Array<Matrix> _prevDisplayedLocals;
    Array<Matrix> _searchLocals;
    Array<Matrix> _prevSearchLocals;

    int64 _tickFrameId = 0;
    bool _orderWarned = false;

    bool _inertialPending = false;
    bool _inertialActive = false;
    float _inertialT = 0.0f;
    float _inertialDur = 0.0f;
    Array<Float3> _inertPosDir;
    Array<float> _inertPosMag;
    Array<float> _inertPosVel;
    Array<Float3> _inertRotAxis;
    Array<float> _inertRotAngle;
    Array<float> _inertRotVel;
    Array<Float3> _inertSclDir;
    Array<float> _inertSclMag;
    Array<float> _inertSclVel;
    bool _warned = false;
    // Scratch storage for BuildInertialOffsets (the extra new-clip sample
    // must outlive the call scope while offsets are computed).
    Array<Matrix> _prevSampleScratch;

    MotionMatchingPoseOverlayFn _overlayFn = nullptr;
    void* _overlayUser = nullptr;

public:
    // C++-only wiring (called by the overlay's static AttachTo, never C#).
    void SetPoseOverlay(MotionMatchingPoseOverlayFn fn, void* userData);

private:

    bool _poleWarned = false;
    bool _scaleWarned = false;

    static float ClampF(float value, float minValue, float maxValue);
    static float MaxF(float a, float b);
    static float MinF(float a, float b);
    static float AbsF(float value);
    static bool IsFiniteF(float value);

    void RefreshRootUpAxis();
    static bool IsFiniteAxis(const Float3& value);
    static Float3 NormalizeAxis(const Float3& value);
    void ReleaseFootLocks(float dt);
    void ReleaseOneFoot(
        LockState& state, bool& firmLock, float& weight, Vector3& pinW, bool& hasPin,
        float& releaseGrace, float& plantingGrace, int32& stateFrames,
        bool& teleLock, float& teleWeight, float& teleCorr, float blendRate);
    void ArmInertial();
    void ClearInertial();
    void AssertTickOrder();
    void ApplyInertialization(Array<Matrix>& baseLocals, SkinnedModel* model, float dt);
    void BuildInertialOffsets(Array<Matrix>& newBase, SkinnedModel* model, float dt);
    static float CalcInertialFloat(float x0, float v0, float t, float t1);
    static Quaternion NormalizeQ(const Quaternion& value);
    static Quaternion ConjugateQ(const Quaternion& value);
    static void QuatToAxisAngle(const Quaternion& value, Float3& axis, float& angle);
    static Quaternion AxisAngleToQuat(const Float3& axis, float angle);
    static float TwistAbout(const Quaternion& value, const Float3& axis);
    static bool TryDecompose(const Matrix& value, Float3& scl, Quaternion& rot, Float3& pos);
    void EnsureHierarchy(SkinnedModel* model);
    bool ValidLeg(int32 thigh, int32 calf, int32 foot) const;
    static bool LegLengthsSane(float upper, float lower);
    static float BindBoneLength(const Array<Matrix>& bindModel, int32 a, int32 b);
    void ResolveLegs();
    Array<Matrix> ApplyFootIK(AnimatedModel* body, SkinnedModel* model, Array<Matrix>& locals, const Array<Matrix>& pose, float dt);
    void UpdateFootLock(int32 side, Actor* body, const Array<Matrix>& pose, const Float3& rootT,
        const Matrix& bodyWorld, const Matrix& bodyWorldInv, float dt, int32 footIndex,
        LockState& state, bool& firmLock, float& weight, Vector3& pinW, bool& hasPin,
        Vector3& prevFootW, float& releaseGrace, float& plantingGrace, int32& stateFrames,
        bool& teleLock, float& teleWeight, float& teleCorr, Float3& target);
    bool RaycastGround(const Vector3& footW, Actor* body, Vector3& groundW, Vector3& groundNW);
    static bool IsSelfHit(PhysicsColliderActor* collider, Actor* body);
    bool SolveLeg(Array<Matrix>& locals, const Array<Matrix>& pose, int32 thigh, int32 calf, int32 ankle, const Float3& target);
    static bool IsFiniteV(const Float3& value);
    static bool NearUnity(const Float3& value);
    static float DotF(const Float3& a, const Float3& b);
    static Float3 CrossF(const Float3& a, const Float3& b);
    static Float3 NormV(const Float3& value);
    static Quaternion QuatNorm(const Quaternion& value);
    static Quaternion QuatInverse(const Quaternion& value);
    static Quaternion RotationFromTo(const Float3& from, const Float3& to);
    void WarnOnce(const String& message);
};
