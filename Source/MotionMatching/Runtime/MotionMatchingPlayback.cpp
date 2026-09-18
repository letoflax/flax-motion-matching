#include "MotionMatchingPlayback.h"

#include "MotionMatchingSampler.h"
#include "../Database/MotionMatchingDatabase.h"
#include "Engine/Content/Assets/Animation.h"
#include "Engine/Content/Assets/SkinnedModel.h"
#include "Engine/Core/Log.h"
#include "Engine/Core/Math/Math.h"
#include "Engine/Level/Actors/AnimatedModel.h"
#include "Engine/Level/Actor.h"
#include "Engine/Physics/Colliders/Collider.h"
#include "Engine/Physics/Physics.h"

#include <cmath>

namespace
{
    constexpr float MinPlayRate = 0.85f;
    constexpr float MaxPlayRate = 1.15f;
    constexpr float PlayRateEpsilon = 1e-4f;
    constexpr float ContactMaxSpeed = 300.0f;
    constexpr float ContactMaxHeight = 12.0f;
    constexpr float ReachMaxScale = 0.999f;
    constexpr float ReachMinEpsilon = 0.5f;
    constexpr float PoleMinBend = 0.05f;
    constexpr float SolveMinDist = 0.0001f;
    constexpr float NormZeroEpsilon = 1e-7f;
    constexpr float ScaleUnityTolerance = 0.05f;
    constexpr float FootLockMinWeight = 0.02f;
    constexpr float FootLockMinCorrection = 0.02f;
    constexpr float PiF = 3.14159265358979323846f;
}

float MotionMatchingPlayback::ClampF(float value, float minValue, float maxValue)
{
    if (value < minValue)
        return minValue;
    if (value > maxValue)
        return maxValue;
    return value;
}

float MotionMatchingPlayback::MaxF(float a, float b)
{
    return a > b ? a : b;
}

float MotionMatchingPlayback::MinF(float a, float b)
{
    return a < b ? a : b;
}

float MotionMatchingPlayback::AbsF(float value)
{
    return value < 0.0f ? -value : value;
}

bool MotionMatchingPlayback::IsFiniteF(float value)
{
    return std::isfinite(value) != 0;
}

float MotionMatchingPlayback::GetPlayRate() const
{
    return _playRate;
}

void MotionMatchingPlayback::SetPlayRate(float value)
{
    const float clamped = ClampF(value, MinPlayRate, MaxPlayRate);
    if (AbsF(clamped - _playRate) > PlayRateEpsilon)
    {
        _playRateBefore = _playRate;
        _playRate = clamped;
        // Rate changes do NOT wipe inertial offsets: offsets are
        // pose-space (oldPose - newPose decaying over wall time), so a
        // retime neither invalidates nor mixes time bases. Wiping here
        // used to kill the blend on every normal speed change. Real
        // discontinuities still reset via Play/Stop/teleport, and
        // BuildInertialOffsets keeps its cross-rate guard.
    }
}

float MotionMatchingPlayback::GetPlayRateBefore() const
{
    return _playRateBefore;
}

float MotionMatchingPlayback::GetPlayRateAfter() const
{
    return _playRate;
}

bool MotionMatchingPlayback::GetInertialActive() const
{
    return _inertialActive;
}

float MotionMatchingPlayback::GetCurrentClipLength() const
{
    if (CurrentClip >= 0 && CurrentClip < _lengths.Count())
        return _lengths[CurrentClip];
    return 0.0f;
}

float MotionMatchingPlayback::GetFootLockMaxSpeed() const
{
    return FootLockEnterSpeed;
}

void MotionMatchingPlayback::SetFootLockMaxSpeed(float value)
{
    FootLockEnterSpeed = value;
    FootLockExitSpeed = value;
}

float MotionMatchingPlayback::GetFootLockMaxHeight() const
{
    return FootLockEnterHeight;
}

void MotionMatchingPlayback::SetFootLockMaxHeight(float value)
{
    FootLockEnterHeight = value;
    FootLockExitHeight = value;
}

void MotionMatchingPlayback::Setup(
    MotionMatchingDatabase* database,
    SkinnedModel* fallbackModel,
    const SkeletonProfile& skeleton,
    const Float3& controllerUpAxis)
{
    Stop();
    _database = database;
    _fallbackModel = fallbackModel;
    _controllerUpAxis = controllerUpAxis;
    _rootNodeIndex = -1;
    _layerPrimaryIndex = -1;
    _layerSecondaryIndex = -1;
    _layerLogged = false;
    _clips.Clear();
    _lengths.Clear();
    _loops.Clear();

    if (_database == nullptr || !_database->IsBaked())
        return;

    const int32 count = _database->GetClipCount();
    _lengths = _database->GetClipLengths();
    for (int32 i = 0; i < count; i++)
    {
        _clips.Add(_database->GetClipAnimation(i));
        _loops.Add(_database->GetClipLoop(i));
    }

    // Resolve the root node once: its baked translation carries the
    // clip's travel through space and must be stripped every frame
    // (see Tick), otherwise the body teleports meters away on walk/run.
    SkinnedModel* resolveModel = _fallbackModel;
    NodeIndices nodes;
    String resolveError;
    if (resolveModel != nullptr && MotionMatchingSkeleton::TryResolve(skeleton, resolveModel, nodes, resolveError))
    {
        _rootNodeIndex = nodes.Root;
        // Optional upper-body nodes: missing (-1) bones are skipped per
        // bone (share dropped, not redistributed), never a setup failure.
        _layerPrimaryIndex = nodes.Spine;
        _layerSecondaryIndex = nodes.Head;
        // Wave C legs: feet come from the same resolve; thigh/calf are
        // found by walking UP from each foot via the parent table
        // (no bone-name guesses, no C++ change needed).
        _footLIndex = nodes.LeftFoot;
        _footRIndex = nodes.RightFoot;
        _legsResolved = false;
        _legsResolveFailed = false;
        LOG(Info, "Motion matching overlay: spine node={} head node={} (clamp=+-{:.0f}deg split={:.2f}/{:.2f}).",
            _layerPrimaryIndex, _layerSecondaryIndex, (double)LayerClampDeg, (double)LayerSplitShare, (double)(1.0f - LayerSplitShare));
    }
    else if (resolveModel != nullptr)
    {
        // Fail-fast with the resolve reason: an empty profile means the host
        // never configured the skeleton for its rig (the struct ships empty
        // with no mannequin fallback); indices stay -1 and travel is NOT
        // stripped until the host configures it.
        LOG(Warning, "Motion matching playback skeleton not resolved: {}. "
            "Configure the SkeletonProfile for your rig "
            "(scene → MotionMatchingController → Schema → Skeleton).", resolveError);
    }
    RefreshRootUpAxis();
    EnsureHierarchy(resolveModel);
}

void MotionMatchingPlayback::RefreshRootUpAxis()
{
    Float3 candidate(0.0f, 0.0f, -1.0f);
    String source(TEXT("default(0,0,-1)"));
    if (_database != nullptr && _database->IsBaked())
    {
        const Float3 bakedUp = _database->GetSchema().RootUpAxis;
        if (IsFiniteAxis(bakedUp))
        {
            candidate = bakedUp;
            source = TEXT("database-baked-schema");
        }
    }
    if (source == String(TEXT("default(0,0,-1)")) && IsFiniteAxis(_controllerUpAxis))
    {
        candidate = _controllerUpAxis;
        source = TEXT("controller-schema");
    }
    const Float3 n = NormalizeAxis(candidate);
    if (n.Length() < 1e-6f)
    {
        LOG(Warning, "Motion matching contact: root-up axis degenerate, keeping ({:.2f},{:.2f},{:.2f}).",
            (double)_rootUpAxis.X, (double)_rootUpAxis.Y, (double)_rootUpAxis.Z);
        return;
    }
    _rootUpAxis = n;
    if (source == String(TEXT("database-baked-schema")))
        LOG(Info, "Motion matching contact axis: up=({:.3f},{:.3f},{:.3f}) from database-baked-schema.",
            (double)_rootUpAxis.X, (double)_rootUpAxis.Y, (double)_rootUpAxis.Z);
    else
        LOG(Info, "Motion matching contact axis: up=({:.3f},{:.3f},{:.3f}) from controller-schema.",
            (double)_rootUpAxis.X, (double)_rootUpAxis.Y, (double)_rootUpAxis.Z);
}

bool MotionMatchingPlayback::IsFiniteAxis(const Float3& value)
{
    return std::isfinite(value.X) && std::isfinite(value.Y) && std::isfinite(value.Z) &&
        (value.X * value.X + value.Y * value.Y + value.Z * value.Z) > 1e-12f;
}

Float3 MotionMatchingPlayback::NormalizeAxis(const Float3& value)
{
    const float len = (float)std::sqrt(value.X * value.X + value.Y * value.Y + value.Z * value.Z);
    if (!std::isfinite(len) || len < 1e-9f)
        return Float3::Zero;
    return Float3(value.X / len, value.Y / len, value.Z / len);
}

void MotionMatchingPlayback::Stop()
{
    CurrentClip = -1;
    CurrentTime = 0.0f;
    ClipCycle = 0;
    LastEffectiveDt = 0.0f;
    _playRate = 1.0f;
    _playRateBefore = 1.0f;
    IsAtEnd = false;
    _displayedLocals.Clear();
    _prevDisplayedLocals.Clear();
    _searchLocals.Clear();
    _prevSearchLocals.Clear();
    _tickFrameId = 0;
    SampleFrameId = -1;
    SearchFrameId = -1;
    SaveSearchPoseFrameId = -1;
    ApplyLayerFrameId = -1;
    ApplyIKFrameId = -1;
    _orderWarned = false;
    _poleWarned = false;
    _scaleWarned = false;
    ClearInertial();
    LayerAngleDeg = 0.0f;
    LastLayerRequested = 0.0f;
    LastLayerPrimary = 0.0f;
    LastLayerSecondary = 0.0f;
    GateBone = false;
    LastLayerGateApplied = false;
    // Wave C / PLAN §5.1: release pins and reset foot-lock state machines to Free.
    ResetFootLock();
}

void MotionMatchingPlayback::ReleaseFootLocks(float dt)
{
    const float rate = dt / MaxF(FootBlendTime, 0.01f);
    ReleaseOneFoot(LockStateL, FirmLockL, _weightL, _pinWL, _hasPinL,
        _releaseL, _plantingGraceL, _stateFramesL,
        LastFootLockL, LastFootWeightL, LastFootCorrL, rate);
    ReleaseOneFoot(LockStateR, FirmLockR, _weightR, _pinWR, _hasPinR,
        _releaseR, _plantingGraceR, _stateFramesR,
        LastFootLockR, LastFootWeightR, LastFootCorrR, rate);
}

void MotionMatchingPlayback::ReleaseOneFoot(
    LockState& state, bool& firmLock, float& weight, Vector3& pinW, bool& hasPin,
    float& releaseGrace, float& plantingGrace, int32& stateFrames,
    bool& teleLock, float& teleWeight, float& teleCorr, float blendRate)
{
    if (state == LockState::Planting || state == LockState::Locked)
        state = LockState::Releasing;
    firmLock = false;
    if (state == LockState::Releasing)
    {
        stateFrames++;
        weight = MaxF(weight - blendRate, 0.0f);
        if (weight <= 0.0f && stateFrames >= FootReleasingMinFrames)
        {
            state = LockState::Free;
            stateFrames = 0;
            weight = 0.0f;
            hasPin = false;
        }
    }
    else if (state == LockState::Free)
    {
        weight = 0.0f;
        hasPin = false;
    }
    teleLock = false;
    teleWeight = weight;
    if (weight <= FootLockMinWeight)
        teleCorr = 0.0f;
}

void MotionMatchingPlayback::ResetFootLock()
{
    ResetFootLockSide(0);
    ResetFootLockSide(1);
    _hasPrevFootW = false;
    _hasPrevRel = false;
}

void MotionMatchingPlayback::ResetFootLockSide(int32 side)
{
    if (side == 0)
    {
        LockStateL = LockState::Free;
        FirmLockL = false;
        _plantL = false;
        _weightL = 0.0f;
        _pinWL = Vector3::Zero;
        _hasPinL = false;
        _releaseL = 0.0f;
        _plantingGraceL = 0.0f;
        _stateFramesL = 0;
        LastFootLockL = false;
        LastFootWeightL = 0.0f;
        LastFootCorrL = 0.0f;
        LastFootYL = 0.0f;
        LastFootVelocityWorldL = 0.0f;
        LastPoseVelocityRootRelativeL = 0.0f;
        LastContactHeightL = 0.0f;
    }
    else
    {
        LockStateR = LockState::Free;
        FirmLockR = false;
        _plantR = false;
        _weightR = 0.0f;
        _pinWR = Vector3::Zero;
        _hasPinR = false;
        _releaseR = 0.0f;
        _plantingGraceR = 0.0f;
        _stateFramesR = 0;
        LastFootLockR = false;
        LastFootWeightR = 0.0f;
        LastFootCorrR = 0.0f;
        LastFootYR = 0.0f;
        LastFootVelocityWorldR = 0.0f;
        LastPoseVelocityRootRelativeR = 0.0f;
        LastContactHeightR = 0.0f;
    }
}

void MotionMatchingPlayback::ResetOnTeleport()
{
    ResetFootLock();
}

void MotionMatchingPlayback::Play(int32 clipIndex, float startTime, bool forceSeek)
{
    if (_database == nullptr || clipIndex < 0 || clipIndex >= _clips.Count())
        return;
    if (clipIndex == CurrentClip)
    {
        // Same clip re-selected: only seek on real discontinuities.
        // Nearby frames (< 0.2s) are playback advance, not a new winner —
        // seeking there every query causes visible stutter. Far jumps
        // (loop wrap, end recovery, forced re-query) do seek.
        const float target = MaxF(startTime, 0.0f);
        const float jump = AbsF(target - CurrentTime);
        if (!forceSeek && !IsAtEnd && jump <= 0.2f)
            return;
        if (forceSeek)
            ResetFootLock();
        _footRebasePending = true;
        ArmInertial();
        CurrentTime = target;
        IsAtEnd = false;
        return;
    }

    ArmInertial();
    _footRebasePending = true;
    CurrentClip = clipIndex;
    CurrentTime = MaxF(startTime, 0.0f);
    ClipCycle = 0;
    IsAtEnd = false;
}

void MotionMatchingPlayback::ArmInertial()
{
    ClearInertial();
    // PLAN §18.10: Rate change invalidates active inertial offsets; never
    // mix a retimed clip with an offset computed at another rate.
    if (AbsF(_playRate - _playRateBefore) > PlayRateEpsilon)
        return;
    if (_searchLocals.Count() > 0)
        _inertialPending = true;
}

void MotionMatchingPlayback::ClearInertial()
{
    _inertialPending = false;
    _inertialActive = false;
    _inertialT = 0.0f;
    _inertialDur = 0.0f;
    _inertPosDir.Clear();
    _inertPosMag.Clear();
    _inertPosVel.Clear();
    _inertRotAxis.Clear();
    _inertRotAngle.Clear();
    _inertRotVel.Clear();
    _inertSclDir.Clear();
    _inertSclMag.Clear();
    _inertSclVel.Clear();
}

void MotionMatchingPlayback::NotifySearchExecuted(int64 tickFrameId)
{
    SearchFrameId = tickFrameId;
}

Array<Matrix> MotionMatchingPlayback::GetSearchLocals() const
{
    return _searchLocals;
}

void MotionMatchingPlayback::SetPoseOverlay(MotionMatchingPoseOverlayFn fn, void* userData)
{
    _overlayFn = fn;
    _overlayUser = userData;
}

void MotionMatchingPlayback::AssertTickOrder()
{
    // Intra-tick invariant: sample -> save -> layer -> IK, all stamped with
    // the same _tickFrameId. Search ordering is checked only when the
    // policy wired NotifySearchExecuted for this tick.
    bool ok = SampleFrameId == _tickFrameId &&
        SaveSearchPoseFrameId == _tickFrameId &&
        ApplyLayerFrameId == _tickFrameId &&
        ApplyIKFrameId == _tickFrameId;
    if (SearchFrameId == _tickFrameId || SearchFrameId == -1)
    {
        // Search (policy side) must not run after the save within a tick.
        // When unwired (-1) there is nothing to check on this half.
    }
    else if (SearchFrameId != _tickFrameId)
    {
        ok = false; // stale search stamp leaked into a new tick
    }
    if (!ok && !_orderWarned)
    {
        _orderWarned = true;
        LOG(Warning, "Motion matching tick order violated: tick={} sample={} search={} save={} layer={} ik={}. Expected sample<=search<=save<layer<IK.",
            _tickFrameId, SampleFrameId, SearchFrameId, SaveSearchPoseFrameId, ApplyLayerFrameId, ApplyIKFrameId);
    }
}

void MotionMatchingPlayback::Tick(float deltaTime, AnimatedModel* body, bool grounded)
{
    if (_database == nullptr || body == nullptr || CurrentClip < 0 || CurrentClip >= _clips.Count())
        return;

    SkinnedModel* model = body->SkinnedModel.Get();
    if (model == nullptr)
        model = _fallbackModel;
    if (model == nullptr)
    {
        WarnOnce(String(TEXT("Motion matching playback stopped: body has no SkinnedModel.")));
        return;
    }

    const float length = _lengths.Count() > CurrentClip ? _lengths[CurrentClip] : 0.0f;
    if (length <= 0.0f)
    {
        char message[128];
        snprintf(message, sizeof(message), "Motion matching playback stopped: clip %d has no length.", CurrentClip);
        WarnOnce(String(message));
        return;
    }

    LastEffectiveDt = MaxF(deltaTime, 0.0f) * _playRate;
    CurrentTime += LastEffectiveDt;
    if (_loops.Count() > CurrentClip && _loops[CurrentClip])
    {
        if (CurrentTime >= length && length > 0.0f)
        {
            while (CurrentTime >= length)
            {
                CurrentTime -= length;
                ClipCycle++;
            }
        }
        IsAtEnd = false;
    }
    else if (CurrentTime >= length)
    {
        CurrentTime = length;
        IsAtEnd = true;
    }

    EnsureHierarchy(model);
    _tickFrameId++;
    Array<Matrix> locals;
    String error;
    if (!MotionMatchingSampler::SampleClipLocalPose(_clips[CurrentClip], model, CurrentTime, locals, error))
    {
        WarnOnce(String(TEXT("Motion matching playback stopped: ")) + error);
        return;
    }
    SampleFrameId = _tickFrameId;

    // Strip in LOCAL space before FK: pinning the composed root alone
    // would leave every child carrying the old travel/yaw.
    Array<Matrix>* stripped = &locals;
    Array<Matrix> strippedLocals;
    if (_hasReferenceRoot && _rootNodeIndex >= 0 && _rootNodeIndex < locals.Count())
    {
        String stripError;
        if (!MotionMatchingSampler::StripRootLocal(locals, _rootNodeIndex, _referenceRootLocal, strippedLocals, stripError))
        {
            WarnOnce(String(TEXT("Motion matching playback stopped: ")) + stripError);
            return;
        }
        stripped = &strippedLocals;
    }

    // Inertialization (replaces linear crossfade): decay the switch-time
    // offset oldPose - newPose per bone in LOCAL space, then the overlay
    // runs on the decayed pose exactly like before.
    // Inertial source is _searchLocals (pre-overlay/pre-IK), NEVER
    // _displayedLocals — see BuildInertialOffsets.
    Array<Matrix>& blendedLocals = *stripped;
    ApplyInertialization(blendedLocals, model, MaxF(deltaTime, 0.000001f));

    // PLAN §5.3 save: snapshot the search pose AFTER inertial, BEFORE the overlay.
    // Deep copy (not an alias): the overlay/foot-IK mutate blendedLocals
    // in place below, and the snapshot must survive them untouched.
    if (_searchLocals.Count() == blendedLocals.Count() && blendedLocals.Count() > 0)
    {
        if (_prevSearchLocals.Count() != _searchLocals.Count())
            _prevSearchLocals.Resize(_searchLocals.Count());
        for (int32 i = 0; i < _searchLocals.Count(); i++)
            _prevSearchLocals[i] = _searchLocals[i];
    }
    else
    {
        _prevSearchLocals.Clear();
    }
    _searchLocals.Resize(blendedLocals.Count());
    for (int32 i = 0; i < blendedLocals.Count(); i++)
        _searchLocals[i] = blendedLocals[i];
    SaveSearchPoseFrameId = _tickFrameId;
    // Overlay (host-owned procedural layer): LOCAL-space layer on top of
    // the stripped+blended pose, before FK. Mutating in place is safe:
    // blendedLocals is a fresh array every tick (sampled or stripped this
    // frame), never the blend snapshot.
    {
        const float clamp = MaxF(LayerClampDeg, 0.0f);
        const float requested = ClampF(LayerAngleDeg, -clamp, clamp);
        float primary = 0.0f;
        float secondary = 0.0f;
        bool gateApplied = false;
        if (_overlayFn != nullptr)
        {
            NodeIndices overlayNodes;
            overlayNodes.Root = _rootNodeIndex;
            overlayNodes.Spine = _layerPrimaryIndex;
            overlayNodes.Head = _layerSecondaryIndex;
            overlayNodes.LeftFoot = _footLIndex;
            overlayNodes.RightFoot = _footRIndex;
            _overlayFn(blendedLocals.Get(), blendedLocals.Count(), overlayNodes,
                requested, ClampF(LayerSplitShare, 0.0f, 1.0f), GateBone,
                primary, secondary, gateApplied, _overlayUser);
        }
        LastLayerRequested = requested;
        LastLayerPrimary = primary;
        LastLayerSecondary = secondary;
        LastLayerGateApplied = gateApplied;
    }
    ApplyLayerFrameId = _tickFrameId;

    Array<Matrix> pose;
    String composeError;
    if (!MotionMatchingSampler::ComposeModelPose(model, blendedLocals, pose, composeError))
    {
        WarnOnce(String(TEXT("Motion matching playback stopped: ")) + composeError);
        return;
    }

    // Wave C foot IK: body-space pass after compose. Edits thigh/calf
    // local rotations inside blendedLocals and recomposes natively when
    // a planted foot needs it; otherwise pose passes through untouched.
    Array<Matrix> finalPose = pose;
    const float frameDt = MaxF(deltaTime, 0.000001f);
    if (FootIKEnabled && grounded)
        finalPose = ApplyFootIK(body, model, blendedLocals, pose, frameDt);
    else
        ReleaseFootLocks(frameDt);
    // Rebase flags are per-tick: consume here so a disabled/airborne tick
    // cannot carry a stale rebase into a later enabled tick.
    _footRebasePending = false;
    ApplyIKFrameId = _tickFrameId;
    AssertTickOrder();

    body->SetCurrentPose(finalPose, false);
    // Foot world-height telemetry (what the eye sees): model-space foot
    // Y from the FINAL shown pose (post-IK). World Y = actor Y + this
    // (actor rotation is yaw-only, preserves Y). NaN-guarded, never fatal.
    if (_footLIndex >= 0 && _footLIndex < finalPose.Count())
        LastFootYL = finalPose[_footLIndex].M42;
    if (_footRIndex >= 0 && _footRIndex < finalPose.Count())
        LastFootYR = finalPose[_footRIndex].M42;
    if (!IsFiniteF(LastFootYL))
        LastFootYL = 0.0f;
    if (!IsFiniteF(LastFootYR))
        LastFootYR = 0.0f;
    // Displayed pose (display/telemetry ONLY): the post-overlay/post-IK locals
    // actually shown. Never an inertial source — inertial offsets come
    // from _searchLocals (pre-overlay/pre-IK). Copy the reference history for
    // audit parity; store a COPY of the shown locals so next frame's
    // in-place decay cannot mutate history through aliasing.
    if (_displayedLocals.Count() > 0)
    {
        if (_prevDisplayedLocals.Count() != _displayedLocals.Count())
            _prevDisplayedLocals.Resize(_displayedLocals.Count());
        for (int32 i = 0; i < _displayedLocals.Count(); i++)
            _prevDisplayedLocals[i] = _displayedLocals[i];
    }
    _displayedLocals.Resize(blendedLocals.Count());
    for (int32 i = 0; i < blendedLocals.Count(); i++)
        _displayedLocals[i] = blendedLocals[i];
    _playRateBefore = _playRate;
}

void MotionMatchingPlayback::ApplyInertialization(Array<Matrix>& baseLocals, SkinnedModel* model, float dt)
{
    if (_inertialPending)
    {
        _inertialPending = false;
        BuildInertialOffsets(baseLocals, model, dt);
    }
    if (!_inertialActive)
        return;
    _inertialT += dt;
    const float t = _inertialT;
    const float dur = MaxF(_inertialDur, 0.0001f);
    const int32 n = baseLocals.Count();
    for (int32 i = 0; i < n; i++)
    {
        Float3 scl;
        Quaternion rot;
        Float3 pos;
        baseLocals[i].Decompose(scl, rot, pos);
        // Translation offset.
        const float pm = _inertPosMag[i];
        if (pm > 0.0f)
        {
            const float k = CalcInertialFloat(pm, _inertPosVel[i], t, dur);
            pos += _inertPosDir[i] * k;
        }
        // Rotation offset (premultiply in the parent frame, same as the overlay).
        const float ra = _inertRotAngle[i];
        if (ra > 0.0f)
        {
            const float k = CalcInertialFloat(ra, _inertRotVel[i], t, dur);
            rot = AxisAngleToQuat(_inertRotAxis[i], k) * rot;
        }
        // Scale offset.
        const float sm = _inertSclMag[i];
        if (sm > 0.0f)
        {
            const float k = CalcInertialFloat(sm, _inertSclVel[i], t, dur);
            scl += _inertSclDir[i] * k;
        }
        Matrix::Transformation(scl, rot, pos, baseLocals[i]);
    }
    if (t >= dur - 1e-7f)
        ClearInertial();
}

void MotionMatchingPlayback::BuildInertialOffsets(Array<Matrix>& newBase, SkinnedModel* model, float dt)
{
    ClearInertial();
    // PLAN §18.10: Never mix a retimed clip with an offset computed at another rate.
    // If rate changed across the transition, continuity/inertial offsets must be reset.
    if (AbsF(_playRate - _playRateBefore) > PlayRateEpsilon)
        return;
    if (_searchLocals.Count() != newBase.Count() || newBase.Count() == 0)
        return;
    const int32 n = newBase.Count();
    // Previous search frame for old-bone velocity (first switch after
    // start has none -> pure quintic diff blend, UE's IgnoreVelocity mode).
    const bool hasOldPrev = _prevSearchLocals.Count() == n;
    // One extra sample of the new clip a frame earlier for new-bone
    // velocity. Failure here only drops the velocity term, never the Play.
    // Uses the same logical time basis: prevTime = CurrentTime - dt * _playRate.
    Array<Matrix> newPrevStorage;
    const Array<Matrix>* newPrev = nullptr;
    if (CurrentClip >= 0 && CurrentClip < _clips.Count() && model != nullptr)
    {
        const float logicalDt = dt * _playRate;
        float prevTime = CurrentTime - logicalDt;
        if (_loops.Count() > CurrentClip && _loops[CurrentClip] && _lengths.Count() > CurrentClip && _lengths[CurrentClip] > 0.0f)
        {
            const float clipLen = _lengths[CurrentClip];
            if (prevTime < 0.0f)
                prevTime = fmodf(fmodf(prevTime, clipLen) + clipLen, clipLen);
        }
        else
        {
            prevTime = MaxF(prevTime, 0.0f);
        }
        String sampleError;
        if (MotionMatchingSampler::SampleClipLocalPose(_clips[CurrentClip], model, prevTime, newPrevStorage, sampleError))
        {
            if (_hasReferenceRoot && _rootNodeIndex >= 0 && _rootNodeIndex < newPrevStorage.Count())
            {
                Array<Matrix> prevStripped;
                String stripError;
                if (MotionMatchingSampler::StripRootLocal(newPrevStorage, _rootNodeIndex, _referenceRootLocal, prevStripped, stripError))
                {
                    // Keep storage alive via member scratch (local dies at scope end).
                    _prevSampleScratch = prevStripped;
                    newPrev = &_prevSampleScratch;
                }
            }
            else
            {
                _prevSampleScratch = newPrevStorage;
                newPrev = &_prevSampleScratch;
            }
        }
    }
    const float invDt = 1.0f / MaxF(dt, 0.000001f);
    // Zero-fill (C# `new T[n]` semantics): bones that hit the
    // un-decomposable `continue` below must read zero offsets, not heap
    // garbage — Flax Array::Resize does not value-initialize PODs.
    _inertPosDir.Resize(n);
    _inertPosMag.Resize(n);
    _inertPosVel.Resize(n);
    _inertRotAxis.Resize(n);
    _inertRotAngle.Resize(n);
    _inertRotVel.Resize(n);
    _inertSclDir.Resize(n);
    _inertSclMag.Resize(n);
    _inertSclVel.Resize(n);
    for (int32 i = 0; i < n; i++)
    {
        _inertPosDir[i] = Float3::Zero;
        _inertPosMag[i] = 0.0f;
        _inertPosVel[i] = 0.0f;
        _inertRotAxis[i] = Float3::Zero;
        _inertRotAngle[i] = 0.0f;
        _inertRotVel[i] = 0.0f;
        _inertSclDir[i] = Float3::Zero;
        _inertSclMag[i] = 0.0f;
        _inertSclVel[i] = 0.0f;
    }
    for (int32 i = 0; i < n; i++)
    {
        Float3 oScl, oPos, nScl, nPos;
        Quaternion oRot, nRot;
        if (!TryDecompose(_searchLocals[i], oScl, oRot, oPos) ||
            !TryDecompose(newBase[i], nScl, nRot, nPos))
            continue; // un-decomposable bone: zero offset (UE ensure-fallback).
        // Translation.
        const Float3 off = oPos - nPos;
        const float mag = off.Length();
        if (mag > 1e-6f && std::isfinite(mag))
        {
            const Float3 dir = off / mag;
            float vel = 0.0f;
            Float3 oPrevPos, nPrevPos, oPrevScl, nPrevScl;
            Quaternion oPrevRot, nPrevRot;
            if (hasOldPrev && TryDecompose(_prevSearchLocals[i], oPrevScl, oPrevRot, oPrevPos) &&
                newPrev != nullptr && TryDecompose((*newPrev)[i], nPrevScl, nPrevRot, nPrevPos))
            {
                const Float3 offVel = ((oPos - oPrevPos) - (nPos - nPrevPos)) * invDt;
                if (std::isfinite(offVel.X + offVel.Y + offVel.Z))
                    vel = Float3::Dot(offVel, dir);
            }
            _inertPosDir[i] = dir;
            _inertPosMag[i] = mag;
            _inertPosVel[i] = vel;
        }
        // Rotation: qOff = qOld * inv(qNew), twist rate about its axis.
        const Quaternion qOff = oRot * ConjugateQ(NormalizeQ(nRot));
        Float3 axis;
        float angle = 0.0f;
        QuatToAxisAngle(qOff, axis, angle);
        if (angle > 1e-6f)
        {
            float angVel = 0.0f;
            Float3 oPrevPos2, nPrevPos2, oPrevScl2, nPrevScl2;
            Quaternion oPrevRot2, nPrevRot2;
            if (hasOldPrev && TryDecompose(_prevSearchLocals[i], oPrevScl2, oPrevRot2, oPrevPos2) &&
                newPrev != nullptr && TryDecompose((*newPrev)[i], nPrevScl2, nPrevRot2, nPrevPos2))
            {
                const Quaternion oldRate = oRot * ConjugateQ(NormalizeQ(oPrevRot2));
                const Quaternion newRate = nRot * ConjugateQ(NormalizeQ(nPrevRot2));
                angVel = (TwistAbout(oldRate, axis) - TwistAbout(newRate, axis)) * invDt;
                if (!std::isfinite(angVel))
                    angVel = 0.0f;
            }
            _inertRotAxis[i] = axis;
            _inertRotAngle[i] = angle;
            _inertRotVel[i] = angVel;
        }
        else
        {
            _inertRotAxis[i] = Float3(1.0f, 0.0f, 0.0f);
        }
        // Scale.
        const Float3 sOff = oScl - nScl;
        const float sMag = sOff.Length();
        if (sMag > 1e-6f && std::isfinite(sMag))
        {
            const Float3 sDir = sOff / sMag;
            float sVel = 0.0f;
            Float3 oPrevPos3, nPrevPos3, oPrevScl3, nPrevScl3;
            Quaternion oPrevRot3, nPrevRot3;
            if (hasOldPrev && TryDecompose(_prevSearchLocals[i], oPrevScl3, oPrevRot3, oPrevPos3) &&
                newPrev != nullptr && TryDecompose((*newPrev)[i], nPrevScl3, nPrevRot3, nPrevPos3))
            {
                const Float3 sOffVel = ((oScl - oPrevScl3) - (nScl - nPrevScl3)) * invDt;
                if (std::isfinite(sOffVel.X + sOffVel.Y + sOffVel.Z))
                    sVel = Float3::Dot(sOffVel, sDir);
            }
            _inertSclDir[i] = sDir;
            _inertSclMag[i] = sMag;
            _inertSclVel[i] = sVel;
        }
    }
    _inertialT = 0.0f;
    _inertialDur = MaxF(InertializationDuration, 0.01f);
    _inertialActive = true;
}

// Quintic inertial decay ported from UE AnimNode_Inertialization
// (CalcInertialFloat): initial (x0, v0), zero value/velocity/acceleration
// at t1, overshoot-clamped (v0 > 0 dropped, t1 shortened when needed).
// Invalid inputs decay to zero (UE ensure-fallback equivalent).
float MotionMatchingPlayback::CalcInertialFloat(float x0, float v0, float t, float t1)
{
    constexpr float Eps = 1e-7f;
    constexpr float Small = 1e-4f;
    if (!std::isfinite(x0) || !std::isfinite(v0) || !std::isfinite(t) || !std::isfinite(t1))
        return 0.0f;
    if (t < 0.0f)
        t = 0.0f;
    if (t >= t1 - Eps)
        return 0.0f;
    float sign = 1.0f;
    if (x0 < 0.0f)
    {
        x0 = -x0;
        v0 = -v0;
        sign = -1.0f;
    }
    if (v0 > 0.0f)
        v0 = 0.0f;
    if (!(x0 >= 0.0f && v0 <= 0.0f && t >= 0.0f && t1 >= 0.0f))
        return 0.0f;
    if (v0 < -Small)
        t1 = MinF(t1, -5.0f * x0 / v0);
    if (t >= t1 - Eps)
        return 0.0f;
    const float t1_2 = t1 * t1;
    const float t1_3 = t1 * t1_2;
    const float t1_4 = t1 * t1_3;
    const float t1_5 = t1 * t1_4;
    const float a0 = MaxF(0.0f, (-8.0f * t1 * v0 - 20.0f * x0) / t1_2);
    const float A = -0.5f * (a0 * t1_2 + 6.0f * t1 * v0 + 12.0f * x0) / t1_5;
    const float B = 0.5f * (3.0f * a0 * t1_2 + 16.0f * t1 * v0 + 30.0f * x0) / t1_4;
    const float C = -0.5f * (3.0f * a0 * t1_2 + 12.0f * t1 * v0 + 20.0f * x0) / t1_3;
    const float D = 0.5f * a0;
    const float x = (((((A * t) + B) * t + C) * t + D) * t + v0) * t + x0;
    return std::isfinite(x) ? x * sign : 0.0f;
}

// Hand-rolled quaternion helpers (no engine-API guessing): unit-quat
// conjugate = inverse; twist = signed angle about an axis.
Quaternion MotionMatchingPlayback::NormalizeQ(const Quaternion& value)
{
    const float lenSq = value.X * value.X + value.Y * value.Y + value.Z * value.Z + value.W * value.W;
    if (lenSq <= 1e-12f)
        return Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    const float inv = 1.0f / (float)std::sqrt(lenSq);
    return Quaternion(value.X * inv, value.Y * inv, value.Z * inv, value.W * inv);
}

Quaternion MotionMatchingPlayback::ConjugateQ(const Quaternion& value)
{
    return Quaternion(-value.X, -value.Y, -value.Z, value.W);
}

void MotionMatchingPlayback::QuatToAxisAngle(const Quaternion& value, Float3& axis, float& angle)
{
    const Quaternion q = NormalizeQ(value);
    const float w = ClampF(q.W, -1.0f, 1.0f);
    angle = 2.0f * (float)std::acos((double)w);
    const float s = (float)std::sqrt(MaxF(0.0f, 1.0f - w * w));
    if (s <= 1e-6f || angle <= 1e-6f)
    {
        axis = Float3(1.0f, 0.0f, 0.0f);
        angle = 0.0f;
        return;
    }
    axis = Float3(q.X / s, q.Y / s, q.Z / s);
    if (angle > PiF)
    {
        angle = 2.0f * PiF - angle;
        axis = axis * -1.0f;
    }
}

Quaternion MotionMatchingPlayback::AxisAngleToQuat(const Float3& axis, float angle)
{
    const float lenSq = axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z;
    if (lenSq <= 1e-12f || AbsF(angle) <= 1e-9f)
        return Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    const float inv = 1.0f / (float)std::sqrt(lenSq);
    const float s = (float)std::sin((double)angle * 0.5);
    return Quaternion(axis.X * inv * s, axis.Y * inv * s, axis.Z * inv * s, (float)std::cos((double)angle * 0.5));
}

// Signed twist of q about axis (radians), small-angle safe at 60fps.
float MotionMatchingPlayback::TwistAbout(const Quaternion& value, const Float3& axis)
{
    const Quaternion q = NormalizeQ(value);
    const float w = ClampF(q.W, -1.0f, 1.0f);
    float ang = 2.0f * (float)std::acos((double)w);
    const float s = (float)std::sqrt(MaxF(0.0f, 1.0f - w * w));
    if (s <= 1e-6f)
        return 0.0f;
    const Float3 v(q.X / s, q.Y / s, q.Z / s);
    const float sign = Float3::Dot(v, axis) >= 0.0f ? 1.0f : -1.0f;
    if (ang > PiF)
        ang = 2.0f * PiF - ang;
    return sign * ang;
}

bool MotionMatchingPlayback::TryDecompose(const Matrix& value, Float3& scl, Quaternion& rot, Float3& pos)
{
    value.Decompose(scl, rot, pos);
    return std::isfinite(scl.X + scl.Y + scl.Z + pos.X + pos.Y + pos.Z +
        rot.X + rot.Y + rot.Z + rot.W);
}

void MotionMatchingPlayback::EnsureHierarchy(SkinnedModel* model)
{
    if (model == nullptr)
        return;
    if (_parents.Count() > 0 && _referenceLocals.Count() == _parents.Count())
        return;
    Array<int32> parents;
    Array<Matrix> referenceLocals;
    String error;
    if (!MotionMatchingSampler::GetSkeletonHierarchy(model, parents, referenceLocals, error))
        return;
    _hierarchyModel = model;
    _parents = parents;
    _referenceLocals = referenceLocals;
    _hasReferenceRoot = false;
    if (_rootNodeIndex >= 0 && _rootNodeIndex < _referenceLocals.Count())
    {
        _referenceRootLocal = _referenceLocals[_rootNodeIndex];
        _hasReferenceRoot = true;
    }
}

bool MotionMatchingPlayback::ValidLeg(int32 thigh, int32 calf, int32 foot) const
{
    const int32 n = _parents.Count();
    return thigh >= 0 && calf >= 0 && foot >= 0 && thigh < n && calf < n && foot < n
        && thigh != calf && calf != foot && thigh != foot;
}

bool MotionMatchingPlayback::LegLengthsSane(float upper, float lower)
{
    // Adult mocap leg segments in units (cm).
    return upper >= 20.0f && upper <= 70.0f && lower >= 20.0f && lower <= 70.0f;
}

float MotionMatchingPlayback::BindBoneLength(const Array<Matrix>& bindModel, int32 a, int32 b)
{
    const Float3 ta(bindModel[a].M41, bindModel[a].M42, bindModel[a].M43);
    const Float3 tb(bindModel[b].M41, bindModel[b].M42, bindModel[b].M43);
    return (tb - ta).Length();
}

void MotionMatchingPlayback::ResolveLegs()
{
    if (_legsResolved || _legsResolveFailed)
        return;
    if (_parents.Count() == 0 || _referenceLocals.Count() != _parents.Count())
        return; // hierarchy not ready yet; retry next tick
    if (_footLIndex < 0 || _footRIndex < 0 ||
        _footLIndex >= _parents.Count() || _footRIndex >= _parents.Count())
    {
        _legsResolveFailed = true;
        ResetFootLock();
        LOG(Warning, "Motion matching foot IK disabled: foot nodes not resolved.");
        return;
    }
    _calfLIndex = _parents[_footLIndex];
    _calfRIndex = _parents[_footRIndex];
    _thighLIndex = _calfLIndex >= 0 && _calfLIndex < _parents.Count() ? _parents[_calfLIndex] : -1;
    _thighRIndex = _calfRIndex >= 0 && _calfRIndex < _parents.Count() ? _parents[_calfRIndex] : -1;
    if (!ValidLeg(_thighLIndex, _calfLIndex, _footLIndex) ||
        !ValidLeg(_thighRIndex, _calfRIndex, _footRIndex))
    {
        _legsResolveFailed = true;
        ResetFootLock();
        LOG(Warning, "Motion matching foot IK disabled: bad leg chain L=({},{},{}) R=({},{},{}).",
            _thighLIndex, _calfLIndex, _footLIndex, _thighRIndex, _calfRIndex, _footRIndex);
        return;
    }
    float upperL, lowerL, upperR, lowerR;
    {
        const int32 count = _parents.Count();
        Array<Matrix> bindModel;
        bindModel.Resize(count);
        for (int32 i = 0; i < count; i++)
        {
            const int32 p = _parents[i];
            bindModel[i] = (p >= 0 && p < count)
                ? _referenceLocals[i] * bindModel[p]
                : _referenceLocals[i];
        }
        upperL = BindBoneLength(bindModel, _thighLIndex, _calfLIndex);
        lowerL = BindBoneLength(bindModel, _calfLIndex, _footLIndex);
        upperR = BindBoneLength(bindModel, _thighRIndex, _calfRIndex);
        lowerR = BindBoneLength(bindModel, _calfRIndex, _footRIndex);
    }
    if (!LegLengthsSane(upperL, lowerL) || !LegLengthsSane(upperR, lowerR))
    {
        _legsResolveFailed = true;
        LOG(Warning, "Motion matching foot IK disabled: implausible leg lengths L=({:.1f},{:.1f}) R=({:.1f},{:.1f}).",
            (double)upperL, (double)lowerL, (double)upperR, (double)lowerR);
        return;
    }
    _legsResolved = true;
    LOG(Info, "Motion matching foot IK: legs resolved L thigh={} calf={} foot={} ({:.1f}+{:.1f}) R thigh={} calf={} foot={} ({:.1f}+{:.1f}).",
        _thighLIndex, _calfLIndex, _footLIndex, (double)upperL, (double)lowerL,
        _thighRIndex, _calfRIndex, _footRIndex, (double)upperR, (double)lowerR);
}

Array<Matrix> MotionMatchingPlayback::ApplyFootIK(AnimatedModel* body, SkinnedModel* model, Array<Matrix>& locals, const Array<Matrix>& pose, float dt)
{
    LastFootCorrL = 0.0f;
    LastFootCorrR = 0.0f;
    ResolveLegs();
    if (!_legsResolved || _rootNodeIndex < 0 || _rootNodeIndex >= pose.Count())
        return pose;

    // Body world matrix, EXACTLY ONCE (C2 fix for the double transform).
    // Flax row-vector ground truth: a point transforms as p_world = p_body
    // * bodyWorld, and hierarchy composes child * parentWorld (cf. the
    // sampler: Multiply(locals[i], parentPose), parent on the RIGHT).
    // Hence: take world exactly once via body LocalToWorldMatrix (engine
    // composes local * parent chain itself). Parent-walking here is
    // FORBIDDEN. Pose space is the BODY actor's space, which is NOT the
    // MM-actor space (Mannequin carries a -90deg yaw under Mesh under
    // Player) — LocalToWorldMatrix of the BODY actor is the right frame.
    Matrix bodyWorld;
    body->GetLocalToWorldMatrix(bodyWorld);
    const Matrix bodyWorldInv = Matrix::Invert(bodyWorld);

    Float3 rootScale, rootT;
    Quaternion rootRot;
    pose[_rootNodeIndex].Decompose(rootScale, rootRot, rootT);

    Float3 targetL = Float3::Zero;
    Float3 targetR = Float3::Zero;
    UpdateFootLock(0, body, pose, rootT, bodyWorld, bodyWorldInv, dt,
        _footLIndex, LockStateL, FirmLockL, _weightL, _pinWL, _hasPinL,
        _prevFootWL, _releaseL, _plantingGraceL, _stateFramesL,
        LastFootLockL, LastFootWeightL, LastFootCorrL, targetL);
    UpdateFootLock(1, body, pose, rootT, bodyWorld, bodyWorldInv, dt,
        _footRIndex, LockStateR, FirmLockR, _weightR, _pinWR, _hasPinR,
        _prevFootWR, _releaseR, _plantingGraceR, _stateFramesR,
        LastFootLockR, LastFootWeightR, LastFootCorrR, targetR);
    _plantL = (LockStateL == LockState::Planting || LockStateL == LockState::Locked);
    _plantR = (LockStateR == LockState::Planting || LockStateR == LockState::Locked);
    _hasPrevFootW = true;
    _hasPrevRel = true;

    bool solved = false;
    if (LastFootCorrL > FootLockMinCorrection)
        solved |= SolveLeg(locals, pose, _thighLIndex, _calfLIndex, _footLIndex, targetL);
    if (LastFootCorrR > FootLockMinCorrection)
        solved |= SolveLeg(locals, pose, _thighRIndex, _calfRIndex, _footRIndex, targetR);
    if (!solved)
        return pose;

    // Local-space discipline: only thigh/calf local rotations changed
    // above, so rebuild the model pose with the same native FK instead
    // of hand-patching model-space matrices (keeps parents consistent,
    // bone lengths exact). On failure keep the pre-IK pose, never break.
    Array<Matrix> pose2;
    String composeError;
    if (!MotionMatchingSampler::ComposeModelPose(model, locals, pose2, composeError))
        return pose;
    return pose2;
}

void MotionMatchingPlayback::UpdateFootLock(int32 side, Actor* body, const Array<Matrix>& pose, const Float3& rootT,
    const Matrix& bodyWorld, const Matrix& bodyWorldInv, float dt, int32 footIndex,
    LockState& state, bool& firmLock, float& weight, Vector3& pinW, bool& hasPin,
    Vector3& prevFootW, float& releaseGrace, float& plantingGrace, int32& stateFrames,
    bool& teleLock, float& teleWeight, float& teleCorr, Float3& target)
{
    Float3 footScale, footT;
    Quaternion footRot;
    pose[footIndex].Decompose(footScale, footRot, footT);
    Vector3 footWV;
    Vector3::Transform(Vector3(footT.X, footT.Y, footT.Z), bodyWorld, footWV);

    // Clip-change rebase (see Play): adopt the current foot as the pin so
    // last clip's plant spot cannot yank this clip, and reseed velocities
    // so the switch displacement never reads as foot speed. States and
    // weights survive: the gates re-derive them in a tick or two.
    if (_footRebasePending)
    {
        if (hasPin)
            pinW = footWV;
        _hasPrevFootW = false;
        _hasPrevRel = false;
    }

    // C1 live contact on the playing sample. Two SEPARATE velocities:
    // - footVelocityWorld: world-space foot displacement/dt. The planted-
    //   foot measure (authoritative for the gate: a body-frame speed would
    //   inherit actor motion, so a fast run would never gate).
    // - poseVelocityRootRelative: root-relative foot displacement/dt in
    //   body space, the feature-space analogue of the bake/query velocity.
    //   Debug trace only — never the gate. A running actor shows why the
    //   split matters: root-relative planted feet read near 0 while the
    //   world foot correctly reports travel.
    float footVelocityWorld = 0.0f;
    if (_hasPrevFootW && dt > 0.0f)
    {
        const Vector3 dv = (footWV - prevFootW) / dt;
        footVelocityWorld = dv.Length();
        // Discontinuity guard: no foot outruns a sprinting body by an
        // order of magnitude — a teleport/switch the resets missed would
        // otherwise read 45k u/s and shatter every lock for a frame.
        if (footVelocityWorld > 3000.0f)
        {
            footVelocityWorld = 0.0f;
            _hasPrevFootW = false;
        }
    }
    const Float3 relativeFoot = footT - rootT;
    float poseVelocityRootRelative = 0.0f;
    if (_hasPrevRel && dt > 0.0f)
    {
        const Float3 prevRel = side == 0 ? _prevRelL : _prevRelR;
        const Float3 drv = (relativeFoot - prevRel) * (1.0f / dt);
        poseVelocityRootRelative = drv.Length();
    }
    // Contact height is BODY-SPACE vertical (footT.Y - rootT.Y), NOT the
    // root-local dot: this runtime gate only needs true ankle height to
    // decide plants (its thresholds are runtime-owned, independent of the
    // bake). Body-space Y is world-vertical for yaw-only bodies — the same
    // assumption the ground sweep and foot telemetry already rely on.
    // The root-local dot measured forward offset instead of height here
    // (median 15 vs a 12 gate: plants only on flicker dips, then yank).
    // Bake/query contact FEATURES are untouched (native root-relative math
    // stays consistent with the database by construction).
    const float height = footT.Y - rootT.Y;
    if (side == 0)
    {
        LastFootVelocityWorldL = footVelocityWorld;
        LastPoseVelocityRootRelativeL = poseVelocityRootRelative;
        LastContactHeightL = height;
    }
    else
    {
        LastFootVelocityWorldR = footVelocityWorld;
        LastPoseVelocityRootRelativeR = poseVelocityRootRelative;
        LastContactHeightR = height;
    }

    // H1 Foot-lock state machine (PLAN §5.1):
    // Free -> Planting -> Locked -> Releasing -> Free
    // Enter / exit contact gates with named thresholds & hysteresis.
    const float gateEnterSpeed = MaxF(FootLockEnterSpeed, 0.0f);
    const float gateExitSpeed = MaxF(FootLockExitSpeed, 0.0f);
    const float gateEnterHeight = FootLockEnterHeight;
    const float gateExitHeight = FootLockExitHeight;

    const bool contactEnter = footVelocityWorld <= gateEnterSpeed && height <= gateEnterHeight;
    const bool contactExit = footVelocityWorld <= gateExitSpeed && height <= gateExitHeight;

    float blendRate = dt / MaxF(FootBlendTime, 0.01f);
    // Anti-jitter: one hitch frame must not snap weight 0->1 (dt-spike
    // would instantly confirm a lock and yank it off next tick). Half
    // weight per tick max; real plants still confirm in ~3 ticks.
    blendRate = MinF(blendRate, 0.5f);
    stateFrames++;

    switch (state)
    {
    case LockState::Free:
        firmLock = false;
        weight = 0.0f;
        hasPin = false;
        if (contactEnter)
        {
            // Free -> Planting: pin world position on rising edge.
            state = LockState::Planting;
            stateFrames = 1;
            plantingGrace = MaxF(FootPlantingDelay, 0.0f);
            pinW = footWV;
            hasPin = true;
            weight = MinF(weight + blendRate, 1.0f);
            if (weight >= 1.0f && stateFrames >= FootPlantingMinFrames)
            {
                state = LockState::Locked;
                stateFrames = 1;
                firmLock = true;
                releaseGrace = MaxF(FootReleaseDelay, 0.0f);
            }
        }
        break;

    case LockState::Planting:
        firmLock = false;
        if (contactExit)
        {
            plantingGrace = MaxF(FootPlantingDelay, 0.0f);
        }
        else if (plantingGrace > 0.0f)
        {
            plantingGrace = MaxF(plantingGrace - MaxF(dt, 0.0f), 0.0f);
        }

        if (contactExit || plantingGrace > 0.0f)
        {
            weight = MinF(weight + blendRate, 1.0f);
            if (weight >= 1.0f && stateFrames >= FootPlantingMinFrames)
            {
                // Planting -> Locked: full weight achieved, lock is firm.
                state = LockState::Locked;
                stateFrames = 1;
                weight = 1.0f;
                firmLock = true;
                releaseGrace = MaxF(FootReleaseDelay, 0.0f);
            }
        }
        else
        {
            // Contact lost during planting and grace expired: transition to Releasing.
            state = LockState::Releasing;
            stateFrames = 1;
            weight = MaxF(weight - blendRate, 0.0f);
            if (weight <= 0.0f)
            {
                state = LockState::Free;
                stateFrames = 0;
                hasPin = false;
            }
        }
        break;

    case LockState::Locked:
        firmLock = true;
        weight = 1.0f;
        if (contactExit)
        {
            releaseGrace = MaxF(FootReleaseDelay, 0.0f);
        }
        else if (releaseGrace > 0.0f)
        {
            releaseGrace = MaxF(releaseGrace - MaxF(dt, 0.0f), 0.0f);
        }

        if (releaseGrace <= 0.0f && stateFrames >= FootLockedMinFrames)
        {
            // Locked -> Releasing: contact lost, grace expired, AND
            // the lock lived long enough to be a real stance.
            state = LockState::Releasing;
            stateFrames = 1;
            firmLock = false;
            weight = MaxF(weight - blendRate, 0.0f);
            if (weight <= 0.0f)
            {
                state = LockState::Free;
                stateFrames = 0;
                hasPin = false;
            }
        }
        break;

    case LockState::Releasing:
        firmLock = false;
        weight = MaxF(weight - blendRate, 0.0f);
        if (weight <= 0.0f && stateFrames >= FootReleasingMinFrames)
        {
            // Releasing -> Free: weight reached zero, pin cleared.
            state = LockState::Free;
            stateFrames = 0;
            weight = 0.0f;
            hasPin = false;
        }
        break;

    default:
        state = LockState::Free;
        firmLock = false;
        weight = 0.0f;
        hasPin = false;
        break;
    }

    target = footT;
    float corr = 0.0f;
    if (weight > FootLockMinWeight && hasPin)
    {
        Vector3 pinBodyV;
        Vector3::Transform(pinW, bodyWorldInv, pinBodyV);
        const Float3 pinBody(pinBodyV.X, pinBodyV.Y, pinBodyV.Z);
        Float3 t = footT + (pinBody - footT) * weight;
        // Ground conform on the ground PLANE (not Y-only): the ankle rides
        // AnkleToSoleOffset above the hit plane along its normal, so the
        // SOLE touches instead of the ankle joint. Raycast fail keeps the
        // pin target (preserves pin target, no snap).
        Vector3 groundW, groundNW;
        if (RaycastGround(footWV, body, groundW, groundNW))
        {
            Vector3 groundBodyV, nBodyV;
            Vector3::Transform(groundW, bodyWorldInv, groundBodyV);
            Vector3::TransformNormal(groundNW, bodyWorldInv, nBodyV);
            const float nLen = nBodyV.Length();
            if (nLen > 1e-6f)
            {
                nBodyV /= nLen;
                const Float3 gb(groundBodyV.X, groundBodyV.Y, groundBodyV.Z);
                const Float3 nb(nBodyV.X, nBodyV.Y, nBodyV.Z);
                const Float3 rel = t - gb;
                const float dist = rel.X * nb.X + rel.Y * nb.Y + rel.Z * nb.Z;
                const float want = MaxF(AnkleToSoleOffset, 0.0f);
                t = t - nb * ((dist - want) * weight);
            }
        }
        Float3 d = t - footT;
        float len = d.Length();
        const float maxC = MaxF(FootMaxCorrection, 0.0f);
        if (len > maxC && len > 0.000001f)
        {
            d = d * (maxC / len);
            len = maxC;
        }
        const float auth = ClampF(FootLockAuthority, 0.0f, 1.0f);
        d = d * auth;
        len = len * auth;
        target = footT + d;
        corr = len;
    }

    prevFootW = footWV;
    if (side == 0)
        _prevRelL = relativeFoot;
    else
        _prevRelR = relativeFoot;
    teleLock = (state == LockState::Planting || state == LockState::Locked);
    teleWeight = weight;
    teleCorr = corr;
}

bool MotionMatchingPlayback::RaycastGround(const Vector3& footW, Actor* body, Vector3& groundW, Vector3& groundNW)
{
    groundW = footW;
    groundNW = Vector3(0.0f, 1.0f, 0.0f);
    // Origin above the ankle (inside the capsule — PhysX skips the
    // containing shape; the self-reject rules below are the backstop).
    // Locomotion does no manual ground raycast (CharacterController owns
    // grounding), so there is no layer convention to reuse: all layers,
    // no triggers (flat floor + terrain in PlayerTestScene).
    Vector3 origin = footW + Vector3(0.0f, 30.0f, 0.0f);
    const Vector3 down(0.0f, -1.0f, 0.0f);
    const float radius = MaxF(FootSweepRadius, 0.5f);
    for (int32 attempt = 0; attempt < 3; attempt++)
    {
        RayCastHit hit;
        if (!Physics::SphereCast(origin, radius, down, hit, 130.0f, MAX_uint32, false))
            return false;
        const Vector3 p = hit.Point;
        // Self-reject: our own capsule (ancestry) or anything at/above
        // ankle level (ground under a ~10cm ankle reads well below it).
        const bool lineal = hit.Collider != nullptr;
        const bool self = (lineal && IsSelfHit(hit.Collider, body)) || p.Y > footW.Y - 5.0f;
        const bool flatEnough = hit.Normal.Y > 0.4f;
        if (!self && flatEnough && hit.Distance > 2.0f)
        {
            groundW = p;
            groundNW = hit.Normal;
            return true;
        }
        // Step past self geometry and retry (capsule exit -> terrain).
        origin = p + Vector3(0.0f, -3.0f, 0.0f);
    }
    return false;
}

bool MotionMatchingPlayback::IsSelfHit(PhysicsColliderActor* collider, Actor* body)
{
    if (collider == nullptr || body == nullptr)
        return false;
    int32 g1 = 0;
    for (Actor* c = collider; c != nullptr && g1 < 8; c = c->GetParent(), g1++)
    {
        int32 g2 = 0;
        for (Actor* b = body; b != nullptr && g2 < 8; b = b->GetParent(), g2++)
        {
            if (c == b)
                return true;
        }
    }
    return false;
}

bool MotionMatchingPlayback::SolveLeg(Array<Matrix>& locals, const Array<Matrix>& pose, int32 thigh, int32 calf, int32 ankle, const Float3& target)
{
    const int32 n = pose.Count();
    if (thigh < 0 || calf < 0 || ankle < 0 || thigh >= n || calf >= n || ankle >= n)
        return false;
    Float3 sT, h, sC, k, sA, a;
    Quaternion rTw, rCw, rAw;
    pose[thigh].Decompose(sT, rTw, h);
    pose[calf].Decompose(sC, rCw, k);
    pose[ankle].Decompose(sA, rAw, a);
    if (!IsFiniteV(h) || !IsFiniteV(k) || !IsFiniteV(a) || !IsFiniteV(target))
        return false;
    // Scale ~1 assert (rotation-only writeback is only valid for rigid
    // bones). Non-fatal: warn once, keep animated pose for the frame.
    if (!NearUnity(sT) || !NearUnity(sC) || !NearUnity(sA))
    {
        if (!_scaleWarned)
        {
            _scaleWarned = true;
            LOG(Warning, "Motion matching leg solve skipped: bone scale not ~1 T=({:.3f},{:.3f},{:.3f}) C=({:.3f},{:.3f},{:.3f}) A=({:.3f},{:.3f},{:.3f}). Keeping animated pose.",
                (double)sT.X, (double)sT.Y, (double)sT.Z, (double)sC.X, (double)sC.Y, (double)sC.Z,
                (double)sA.X, (double)sA.Y, (double)sA.Z);
        }
        return false;
    }

    const Float3 kh = k - h;
    const Float3 ak = a - k;
    const float l1 = kh.Length();
    const float l2 = ak.Length();
    if (!std::isfinite(l1) || !std::isfinite(l2))
        return false;
    if (l1 < 1.0f || l2 < 1.0f)
        return false;
    const Float3 d = target - h;
    const float dist = d.Length();
    if (!std::isfinite(dist) || dist < SolveMinDist)
        return false;
    const float maxR = (l1 + l2) * ReachMaxScale;
    const float minR = AbsF(l1 - l2) + ReachMinEpsilon;
    float dc = dist;
    if (dc > maxR)
        dc = maxR;
    if (dc < minR)
        dc = minR;
    const Float3 t = h + d * (dc / dist);
    const Float3 dir = (t - h) * (1.0f / dc);

    const Float3 bend = kh - dir * DotF(kh, dir);
    const float bendLen = bend.Length();
    if (!std::isfinite(bendLen) || bendLen < PoleMinBend)
    {
        // Straight-leg pole fallback: pole undefined, keep animated pose.
        if (!_poleWarned)
        {
            _poleWarned = true;
            LOG(Warning, "Motion matching leg solve: pole degenerate (bend={:.4f}cm), keeping animated pose for this frame.", (double)bendLen);
        }
        return false;
    }
    const Float3 pole = bend * (1.0f / bendLen);

    const float a1 = (l1 * l1 - l2 * l2 + dc * dc) / (2.0f * dc);
    const float h2 = l1 * l1 - a1 * a1;
    const float hh = h2 > 0.0f ? (float)std::sqrt((double)h2) : 0.0f;
    Float3 knew = h + dir * a1 + pole * hh;

    // Zero-norm guard: Norm returns Zero below epsilon, and a zero input
    // would make RotationFromTo's dot/cross meaningless (NaN risk via
    // normalize-of-zero downstream). Bail out to animated pose instead.
    const Float3 fromT = NormV(kh);
    const Float3 toT = NormV(knew - h);
    if (fromT.Length() < 0.5f || toT.Length() < 0.5f)
        return false;
    const Quaternion qT = RotationFromTo(fromT, toT);
    const Quaternion rTwNew = QuatNorm(qT * rTw);
    const int32 parentThigh = thigh >= 0 && thigh < _parents.Count() ? _parents[thigh] : -1;
    Quaternion rPw = Quaternion::Identity;
    if (parentThigh >= 0 && parentThigh < n)
    {
        Float3 sP, tP;
        Quaternion rP;
        pose[parentThigh].Decompose(sP, rP, tP);
        if (!NearUnity(sP))
        {
            if (!_scaleWarned)
            {
                _scaleWarned = true;
                LOG(Warning, "Motion matching leg solve skipped: parent scale not ~1. Keeping animated pose.");
            }
            return false;
        }
        rPw = rP;
    }
    // Hamilton world-to-local: inv(P) * D (NOT D * inv(P)).
    const Quaternion rTlocalNew = QuatNorm(QuatInverse(rPw) * rTwNew);

    const Float3 fromC = NormV(ak);
    const Float3 toC = NormV(t - knew);
    if (fromC.Length() < 0.5f || toC.Length() < 0.5f)
        return false;
    const Quaternion qC = RotationFromTo(fromC, toC);
    const Quaternion rCwNew = QuatNorm(qC * rCw);
    const Quaternion rClocalNew = QuatNorm(QuatInverse(rTwNew) * rCwNew);

    Float3 lsT, ltT, lsC, ltC;
    Quaternion lrT, lrC;
    locals[thigh].Decompose(lsT, lrT, ltT);
    Matrix::Transformation(lsT, rTlocalNew, ltT, locals[thigh]);
    locals[calf].Decompose(lsC, lrC, ltC);
    Matrix::Transformation(lsC, rClocalNew, ltC, locals[calf]);
    return true;
}

bool MotionMatchingPlayback::IsFiniteV(const Float3& value)
{
    return std::isfinite(value.X) && std::isfinite(value.Y) && std::isfinite(value.Z);
}

bool MotionMatchingPlayback::NearUnity(const Float3& value)
{
    return AbsF(value.X - 1.0f) <= ScaleUnityTolerance &&
        AbsF(value.Y - 1.0f) <= ScaleUnityTolerance &&
        AbsF(value.Z - 1.0f) <= ScaleUnityTolerance;
}

float MotionMatchingPlayback::DotF(const Float3& a, const Float3& b)
{
    return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
}

Float3 MotionMatchingPlayback::CrossF(const Float3& a, const Float3& b)
{
    return Float3(
        a.Y * b.Z - a.Z * b.Y,
        a.Z * b.X - a.X * b.Z,
        a.X * b.Y - a.Y * b.X);
}

Float3 MotionMatchingPlayback::NormV(const Float3& value)
{
    const float len = value.Length();
    if (!std::isfinite(len) || len < NormZeroEpsilon)
        return Float3::Zero;
    return value * (1.0f / len);
}

Quaternion MotionMatchingPlayback::QuatNorm(const Quaternion& value)
{
    const float len = (float)std::sqrt((double)(value.X * value.X + value.Y * value.Y + value.Z * value.Z + value.W * value.W));
    if (len < 0.0000001f)
        return Quaternion::Identity;
    return Quaternion(value.X / len, value.Y / len, value.Z / len, value.W / len);
}

Quaternion MotionMatchingPlayback::QuatInverse(const Quaternion& value)
{
    // Unit-quaternion inverse = conjugate (all inputs here are unit:
    // Decompose outputs and FromTo products of unit quats).
    const Quaternion c(-value.X, -value.Y, -value.Z, value.W);
    return QuatNorm(c);
}

Quaternion MotionMatchingPlayback::RotationFromTo(const Float3& from, const Float3& to)
{
    // Zero/non-finite guard: callers must pass unit vectors, but a zero
    // Norm output (degenerate segment) would otherwise flow into dot/cross
    // and produce a NaN quaternion. Fail safe to identity (keep animated).
    if (!IsFiniteV(from) || !IsFiniteV(to))
        return Quaternion::Identity;
    const float fromLenSq = from.X * from.X + from.Y * from.Y + from.Z * from.Z;
    const float toLenSq = to.X * to.X + to.Y * to.Y + to.Z * to.Z;
    if (!(fromLenSq > 0.25f) || !(toLenSq > 0.25f))
        return Quaternion::Identity;
    const float c = DotF(from, to);
    if (c >= 1.0f - 0.000001f)
        return Quaternion::Identity;
    if (c <= -1.0f + 0.000001f)
    {
        // 180deg: rotate about any perpendicular.
        const float ax = AbsF(from.X);
        const float ay = AbsF(from.Y);
        const float az = AbsF(from.Z);
        const Float3 t = (ax < ay)
            ? (ax < az ? Float3(1.0f, 0.0f, 0.0f) : Float3(0.0f, 0.0f, 1.0f))
            : (ay < az ? Float3(0.0f, 1.0f, 0.0f) : Float3(0.0f, 0.0f, 1.0f));
        const Float3 axis = NormV(CrossF(from, t));
        if (axis.Length() < 0.5f)
            return Quaternion::Identity; // degenerate perpendicular: keep animated
        return QuatNorm(Quaternion(axis.X, axis.Y, axis.Z, 0.0f));
    }
    const Float3 axis2 = CrossF(from, to);
    const float s = axis2.Length();
    if (s < 0.0000001f)
        return Quaternion::Identity;
    const Float3 axisN = axis2 * (1.0f / s);
    const float angle = (float)std::atan2((double)s, (double)c);
    const float sh = (float)std::sin((double)angle * 0.5);
    const float ch = (float)std::cos((double)angle * 0.5);
    return QuatNorm(Quaternion(axisN.X * sh, axisN.Y * sh, axisN.Z * sh, ch));
}

void MotionMatchingPlayback::WarnOnce(const String& message)
{
    if (_warned)
        return;
    _warned = true;
    LOG(Warning, "{}", message);
}
