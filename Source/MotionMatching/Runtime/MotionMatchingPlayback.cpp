#include "MotionMatchingPlayback.h"

#include "MotionMatchingSampler.h"
#include "../Database/MotionMatchingDatabase.h"
#include "Engine/Content/Assets/Animation.h"
#include "Engine/Content/Assets/SkinnedModel.h"
#include "Engine/Core/Log.h"
#include "Engine/Core/Math/Math.h"
#include "Engine/Level/Actors/AnimatedModel.h"
#include "Engine/Level/Actor.h"

#include <cmath>

namespace
{
    constexpr float MinPlayRate = 0.85f;
    constexpr float MaxPlayRate = 1.15f;
    constexpr float PlayRateEpsilon = 1e-4f;
    constexpr float ScaleUnityTolerance = 0.05f;
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









void MotionMatchingPlayback::Setup(
    MotionMatchingDatabase* database,
    SkinnedModel* fallbackModel,
    const SkeletonProfile& skeleton,
    const Float3& controllerUpAxis)
{
    Stop();
    _database = database;
    _fallbackModel = fallbackModel;
    (void)controllerUpAxis;
    _rootNodeIndex = -1;
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
    EnsureHierarchy(resolveModel);
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
    _searchLocals.Clear();
    _prevSearchLocals.Clear();
    _tickFrameId = 0;
    SampleFrameId = -1;
    SearchFrameId = -1;
    SaveSearchPoseFrameId = -1;
    _orderWarned = false;
    ClearInertial();
}









void MotionMatchingPlayback::ResetOnTeleport()
{
    ClearInertial();
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
        ArmInertial();
        CurrentTime = target;
        IsAtEnd = false;
        return;
    }

    ArmInertial();
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



void MotionMatchingPlayback::AssertTickOrder()
{
    // P2 intra-tick invariant: sample -> save, with the policy search
    // stamped between (NotifySearchExecuted). No layer/IK stages exist.
    bool ok = SampleFrameId == _tickFrameId &&
        SaveSearchPoseFrameId == _tickFrameId;
    if (SearchFrameId == _tickFrameId || SearchFrameId == -1)
    {
    }
    else if (SearchFrameId != _tickFrameId)
    {
        ok = false;
    }
    if (!ok && !_orderWarned)
    {
        _orderWarned = true;
        LOG(Warning, "Motion matching tick order violated: tick={} sample={} search={} save={}. Expected sample<=search<=save.",
            _tickFrameId, SampleFrameId, SearchFrameId, SaveSearchPoseFrameId);
    }
}

void MotionMatchingPlayback::Tick(float deltaTime, AnimatedModel* body, bool grounded)
{
    (void)grounded;

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
    // Inertial source is _searchLocals (post-strip, post-blend snapshot),
    // never any displayed pose — see BuildInertialOffsets.
    Array<Matrix>& blendedLocals = *stripped;
    ApplyInertialization(blendedLocals, model, MaxF(deltaTime, 0.000001f));

    // Search-pose save: snapshot AFTER inertial. Deep copy (not an alias)
    // so later game-side post passes cannot mutate the saved snapshot.
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
    // P2: no overlay / compose / IK / pose write here. The game reads
    // the search locals above via GetSearchLocals, applies its own post
    // passes, composes and writes the final pose itself.
    AssertTickOrder();
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



































void MotionMatchingPlayback::WarnOnce(const String& message)
{
    if (_warned)
        return;
    _warned = true;
    LOG(Warning, "{}", message);
}
