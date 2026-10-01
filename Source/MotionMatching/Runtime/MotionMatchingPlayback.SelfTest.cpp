#include "MotionMatchingPlayback.h"

#include "../Database/MotionMatchingDatabase.h"
#include "MotionMatchingSampler.h"
#include "Engine/Content/Assets/Animation.h"
#include "Engine/Content/Assets/SkinnedModel.h"
#include "Engine/Core/Log.h"
#include "Engine/Core/Math/Math.h"
#include "Engine/Level/Actors/AnimatedModel.h"

#include <cmath>

#if USE_EDITOR

// Playback self-test: time advance, play-rate clamp range, one-shot end and
// loop wrap, finite output pose and root-motion stripping. Each case FAILS
// LOUDLY when its fixture class is missing from the database (no silent pass
// on zero probes). Searching is covered by MotionMatchingSearch.SelfTest.cpp.

namespace
{
    constexpr float TimeTol = 1e-4f;
    constexpr float PoseTol = 1e-3f;

    float AbsF(float value)
    {
        return value < 0.0f ? -value : value;
    }

    float ClipLength(MotionMatchingDatabase* database, int32 clip)
    {
        const Array<float>& lengths = database->GetClipLengths();
        return lengths.IsValidIndex(clip) ? lengths[clip] : 0.0f;
    }

    // First clip of the wanted kind that is long enough to step through.
    int32 FindClip(MotionMatchingDatabase* database, bool loop, float minLength)
    {
        for (int32 i = 0; i < database->GetClipCount(); i++)
        {
            if (database->GetClipAnimation(i) == nullptr || database->GetClipLoop(i) != loop)
                continue;
            if (ClipLength(database, i) >= minLength)
                return i;
        }
        return -1;
    }

    bool IsFiniteMatrix(const Matrix& value)
    {
        Float3 scale;
        Quaternion rotation;
        Float3 translation;
        value.Decompose(scale, rotation, translation);
        return std::isfinite(scale.X + scale.Y + scale.Z + translation.X + translation.Y + translation.Z +
            rotation.X + rotation.Y + rotation.Z + rotation.W) != 0;
    }
}

bool MotionMatchingPlayback::RunPlaybackSelfTest(MotionMatchingDatabase* database, AnimatedModel* body, String& log)
{
    log = String::Empty;
    if (database == nullptr || database->WaitForLoaded())
    {
        log = TEXT("Playback self-test failed: database is not loaded.");
        return false;
    }
    if (!database->IsBaked() || !database->ValidateData())
    {
        log = TEXT("Playback self-test failed: database is not baked or not valid.");
        return false;
    }
    if (body == nullptr)
    {
        log = TEXT("Playback self-test failed: body actor is missing (pass CharacterBody from the facade).");
        return false;
    }
    SkinnedModel* model = body->SkinnedModel.Get();
    if (model == nullptr || model->WaitForLoaded())
    {
        log = TEXT("Playback self-test failed: body has no loaded SkinnedModel.");
        return false;
    }

    const PoseSearchSchema& schema = database->GetSchema();
    NodeIndices nodes;
    String resolveError;
    if (!MotionMatchingSkeleton::TryResolve(schema.Skeleton, model, nodes, resolveError))
    {
        log = String::Format(TEXT("Playback self-test failed: skeleton not resolved for this rig: {}."), resolveError);
        return false;
    }
    if (nodes.Root < 0)
    {
        log = TEXT("Playback self-test failed: skeleton profile resolved without a root node.");
        return false;
    }

    const float dt = schema.SampleRate > 0 ? 1.0f / (float)schema.SampleRate : 1.0f / 30.0f;
    const int32 loopClip = FindClip(database, true, dt * 4.0f);
    const int32 oneShotClip = FindClip(database, false, dt * 4.0f);
    if (loopClip < 0 || oneShotClip < 0)
    {
        log = String::Format(TEXT("Playback self-test failed: need a loop clip and a one-shot clip of at least 4 frames (loop {}, one-shot {})."),
            loopClip, oneShotClip);
        return false;
    }

    int32 passed = 0;
    int32 tested = 0;
    auto check = [&](bool ok, const String& label, const String& detail) -> bool
    {
        tested++;
        if (!ok)
            log += String::Format(TEXT("{} FAIL: {}.\n"), label, detail);
        else
            passed++;
        return ok;
    };

    MotionMatchingPlayback playback;
    playback.Setup(database, model, schema.Skeleton);

    // Time advance: one tick moves the clip time by dt * rate.
    playback.SetPlayRate(1.0f);
    playback.Play(loopClip, 0.0f, true);
    playback.Tick(dt, body);
    check(AbsF(playback.CurrentTime - dt) <= TimeTol,
        TEXT("advance"),
        String::Format(TEXT("loop clip {}: time {:.6f}, want {:.6f}"), loopClip, playback.CurrentTime, dt));

    // Play-rate range: default clamp, custom clamp, invalid range ignored.
    playback.SetPlayRate(1.1f);
    playback.Play(loopClip, 0.0f, true);
    playback.Tick(dt, body);
    check(AbsF(playback.CurrentTime - dt * 1.1f) <= TimeTol,
        TEXT("rate-scales-time"),
        String::Format(TEXT("time {:.6f}, want {:.6f}"), playback.CurrentTime, dt * 1.1f));
    playback.SetPlayRate(3.0f);
    check(AbsF(playback.GetPlayRate() - 1.15f) <= TimeTol,
        TEXT("default-rate-clamp"),
        String::Format(TEXT("rate {:.4f}, want 1.15"), playback.GetPlayRate()));
    playback.SetPlayRateRange(0.5f, 2.0f);
    playback.SetPlayRate(1.8f);
    check(AbsF(playback.GetPlayRate() - 1.8f) <= TimeTol,
        TEXT("custom-rate-range"),
        String::Format(TEXT("rate {:.4f}, want 1.8"), playback.GetPlayRate()));
    playback.SetPlayRateRange(2.0f, 1.0f);
    playback.SetPlayRate(5.0f);
    check(AbsF(playback.GetPlayRate() - 2.0f) <= TimeTol,
        TEXT("invalid-range-ignored"),
        String::Format(TEXT("rate {:.4f}, want 2.0 (range stays 0.5..2.0)"), playback.GetPlayRate()));
    playback.SetPlayRateRange(0.85f, 1.15f);
    playback.SetPlayRate(1.0f);

    // One-shot end: time holds at the clip length and IsAtEnd is raised.
    {
        const float len = ClipLength(database, oneShotClip);
        playback.Play(oneShotClip, len - dt * 0.5f, true);
        playback.Tick(dt, body);
        check(playback.IsAtEnd && AbsF(playback.CurrentTime - len) <= TimeTol,
            TEXT("one-shot-end"),
            String::Format(TEXT("clip {}: atEnd {}, time {:.6f}, want {:.6f}"), oneShotClip, playback.IsAtEnd ? 1 : 0, playback.CurrentTime, len));
    }

    // Loop wrap: time wraps into [0, length) and the cycle counter advances.
    {
        const float len = ClipLength(database, loopClip);
        playback.Play(loopClip, len - dt * 0.5f, true);
        const int32 cycleBefore = playback.ClipCycle;
        playback.Tick(dt, body);
        check(playback.ClipCycle == cycleBefore + 1 && playback.CurrentTime >= 0.0f && playback.CurrentTime < len,
            TEXT("loop-wrap"),
            String::Format(TEXT("clip {}: cycle {} -> {}, time {:.6f} of {:.6f}"), loopClip, cycleBefore, playback.ClipCycle, playback.CurrentTime, len));
    }

    // Output pose: finite, and the root carries no animated travel or yaw
    // (stripped to the model's bind translation and rotation).
    {
        MotionMatchingPlayback fresh;
        fresh.Setup(database, model, schema.Skeleton);
        fresh.SetPlayRate(1.0f);
        fresh.Play(loopClip, ClipLength(database, loopClip) * 0.5f, true);
        fresh.Tick(dt, body);
        const int32 count = fresh.GetSearchLocalsCount();
        bool allFinite = count > nodes.Root;
        for (int32 i = 0; i < count && allFinite; i++)
        {
            Matrix local;
            allFinite = fresh.GetSearchLocal(i, local) && IsFiniteMatrix(local);
        }
        check(allFinite,
            TEXT("pose-finite"),
            String::Format(TEXT("bones {} (root index {})"), count, nodes.Root));

        Matrix rootLocal;
        bool stripped = false;
        float dist = 0.0f;
        float dot = 0.0f;
        const auto& modelNodes = model->GetNodes();
        if (modelNodes.IsValidIndex(nodes.Root) && fresh.GetSearchLocal(nodes.Root, rootLocal))
        {
            Float3 scale;
            Quaternion rotation;
            Float3 translation;
            rootLocal.Decompose(scale, rotation, translation);
            const Transform& bind = modelNodes[nodes.Root].LocalTransform;
            dist = Float3::Distance(translation, bind.Translation);
            dot = AbsF(rotation.X * bind.Orientation.X + rotation.Y * bind.Orientation.Y +
                rotation.Z * bind.Orientation.Z + rotation.W * bind.Orientation.W);
            stripped = dist <= PoseTol && dot >= 1.0f - PoseTol;
        }
        check(stripped,
            TEXT("root-stripped"),
            String::Format(TEXT("root translation off by {:.5f}, rotation dot {:.6f}"), dist, dot));
    }

    log += String::Format(TEXT("Playback self-test: {}/{} passed (clips={}, loop={}, one-shot={}, dt={:.5f}).\n"),
        passed, tested, database->GetClipCount(), loopClip, oneShotClip, dt);
    return passed == tested;
}

#endif // USE_EDITOR
