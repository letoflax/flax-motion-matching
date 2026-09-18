#include "MotionMatchingRuntimePolicy.h"

#include "../Database/MotionMatchingDatabase.h"
#include "../Search/MotionMatchingSearch.h"
#include "Engine/Content/Assets/Animation.h"
#include "Engine/Content/Assets/SkinnedModel.h"
#include "Engine/Core/Log.h"
#include "Engine/Core/Math/Math.h"
#include "Engine/Engine/Time.h"
#include "Engine/Level/Actors/AnimatedModel.h"
#include "Engine/Level/Actor.h"
#include "Engine/Platform/FileSystem.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace
{
    constexpr float WarmupTime = 0.5f;
    constexpr float TeleportDistance = 2000.0f;
    constexpr float TravelingDistance = 50.0f;
    constexpr float MaxFastPathAge = 0.5f;
    constexpr int32 MaxFastPathFrames = 5;
    constexpr float ReselectHistoryDuration = 0.3f;
    constexpr float ReselectSampleWindow = 0.25f;
    constexpr bool EnableReselectBan = true;
    constexpr bool EnableLoopTailExclusion = false;
    constexpr float SameSegmentTwinBan = 0.12f;
    constexpr float NonLoopTailExclusion = 0.3f;
    constexpr float JumpRiseVelY = 300.0f;
    constexpr int32 JumpMinAirTicks = 6;
    constexpr float JumpLandHoldLight = 0.15f;
    constexpr float JumpLandHoldHeavy = 0.30f;
    constexpr float JumpLandHoldSevere = 0.45f;
    constexpr float JumpFallVelY = -50.0f;
    constexpr float JumpTakeoffMaxTime = 0.45f;
    constexpr float JumpLightMaxAir = 0.55f;
    constexpr float JumpSevereMinAir = 1.6f;
    constexpr float JumpBlendLight = 0.08f;
    constexpr float JumpBlendHeavy = 0.18f;
    constexpr float JumpBlendSevere = 0.25f;
    constexpr float PiF = 3.14159265358979323846f;

    void FormatFloatText(float value, char* buffer, int32 capacity)
    {
        if (std::isinf(value))
        {
            snprintf(buffer, (size_t)capacity, "%s", value > 0.0f ? "1e30" : "-1e30");
            return;
        }
        if (std::isnan(value))
        {
            snprintf(buffer, (size_t)capacity, "0");
            return;
        }
        snprintf(buffer, (size_t)capacity, "%.9g", (double)value);
        for (int32 i = 0; buffer[i] != '\0'; i++)
        {
            if (buffer[i] == ',')
                buffer[i] = '.';
        }
    }
}

float MotionMatchingRuntimePolicy::ClampF(float value, float minValue, float maxValue)
{
    if (value < minValue)
        return minValue;
    if (value > maxValue)
        return maxValue;
    return value;
}

float MotionMatchingRuntimePolicy::MaxF(float a, float b)
{
    return a > b ? a : b;
}

float MotionMatchingRuntimePolicy::MinF(float a, float b)
{
    return a < b ? a : b;
}

float MotionMatchingRuntimePolicy::AbsF(float value)
{
    return value < 0.0f ? -value : value;
}

int32 MotionMatchingRuntimePolicy::MaxI(int32 a, int32 b)
{
    return a > b ? a : b;
}

bool MotionMatchingRuntimePolicy::GetIsReady() const
{
    return _samplingReady;
}

MotionMatchingRuntimePolicy::~MotionMatchingRuntimePolicy()
{
    Shutdown();
}

MotionMatchingPlayback* MotionMatchingRuntimePolicy::GetPlayback()
{
    return _playback;
}

MotionMatchingTraceLog* MotionMatchingRuntimePolicy::GetTrace()
{
    return _trace;
}

MotionMatchingTraceEntry MotionMatchingRuntimePolicy::GetLastEntry() const
{
    return _lastEntry;
}

void MotionMatchingRuntimePolicy::Initialize(
    MotionMatchingDatabase* database,
    AnimatedModel* body,
    const PoseSearchSchema& schema,
    const String& cacheFolder,
    float defaultInertial)
{
    Shutdown();
    _database = database;
    _body = body;
    _liveSchema = schema;
    _cacheFolder = cacheFolder;
    (void)defaultInertial;
    _playback = New<MotionMatchingPlayback>();
    _trace = New<MotionMatchingTraceLog>();
    SkinnedModel* fallbackModel = nullptr;
    Float3 controllerUp(0.0f, 0.0f, -1.0f);
    if (body != nullptr)
    {
        fallbackModel = body->SkinnedModel.Get();
        _playback->Setup(database, fallbackModel, schema.Skeleton, schema.RootUpAxis);
    }
    else
    {
        _playback->Setup(database, nullptr, schema.Skeleton, schema.RootUpAxis);
    }
    _queryTimer = 0.0f;
    _timeSinceSwitch = 999.0f;
    _timeSinceInit = 0.0f;
    _reselectClip = -1;
    _reselectStartTime = -99.0f;
    _lastScanTotal = MAX_float;
    _lastFullScanTime = -99.0f;
    _lastFullScanFrame = -1;
    _consecutiveContinuationMisses = 0;
    _fastPathFrames = 0;
    _hasLastScanSignature = false;
    _fastPathReseed = true;
    ForcedReason = String::Empty;
    _policyFrame = 0;
    _nextSearchFrameId = 1;
    _lastObservedSampleFrameId = -1;
    _hasSampleStarts = false;
    _tickTrajectory.Clear();
    FastPathHits = 0;
    LastFastPathAge = 0.0f;
    _currentClip = -1;
    _wasAirborne = false;
    _hasLastActorPos = false;
    _poseValid = false;
    _latestPose.Clear();
    _latestRootWorld = Matrix::Identity;

    if (database == nullptr || body == nullptr)
        return;

    // Manual-pose ownership: a bound AnimGraph would overwrite every
    // SetCurrentPose write. Warn once; do not silently reconfigure.
    if (body->AnimationGraph.Get() != nullptr)
        LOG(Warning, "Motion matching playback expects no AnimGraph on the body: the graph would overwrite manually written poses every frame.");

    SkinnedModel* model = body->SkinnedModel.Get();
    if (model == nullptr)
    {
        LOG(Warning, "Motion matching policy disabled: no SkinnedModel on the body.");
        return;
    }

    String resolveError;
    if (!MotionMatchingSkeleton::TryResolve(schema.Skeleton, model, _nodes, resolveError))
    {
        const StringAnsi errorAnsi(resolveError);
        char message[512];
        snprintf(message, sizeof(message), "Motion matching policy disabled: %s", errorAnsi.GetText());
        LOG(Warning, "{}", String(message));
        return;
    }

    _prevPositions.Resize(21);
    for (int32 i = 0; i < 21; i++)
        _prevPositions[i] = 0.0f;
    _packedTrajectory.Resize(36);
    for (int32 i = 0; i < 36; i++)
        _packedTrajectory[i] = 0.0f;
    _hasPrevPositions = false;
    _usedSearchSnapshot = false;
    _searchSnapshotLength = 0;
    MeasureClipMotion();
    _samplingReady = true;
    PolicyState = String(TEXT("Locomotion"));
    _trace->Clear();
    _lastEntry = MotionMatchingTraceEntry();
    LastSwitchReason = String(TEXT("init"));
    ChooserStatusText = String::Empty;
}

void MotionMatchingRuntimePolicy::Shutdown()
{
    _samplingReady = false;
    _database = nullptr;
    _body = nullptr;
    if (_playback != nullptr)
    {
        Delete(_playback);
        _playback = nullptr;
    }
    if (_trace != nullptr)
    {
        Delete(_trace);
        _trace = nullptr;
    }
    _clipTravel.Clear();
    _clipTurnAngle.Clear();
    _clipNet.Clear();
    _clipTags.Clear();
    PolicyState = String(TEXT("Disabled"));
}

void MotionMatchingRuntimePolicy::MeasureClipMotion()
{
    _clipTravel.Clear();
    _clipTurnAngle.Clear();
    _clipNet.Clear();
    _clipTags.Clear();
    MotionMatchingDatabase* db = _database;
    if (db == nullptr || !db->IsBaked())
        return;

    const int32 clipCount = db->GetClipCount();
    const Array<int32>& counts = db->GetClipSampleCounts();
    const Array<Float3>& positions = db->GetRootPositions();
    const Array<Float3>& forwards = db->GetRootForwards();
    if (counts.Count() == 0 || positions.Count() == 0 || forwards.Count() == 0)
        return;

    _clipTravel.Resize(clipCount);
    _clipTurnAngle.Resize(clipCount);
    _clipNet.Resize(clipCount);
    // Snapshot tag SETS once: every per-clip/per-frame reader below goes
    // through ClipTags() and never touches native strings again.
    _clipTags.Resize(clipCount);
    for (int32 c = 0; c < clipCount; c++)
        _clipTags[c] = db->GetClipTags(c);
    int32 cursor = 0;
    for (int32 c = 0; c < clipCount && cursor < positions.Count(); c++)
    {
        float path = 0.0f;
        float yPath = 0.0f;
        float yaw = 0.0f;
        const int32 n = c < counts.Count() ? counts[c] : 0;
        Float3 first = cursor < positions.Count() ? positions[cursor] : Float3::Zero;
        Float3 last = first;
        for (int32 i = 1; i < n && cursor + i < positions.Count() && cursor + i < forwards.Count(); i++)
        {
            const Float3 dp = positions[cursor + i] - positions[cursor + i - 1];
            path += Float2(dp.X, dp.Z).Length();
            yPath += AbsF(dp.Y);
            last = positions[cursor + i];
            const Float2 a(forwards[cursor + i - 1].X, forwards[cursor + i - 1].Z);
            const Float2 b(forwards[cursor + i].X, forwards[cursor + i].Z);
            if (a.LengthSquared() > 0.000001f && b.LengthSquared() > 0.000001f)
            {
                // Signed yaw travel (radians, engine convention): same
                // atan2(x, z) formula everywhere, so bucket matching and
                // facing steering agree by design.
                // NOTE: ports the C# verbatim (Atan2(a.X, a.Y) Ã¢â‚¬â€ Float2
                // fields are X=first, Y=second, i.e. world X and Z).
                const float ya = atan2f(a.X, a.Y);
                const float yb = atan2f(b.X, b.Y);
                float dy = yb - ya;
                while (dy > PiF)
                    dy -= 2.0f * PiF;
                while (dy < -PiF)
                    dy += 2.0f * PiF;
                yaw += dy;
            }
            // NOTE (P2): per-sample yaw table deleted with the turn driver;
            // clip totals above are kept for the search straightness prior.
        }
        const float net = Float2(last.X - first.X, last.Z - first.Z).Length();
        _clipTravel[c] = path;
        _clipTurnAngle[c] = yaw;
        _clipNet[c] = net;
        cursor += n;
    }
    LOG(Info, "Motion matching clip audit: {} clips measured.", clipCount);
    LogAccelAudit();
    WriteClipAuditFile();
}

void MotionMatchingRuntimePolicy::LogAccelAudit()
{
    MotionMatchingDatabase* db = _database;
    if (db == nullptr || !db->IsBaked())
        return;
    const Array<int32>& counts = db->GetClipSampleCounts();
    const Array<Float3>& velocities = db->GetRootVelocities();
    if (counts.Count() == 0 || velocities.Count() == 0)
        return;
    const float dt = 1.0f / MaxF(_liveSchema.SampleRate, 1.0f);
    Array<String> tagNames;
    Array<float> tagPeak;
    Array<double> tagSum;
    Array<int32> tagNum;
    int32 cursor = 0;
    const int32 clipCount = db->GetClipCount();
    for (int32 c = 0; c < clipCount && cursor < velocities.Count(); c++)
    {
        const int32 n = c < counts.Count() ? counts[c] : 0;
        // Multi-tag: one sample's accel counts into EVERY tag its clip
        // carries (membership counting, same as RebuildTagCounts).
        const Array<String>& tagSet = ClipTags(c);
        for (int32 i = 1; i < n && cursor + i < velocities.Count(); i++)
        {
            const Float3 dv = velocities[cursor + i] - velocities[cursor + i - 1];
            const float a = Float2(dv.X, dv.Z).Length() / dt;
            for (const String& tag : tagSet)
            {
                int32 found = -1;
                for (int32 k = 0; k < tagNames.Count(); k++)
                {
                    if (tagNames[k] == tag)
                    {
                        found = k;
                        break;
                    }
                }
                if (found < 0)
                {
                    tagNames.Add(tag);
                    tagPeak.Add(a);
                    tagSum.Add((double)a);
                    tagNum.Add(1);
                }
                else
                {
                    if (a > tagPeak[found])
                        tagPeak[found] = a;
                    tagSum[found] += (double)a;
                    tagNum[found]++;
                }
            }
        }
        cursor += n;
    }
    char message[1024];
    int32 offset = snprintf(message, sizeof(message), "Motion matching accel audit (u/s^2 by tag): ");
    for (int32 k = 0; k < tagNames.Count() && offset < 900; k++)
    {
        const StringAnsi nameAnsi(tagNames[k]);
        const double avg = tagSum[k] / (double)MaxI(tagNum[k], 1);
        offset += snprintf(message + offset, sizeof(message) - (size_t)offset, "%s%s peak=%.0f avg=%.0f",
            k > 0 ? "; " : "", nameAnsi.GetText(), (double)tagPeak[k], avg);
    }
    LOG(Info, "{}", String(message));
}

void MotionMatchingRuntimePolicy::WriteClipAuditFile()
{
    if (_clipTravel.Count() == 0 || _clipNet.Count() == 0 || _clipTurnAngle.Count() == 0)
        return;
    if (_cacheFolder.IsEmpty())
        return;
    MotionMatchingDatabase* db = _database;
    if (db == nullptr)
        return;
    const Array<float>& lengths = db->GetClipLengths();
    StringBuilder sb;
    char line[512];
    for (int32 c = 0; c < _clipTravel.Count(); c++)
    {
        const String name = GetClipName(c);
        const float len = lengths.Count() > c ? lengths[c] : 0.0f;
        const float spd = len > 0.01f ? _clipTravel[c] / len : 0.0f;
        bool loop = false;
        bool turnLike = false;
        // Audit file tag column is '+'-joined (tools split it back).
        const String tag = MotionMatchingDatabase::JoinTags(ClipTags(c));
        loop = db->GetClipLoop(c);
        turnLike = db->GetClipTurnLike(c);
        const StringAnsi nameAnsi(name);
        const StringAnsi tagAnsi(tag);
        char fTravel[32], fNet[32], fYaw[32], fLen[32], fSpd[32];
        FormatFloatText(_clipTravel[c], fTravel, sizeof(fTravel));
        FormatFloatText(_clipNet[c], fNet, sizeof(fNet));
        FormatFloatText(_clipTurnAngle[c], fYaw, sizeof(fYaw));
        FormatFloatText(len, fLen, sizeof(fLen));
        FormatFloatText(spd, fSpd, sizeof(fSpd));
        snprintf(line, sizeof(line), "%d|%s|%s|%s|%s|%s|%s|%s|%s|%s\n",
            c, nameAnsi.GetText(), tagAnsi.GetText(),
            loop ? "loop" : "once", turnLike ? "turn" : "-",
            fTravel, fNet, fYaw, fLen, fSpd);
        sb.Append(line);
    }
    String text;
    sb.ToString(text);
    const StringAnsi folderAnsi(_cacheFolder);
    char path[1024];
    snprintf(path, sizeof(path), "%s/MMClipAudit.txt", folderAnsi.GetText());
    const StringAnsi bodyAnsi(text);
    FILE* file = fopen(path, "wb");
    if (file != nullptr)
    {
        fwrite(bodyAnsi.GetText(), 1, (size_t)bodyAnsi.Length(), file);
        fclose(file);
    }
}

void MotionMatchingRuntimePolicy::ResetOnTeleport()
{
    // Phase 3: pose history + playback SURVIVE teleports. Bone poses are
    // root-relative (recomputed fresh against the current root every
    // tick), so a teleport changes neither positions nor velocities Ã¢â‚¬â€
    // zeroing them poisoned the first query (pose cost in the thousands
    // on 44 dims) and stopping playback discarded continuity for
    // nothing. Only trajectory (world-frame) and chooser state reset;
    // the first post-teleport query still adopts tags immediately and
    // searches loops only (reseed rule below).
    _poseValid = false;
    _latestPose.Clear();
    _usedSearchSnapshot = false;
    _searchSnapshotLength = 0;
    _queryTimer = 0.0f;
    _timeSinceSwitch = 999.0f;
    _timeSinceInit = 0.0f;
    // _currentClip + playback survive (see above): continuity needs no
    // re-pick after a pure position jump.
    _hasLastActorPos = false;
    _chooserReseed = true;
    ActiveTag = String::Empty;
    _prevDesVel = Float3::Zero;
    _prevDesSpeed = 0.0f;
    _oneshotUntil = -99.0f;
    _reselectClip = -1;
    _reselectStartTime = -99.0f;
    _lastScanTotal = MAX_float;
    _lastFullScanTime = -99.0f;
    _lastFullScanFrame = -1;
    _consecutiveContinuationMisses = 0;
    _fastPathFrames = 0;
    _hasLastScanSignature = false;
    _fastPathReseed = true;
    ForcedReason = String::Empty;
    _hasSampleStarts = false;
    _tickTrajectory.Clear();
    FastPathHits = 0;
    LastFastPathAge = 0.0f;
    _wasAirborne = false;
    if (_playback != nullptr)
        _playback->SetPlayRate(1.0f);
    LastSwitchReason = String(TEXT("teleport-reset"));
    // Playback keeps playing (see above): same clip, same time, root
    // stripped Ã¢â‚¬â€ the picture never jumps. Only timers/locks release.
}

void MotionMatchingRuntimePolicy::CheckSchemaBinding()
{
    if (_bindingWarned || _database == nullptr || !_database->IsBaked())
        return;
    _bindingWarned = true;
    // Scene weights are live tuning (allowed to differ); skeleton names,
    // axes, sample rate and trajectory times must match the baked snapshot
    // or the query frame silently disagrees with the database.
    const PoseSearchSchema& live = _liveSchema;
    const PoseSearchSchema& baked = _database->GetSchema();
    const bool mismatch =
        live.Skeleton.Root != baked.Skeleton.Root ||
        live.Skeleton.Pelvis != baked.Skeleton.Pelvis ||
        live.Skeleton.LeftFoot != baked.Skeleton.LeftFoot ||
        live.Skeleton.RightFoot != baked.Skeleton.RightFoot ||
        live.SampleRate != baked.SampleRate ||
        !AxesNear(live.RootForwardAxis, baked.RootForwardAxis) ||
        !AxesNear(live.RootUpAxis, baked.RootUpAxis) ||
        !TimesEqual(live.TrajectorySampleTimes, baked.TrajectorySampleTimes);
    if (mismatch)
        LOG(Warning, "Motion matching schema differs from the baked database snapshot (skeleton/axes/rate/times). Query uses baked axes; rebake to reconcile. Scene weights remain live.");
}

bool MotionMatchingRuntimePolicy::AxesNear(const Float3& a, const Float3& b)
{
    const Float3 na = Float3::Normalize(a);
    const Float3 nb = Float3::Normalize(b);
    return (na - nb).Length() <= 0.001f;
}

bool MotionMatchingRuntimePolicy::TimesEqual(const Array<float>& a, const Array<float>& b)
{
    if (a.Count() != b.Count())
        return false;
    for (int32 i = 0; i < a.Count(); i++)
    {
        if (a[i] != b[i])
            return false;
    }
    return true;
}

String MotionMatchingRuntimePolicy::GetClipName(int32 clipIndex)
{
    Animation* anim = (_database != nullptr) ? _database->GetClipAnimation(clipIndex) : nullptr;
    if (anim != nullptr)
    {
        const StringAnsi pathAnsi(anim->GetPath());
        const char* path = pathAnsi.GetText();
        const int32 length = pathAnsi.Length();
        int32 start = 0;
        int32 dot = -1;
        for (int32 i = 0; i < length; i++)
        {
            if (path[i] == '/' || path[i] == '\\')
                start = i + 1;
            else if (path[i] == '.')
                dot = i;
        }
        const int32 end = (dot > start) ? dot : length;
        if (end > start)
            return String(path + start, end - start);
    }
    char fallback[32];
    snprintf(fallback, sizeof(fallback), "#%d", clipIndex);
    return String(fallback);
}

const Array<String>& MotionMatchingRuntimePolicy::ClipTags(int32 clipIndex)
{
    if (_clipTags.Count() > 0 && clipIndex >= 0 && clipIndex < _clipTags.Count())
        return _clipTags[clipIndex];
    if (_database != nullptr)
        return _database->GetClipTags(clipIndex);
    static const Array<String> empty;
    return empty;
}

String MotionMatchingRuntimePolicy::ClipTag(int32 clipIndex)
{
    const Array<String>& set = ClipTags(clipIndex);
    return set.HasItems() ? set[0] : String::Empty;
}

void MotionMatchingRuntimePolicy::WarnOnce(const String& message)
{
    if (_warnedOnce)
        return;
    _warnedOnce = true;
    const StringAnsi ansi(message);
    LOG(Warning, "{}", message);
}

float MotionMatchingRuntimePolicy::SelectPlayRate()
{
    return 1.0f;
}

void MotionMatchingRuntimePolicy::Tick(
    float deltaTime,
    const MotionMatchingPlayerSnapshot& state,
    const Array<TrajectoryPoint>& trajectoryPoints,
    const Matrix& actorWorld,
    const Array<float>& trajectorySampleTimes,
    const MotionMatchingPolicyTuning& tuning)
{
    if (!_samplingReady || _database == nullptr || _body == nullptr || _playback == nullptr || _trace == nullptr)
        return;

    _snapshot = state;
    _tuning = tuning;
    _trajectoryPoints = trajectoryPoints;
    _sampleTimes = trajectorySampleTimes;

    const float frameDt = MaxF(deltaTime, 0.000001f);
    _timeSinceSwitch += frameDt;
    _timeSinceInit += frameDt;
    _policyFrame++;

    // Teleport/discontinuity guard: an actor jump without Reset call would
    // otherwise leave stale previous positions and a locked continuity
    // winner. Threshold is far above any legitimate per-frame travel.
    const Float3 actorPos(actorWorld.M41, actorWorld.M42, actorWorld.M43);
    if (_hasLastActorPos && (actorPos - _lastActorPos).LengthSquared() > TeleportDistance * TeleportDistance)
    {
        ResetOnTeleport();
        // Re-accumulate this frame after the reset so warmup restarts cleanly.
        _timeSinceInit = frameDt;
        _timeSinceSwitch = 999.0f;
    }
    _lastActorPos = actorPos;
    _hasLastActorPos = true;

    CheckSchemaBinding();

    // Sample the live pose EVERY tick. Velocity = (pos - prev) / frameDt,
    // so the time basis always matches the sampling rate. Seeding happens
    // only after a successful sample (zero-velocity first frame).
    // PLAN Ã‚Â§5.3 source: prefer the playback's pre-layer/pre-IK search
    // snapshot over the displayed pose, so the layer and foot IK never feed
    // back into the next frame's candidate costs. Legacy displayed-pose
    // sampling stays as the fallback (no snapshot before the first
    // playback tick). A source switch reseeds velocities (one
    // zero-velocity frame) because the two poses differ by layer/IK.
    bool justSeeded = false;
    const Array<Matrix> searchLocals = _playback->GetSearchLocals();
    const bool haveSnapshot = searchLocals.Count() > 0;
    if (haveSnapshot != _usedSearchSnapshot ||
        (haveSnapshot && searchLocals.Count() != _searchSnapshotLength))
    {
        _hasPrevPositions = false;
        _usedSearchSnapshot = haveSnapshot;
        _searchSnapshotLength = haveSnapshot ? searchLocals.Count() : 0;
    }
    const float sampleDt = _hasPrevPositions ? frameDt : 0.0f;
    bool sampled = false;
    Array<float> pose;
    Array<float> prev;
    Matrix rootWorld;
    String poseError;
    if (haveSnapshot)
    {
        sampled = MotionMatchingQuery::SampleSearchPose(_body, searchLocals, _nodes, sampleDt,
            _prevPositions, _database->GetSchema(), pose, prev, rootWorld, poseError);
    }
    else
    {
        sampled = MotionMatchingQuery::SampleLivePose(_body, _nodes, sampleDt, _prevPositions,
            pose, prev, rootWorld, poseError);
    }
    if (sampled)
    {
        if (!_hasPrevPositions)
        {
            _hasPrevPositions = true;
            justSeeded = true;
        }
        _prevPositions = prev;
        _latestPose = pose;
        _latestRootWorld = rootWorld;
        _poseValid = true;
    }
    else
    {
        // Keep the old previous positions and try again next tick; never
        // seed success on failure and never feed a stale pose to search.
        _poseValid = false;
        if (!_warnedOnce)
        {
            _warnedOnce = true;
            const StringAnsi errorAnsi(poseError);
            char message[512];
            snprintf(message, sizeof(message), "Motion matching query skipped: %s", errorAnsi.GetText());
            LOG(Warning, "{}", String(message));
        }
    }

    // Phase 3: intent signals computed every tick so the fast-path below
    // yields to fresh input on the same tick (a lagging body otherwise
    // holds forever: pose matches old motion while intent already left).
    UpdateIntentSignals();
    // Phase 3: trajectory is built EVERY tick (cheap), not just on query
    // ticks, so the continuing fast-path can score each frame. Failures
    // null the cache; RunQuery treats null like today's trajError path.
    _tickTrajectory.Clear();
    if (_poseValid && trajectoryPoints.Count() == 6)
    {
        for (int32 i = 0; i < 6; i++)
        {
            _packedTrajectory[i * 6] = trajectoryPoints[i].Position.X;
            _packedTrajectory[i * 6 + 1] = trajectoryPoints[i].Position.Y;
            _packedTrajectory[i * 6 + 2] = trajectoryPoints[i].Position.Z;
            _packedTrajectory[i * 6 + 3] = trajectoryPoints[i].Facing.X;
            _packedTrajectory[i * 6 + 4] = trajectoryPoints[i].Facing.Y;
            _packedTrajectory[i * 6 + 5] = trajectoryPoints[i].Facing.Z;
        }
        Array<float> tickTraj;
        String trajError;
        if (!MotionMatchingQuery::BuildLiveTrajectory(_packedTrajectory, actorWorld, _latestRootWorld,
                _database->GetSchema(), tickTraj, trajError))
            _tickTrajectory.Clear();
        else
            _tickTrajectory = tickTraj;
    }


    _queryTimer += frameDt;
    const bool warmed = !justSeeded && _timeSinceInit >= WarmupTime;
    // Per-frame search: when enabled, every warmed tick may scan (the
    // budget/stride caps the cost, intent changes scan unstrided). The
    // cadence path below stays for SearchEveryFrame=false.
    const bool queryDue = warmed && (_tuning.SearchEveryFrame ||
        _queryTimer >= MaxF(_tuning.QueryInterval, 0.01f));
    // P2: pure search dispatch (machine holds deleted).
    if (queryDue && warmed)
    {
        _queryTimer = 0.0f;
        // P2: pure search dispatch (machine holds deleted). Cadence ticks
        // may run a full scan on continuation miss; off-cadence ticks below
        // only try the cheap continuation.
        _fastPathFrames++;
        if (!TryFastPath(frameDt))
            RunQuery(trajectoryPoints, actorWorld, frameDt);
    }
    else if (queryDue && (_timeSinceInit < WarmupTime || justSeeded))
    {
        // Warmup/seeding ticks consume the timer without searching so the
        // first real query sees settled pose velocity and trajectory past.
        _queryTimer = 0.0f;
    }
    else if (warmed && _poseValid)
    {
        // Ã‚Â§7.1/Ã‚Â§18.8 off-cadence tick: cheap continuation only. A miss is
        // recorded (and forces the full scan at the next cadence tick), but
        // no scan runs here Ã¢â‚¬â€ a jittering frame time can no longer turn
        // every off-cadence frame into a full search (scan storm).
        TryFastPath(frameDt);
    }


    _playback->SetPlayRate(SelectPlayRate());
    // Ã‚Â§18.9: stamp this tick's search decision on playback before it
    // asserts intra-tick ordering (sample <= search <= save < layer < IK).
    NotifySearch();
    _playback->Tick(frameDt, _body, state.IsGrounded);
    // Keep the search stamp mirror in lockstep with playback's tick
    // counter: it advances only on a successful playback Tick, so early
    // outs never desync the stamp.
    if (_playback->SampleFrameId != _lastObservedSampleFrameId)
    {
        _lastObservedSampleFrameId = _playback->SampleFrameId;
        if (_playback->SampleFrameId >= 0)
            _nextSearchFrameId = _playback->SampleFrameId + 1;
    }

    // A finished one-shot releases the continuity lock so the next
    // query may leave immediately instead of holding the last frame.
    if (_playback->IsAtEnd)
        _timeSinceSwitch = 999.0f;
}

// P2: onset band ceilings are the ex-default provider values (5/200/490).
// The provider owns the live ceilings for its scope; the core keeps fixed
// bands so the onset one-shot filter has no gameplay tuning.
int32 MotionMatchingRuntimePolicy::BandFromSpeed(float speed) const
{
    if (speed <= 5.0f)
        return 0;
    if (speed <= 200.0f)
        return 1;
    if (speed <= 490.0f)
        return 2;
    return 3;
}

void MotionMatchingRuntimePolicy::RecordPolicyEvent(const char* kind, int32 clipIndex, const String& reason)
{
    const MotionMatchingPlayerSnapshot& state = _snapshot;
    const float speed = Float2(state.CurrentVelocity.X, state.CurrentVelocity.Z).Length();
    MotionMatchingTraceEntry entry;
    entry.Kind = String(kind);
    entry.Time = Time::GetGameTime();
    entry.State = PolicyState;
    entry.Tags = ActiveTag;
    entry.Speed = speed;
    entry.YawRate = AbsF(state.AngularVelocity.Y);
    entry.Grounded = state.IsGrounded;
    entry.Sprinting = state.IsSprinting;
    entry.DesiredSpeed = Float2(state.DesiredVelocity.X, state.DesiredVelocity.Z).Length();
    entry.Valid = false;
    entry.ClipIndex = clipIndex;
    entry.Reason = reason;
    entry.Chooser = ChooserStatusText;
    entry.QueryMs = 0.0f;
    Array<float> empty;
    _trace->Record(entry, empty, empty);
}

void MotionMatchingRuntimePolicy::UpdateIntentSignals()
{
    const MotionMatchingPlayerSnapshot& state = _snapshot;
    const bool airborne = !state.IsGrounded;
    const float planarSpeed = Float2(state.CurrentVelocity.X, state.CurrentVelocity.Z).Length();
    const float yawRate = AbsF(state.AngularVelocity.Y);
    const float desSpeed = Float2(state.DesiredVelocity.X, state.DesiredVelocity.Z).Length();
    if (!airborne && yawRate < 0.5f &&
        desSpeed - _prevDesSpeed >= 150.0f)
        _oneshotUntil = Time::GetGameTime() + 0.35f;
    _prevDesSpeed = desSpeed;
    _tickOneshotWindow = !airborne && Time::GetGameTime() < _oneshotUntil;
    _tickFairFight = false;
    if (!airborne && (planarSpeed > 100.0f || desSpeed > 100.0f))
    {
        const Float3 dDes = state.DesiredVelocity - _prevDesVel;
        const float intentDelta = Float2(dDes.X, dDes.Z).Length();
        // Leaky accumulator: spring slew spreads a flip over ~10 ticks so
        // no single tick exceeds the 100 gate (measured: 1 fair-fight scan
        // per 90-frame flip window). Sustained re-aim (~20/tick) trips the
        // accumulator in 2-3 ticks and latches unbiased scans through the
        // slew; steady-motion noise (~a few u/s) never reaches it.
        _intentAccum = _intentAccum * 0.7f + intentDelta;
        if (intentDelta > 100.0f || _intentAccum > _tuning.FairFightAccum)
        {
            _tickFairFight = true;
            _fairFightUntil = Time::GetGameTime() + _tuning.FairFightWindow;
        }
        else if (Time::GetGameTime() < _fairFightUntil)
        {
            _tickFairFight = true;
        }
        else
        {
            _intentAccum = 0.0f;
        }
    }
    else
    {
        _intentAccum = 0.0f;
    }
    _prevDesVel = state.DesiredVelocity;
}

void MotionMatchingRuntimePolicy::RunQuery(const Array<TrajectoryPoint>& trajectoryPoints, const Matrix& actorWorld, float frameDt)
{
    const auto watchStart = std::chrono::high_resolution_clock::now();
    const MotionMatchingPlayerSnapshot& state = _snapshot;

    if (!_poseValid || _latestPose.Count() == 0)
        return;

    // Ã‚Â§18.9: this tick's search decision is a real full scan.
    NotifySearch();

    const bool airborne = !state.IsGrounded;
    PolicyState = airborne ? String(TEXT("Airborne")) : String(TEXT("Locomotion"));
    const float planarSpeed = Float2(state.CurrentVelocity.X, state.CurrentVelocity.Z).Length();
    const float yawRate = AbsF(state.AngularVelocity.Y);
    const float desSpeed = Float2(state.DesiredVelocity.X, state.DesiredVelocity.Z).Length();
    // Onset/decel + fair-fight now arrive per-tick from UpdateIntentSignals
    // (Phase 3: the fast-path needs them on non-scan ticks too).
    const bool oneshotWindow = _tickOneshotWindow;
    const bool fairFight = _tickFairFight;
    // Forced transitions bypass continuity ONLY on the grounded/airborne
    // edge (take-off, landing). Continuously forcing while airborne made
    // every midair query jump freely and landing frames start midair poses.
    const bool edge = airborne != _wasAirborne;
    bool forced = edge;
    _wasAirborne = airborne;

    // Reseed query (M3): the first query after a teleport/discontinuity
    // runs on a zero-velocity reseed pose that cannot honestly match a
    // transition (all bone velocities read zero). Loops only Ã¢â‚¬â€ otherwise
    // a planted one-shot (pivot) wins the poisoned query and the playing
    // pose then looks like itself forever.
    const bool reseeded = _chooserReseed;
    // P2: provider-decided scope only (native legacy chooser deleted). The
    // provider owns hysteresis/bands/roles game-side; empty allowed is an
    // explicit no-match: hold current, never fall back to any pool.
    _chooserReseed = false;
    Array<String> tags = MotionMatchingDatabase::SplitTagSet(_snapshot.AllowedTagSet);
    ActiveTag = MotionMatchingDatabase::JoinTags(tags);
    ChooserStatusText = String(TEXT("ext:")) + ActiveTag;
    if (tags.Count() == 0)
    {
        ActiveTag = String::Empty;
        ChooserStatusText = String(TEXT("ext:empty (no-match)"));
        LastSwitchReason = String(TEXT("no-result:empty-scope"));
        return;
    }

    if (trajectoryPoints.Count() != 6)
    {
        WarnOnce(String(TEXT("Motion matching query skipped: trajectory has 6 points.")));
        return;
    }

    // Phase 3: trajectory is built every tick in Tick (see _tickTrajectory);
    // a null cache means the build failed and matches today's trajError path.
    const Array<float>& trajectory = _tickTrajectory;
    if (trajectory.Count() == 0)
    {
        WarnOnce(String(TEXT("Motion matching query skipped: trajectory build failed this tick.")));
        return;
    }

    // Weights come from the live tuning, not the baked snapshot:
    // weights are never baked into features, so tuning them here applies
    // immediately with no rebake (edit them in the inspector, even mid-play).
    const float currentTime = (_currentClip >= 0 && _playback != nullptr) ? _playback->CurrentTime : -1.0f;
    // PLAN Ã‚Â§6.1 / Ã‚Â§18.10: ExpectedTime = playback time + THIS frame's
    // upcoming advance (frameDt * PlayRate). The decision takes effect in
    // this frame's _playback.Tick below, so the continuity anchor is the
    // post-advance position Ã¢â‚¬â€ never CurrentTime + the full inter-query
    // interval (that double-counts: CurrentTime already contains every
    // intermediate frame's advance). Matches TryFastPath's prediction.
    const float playRate = _playback != nullptr ? _playback->GetPlayRate() : 1.0f;
    float expectedTime = -1.0f;
    if (currentTime >= 0.0f)
    {
        const float advance = MaxF(frameDt, 0.000001f) * playRate;
        expectedTime = currentTime + advance;
        if (_database != nullptr && _currentClip >= 0 && _database->GetClipLoop(_currentClip))
        {
            const float len = _playback != nullptr ? _playback->GetCurrentClipLength() : 0.0f;
            if (len > 0.0f && expectedTime >= len)
                expectedTime = fmodf(expectedTime, len);
        }
    }
    // Rest-calm = slow + grounded + no yaw: one-shot end stances lose to
    // loops structurally here (break/stop/turn tails cannot re-win).
    // Measured rest gaps run 0.2-2.1, so the 0.5 preference below is only
    // a tie-break; the structural exit is end-triggered LoopOnly (next).
    // P2: rest/yaw gates use the ex-default provider ceilings (5 u/s, 0.5
    // rad/s). The provider owns the live values for its scope decision;
    // the core keeps fixed gates so search has no gameplay tuning.
    const bool restCalm = !airborne && planarSpeed <= 5.0f &&
        yawRate < 0.5f;
    // End-triggered loop recovery: playback frozen on a finished one-shot
    // end frame searches loop clips ONLY (same chooser tags). The ended
    // one-shot cannot re-win no matter how exactly the pose matches, so
    // breaks/stops/turns always resolve into a loop at rest. Gated by
    // restCalm: mid-motion or mid-yaw ends keep normal search (a turn end
    // while still rotating must be allowed another turn, a stop end while
    // sliding must find walk loops).
    const bool endedRest = _playback != nullptr && _playback->IsAtEnd && restCalm;
    // Moving twin of the above: a one-shot that ends while steadily
    // moving (no yaw) also recovers into loops ONLY. Without this, a
    // transition that fairly won on entry (e.g. run entry after a turn)
    // re-wins forever Ã¢â‚¬â€ the playing pose looks like itself, so the loop
    // never gets a foothold. Turn ends with live yaw are excluded (B2b
    // replay owns them) and airborne ends keep normal search.
    const bool endedMoving = _playback != nullptr && _playback->IsAtEnd && !airborne && !restCalm &&
        planarSpeed > 5.0f && yawRate < 0.5f;
    const bool endedRecovery = endedRest || endedMoving;
    // Settle-to-loop: a one-shot still playing while steadily moving (no
    // yaw) yields to loops early instead of riding a seconds-long
    // transition to its end. The playing pose looks like the transition
    // itself, so a fair search re-wins it forever and the loop never gets
    // a foothold (observed: holding W played Transition_Run_to_Walk while
    // Run_Loop_F at 493u/s sat out). Gated past the settle delay so short
    // starts still play, and turns/airborne keep their own paths (B2b
    // replay, jump). LoopOnly removes the one-shot from candidates, so no
    // margin fight is needed Ã¢â‚¬â€ the best loop simply wins.
    // Live value: SettleToLoopDelay (default 0.3s).
    // Decel guard: settling requires LIVE intent (des beyond idle). A stop
    // one-shot coasting down (des ~ 0) must play through to its end stance
    // and hand off via end-recovery Ã¢â‚¬â€ cutting it to a loop mid-decel is
    // what produced the stop->walk-loop detour (measured 32.05).
    const bool currentOneShot = _currentClip >= 0 && _playback != nullptr &&
        _database != nullptr && !_database->GetClipLoop(_currentClip);
    // No turn-like exclusion: calm yaw is already required above, and the
    // turn machinery (hold/replay) only owns live-yaw cases. A pivot held
    // while running straight otherwise deadlocks (observed M3: pivot wins
    // a poisoned reseed query, then re-wins forever on self-pose).
    const bool settleToLoop = currentOneShot && !airborne && !restCalm &&
        planarSpeed > 5.0f && desSpeed > 5.0f &&
        yawRate < 0.5f &&
        _timeSinceSwitch > _tuning.SettleToLoopDelay;
    const float minSwitch = _tuning.MinSwitchTime;
    MotionMatchingSearchSettings settings;
    settings.PoseWeight = _tuning.PoseWeight;
    settings.TrajectoryWeight = _tuning.TrajectoryWeight;
    settings.ContinuityWeight = _tuning.ContinuityWeight;
    settings.MinSwitchTime = minSwitch;
    settings.CurrentTime = currentTime;
    settings.SameClipExclusionWindow = _tuning.SameClipExclusion;
    settings.ExpectedTime = expectedTime;
    settings.ContinuingBias = fairFight ? _tuning.ContinuingBias * 0.5f : _tuning.ContinuingBias;
    settings.ContinuingWindow = _tuning.ContinuingWindow;
    // foot-ik branch: fair-fight HALVES (not zeroes) bias/margin.
    // Zeroed unbiased scans thrashed mid-slew (transitions/twins won,
    // shares dropped 12/12-capable runs to 10/12). Half keeps a
    // preference for the incumbent while letting a committed new
    // intent win earlier.
    settings.SwitchMargin = fairFight ? _tuning.SwitchMargin * 0.5f : _tuning.SwitchMargin;
    settings.LoopPreference = restCalm ? _tuning.RestLoopPreference : 0.0f;
    // Wave 1 bans: reselect lives on a wall-clock lifetime owned here
    // (the search is stateless); twin + tail are always-on structural
    // scope cuts, replayed bit-exactly from the trace entry.
    {
        const bool banLive = EnableReselectBan && _reselectClip >= 0 &&
            Time::GetGameTime() - _reselectStartTime < ReselectHistoryDuration;
        settings.ReselectClip = banLive ? _reselectClip : -1;
        settings.ReselectSampleTime = _reselectSampleTime;
        settings.ReselectBanWindow = banLive ? ReselectSampleWindow : 0.0f;
    }
    settings.TwinBanWindow = SameSegmentTwinBan;
    settings.LoopTailExclusion = EnableLoopTailExclusion ? NonLoopTailExclusion : 0.0f;
    // S1 Ã‚Â§7.2/Ã‚Â§18.6 structural validity: non-loop candidates whose
    // future horizon passes the clip end baked a clamped fake-stopped
    // future Ã¢â‚¬â€ never rank them (measured: start-tail samples winning
    // releases on exactly such fakes). C# bools default false, so this
    // must be set explicitly; the C++ default never reaches runtime.
    settings.ExcludeNonLoopTail = true;
    // Per-frame budgeted search: steady queries score a rotating
    // subset (FrameSearchBudget samples), intent-change queries
    // (fair fight / onset window) always scan unstrided Ã¢â‚¬â€ transitions
    // get exact rankings while steady motion stays cheap. QueryFrame
    // rotates the stride cursor (policy frame, reset per session).
    settings.MaxCandidates = (fairFight || oneshotWindow) ? 0 : MaxI(_tuning.FrameSearchBudget, 0);
    settings.QueryFrame = _policyFrame;
    // Phase 1 indexed search (default off; brute force reference).
    // Phase 2 audit file-harness also forces both flags (no default
    // or scene change needed to measure divergence).
    settings.UseIndexedSearch = _tuning.UseIndexedSearch || _tuning.IndexedVerifyAuto;
    settings.TopKCandidates = MaxI(1, _tuning.TopKCandidates);
    settings.VerifyIndexedSearch = _tuning.VerifyIndexedSearch || _tuning.IndexedVerifyAuto;
    settings.QueryYawRate = yawRate;
    // Foot-lock prior weights: playback pin weights (0..1, clamped).
    // Recorded in the trace below, so replay stays bit-exact.
    // P2: no plugin lock pins exist (solver deleted); the search prior
    // stays structurally (settings + FootLockCost + selftest Phase K) but
    // never fires at runtime. Game-side IK owns its own contact state.
    settings.FootLockL = 0.0f;
    settings.FootLockR = 0.0f;
    settings.FirmLockL = false;
    settings.FirmLockR = false;
    // Fresh input beats stale end-state: an onset window (des jumped)
    // wants transitions, even if playback sits on an ended one-shot
    // that would otherwise force loop-only recovery. Recovery still
    // owns genuinely stuck ends (no fresh input, no window). Reseed
    // (above) beats both: a zero-velocity pose cannot pick honestly.
    // Onset gait band: the window fires on speed increase, and the
    // one-shot pool is cut to the intended band Ã¢â‚¬â€ at the onset instant
    // the features carry almost no gait information, so walk/run
    // starts would otherwise coin-flip (measured: Run_Start winning a
    // ctrl+D walk onset). Loops are never band-cut.
    settings.LoopFilter = reseeded ? MotionLoopFilter::LoopOnly :
        (oneshotWindow ? MotionLoopFilter::OneShotOnly :
        ((endedRecovery || settleToLoop) ? MotionLoopFilter::LoopOnly : MotionLoopFilter::Any));
    settings.OneShotBand = oneshotWindow ? BandFromSpeed(desSpeed) : 0;
    // Controller-decided exclusions ride the snapshot (plain-String
    // transport, split here). Independent of the include scope above.
    if (!_snapshot.ExcludedTagSet.IsEmpty())
        settings.ExcludeTags = MotionMatchingDatabase::SplitTagSet(_snapshot.ExcludedTagSet);
    settings.CollectClassDiagnostics = _tuning.EffectiveTelemetry;

    // Ã‚Â§18.8: capture the query signature this full scan anchors the fast
    // path against (clip/time cannot change until Play below).
    const FastPathSignature scanSignature = CaptureFastPathSignature();

    // Forced edge transitions (take-off/landing) bypass continuity entirely:
    // no current clip, no lock, so the search is free to jump anywhere.
    const int32 continuityClip = forced ? -1 : _currentClip;
    const float sinceSwitch = forced ? 999.0f : _timeSinceSwitch;

    MotionMatchingResult result;
    String error;
    MotionMatchingSearch::FindBest(_database, _latestPose, trajectory, settings,
        continuityClip, sinceSwitch, tags, result, error);

    // End recovery: a finished one-shot with temporal exclusion active can
    // exclude every same-clip candidate and return nothing (frozen last
    // frame forever). Retry once without exclusion, demanding LOOP clips
    // structurally (LoopOnly) instead of naming a rest gait: the finished
    // one-shot cannot re-win on exact pose match, whatever tag it has.
    bool recovered = false;
    if (!result.IsValid && _playback != nullptr && _playback->IsAtEnd)
    {
        MotionMatchingSearchSettings retry = settings;
        retry.CurrentTime = -1.0f;
        retry.SameClipExclusionWindow = 0.0f;
        // Recovery must always have somewhere to go: drop the reselect
        // ban (twin is already gated off by continuity -1, tail is moot
        // under LoopOnly).
        retry.ReselectClip = -1;
        retry.ReselectBanWindow = 0.0f;
        retry.LoopFilter = MotionLoopFilter::LoopOnly;
        MotionMatchingResult retryResult;
        String retryError;
        MotionMatchingSearch::FindBest(_database, _latestPose, trajectory, retry,
            -1, 999.0f, tags, retryResult, retryError);
        if (retryResult.IsValid)
        {
            result = retryResult;
            error = retryError;
            forced = true;
            recovered = true;
        }
    }

    const auto watchEnd = std::chrono::high_resolution_clock::now();
    LastQueryMs = (float)std::chrono::duration<double, std::milli>(watchEnd - watchStart).count();

    // Onset fallback: regimes without one-shot coverage (no back starts
    // exist) retry with the full pool instead of freezing on nothing.
    if (!result.IsValid && oneshotWindow)
    {
        settings.LoopFilter = MotionLoopFilter::Any;
        MotionMatchingResult anyResult;
        String anyError;
        MotionMatchingSearch::FindBest(_database, _latestPose, trajectory, settings,
            continuityClip, sinceSwitch, tags, anyResult, anyError);
        if (anyResult.IsValid)
        {
            result = anyResult;
            error = anyError;
        }
    }

    // Ã‚Â§18.8: a completed full scan re-anchors the fast-path baseline and
    // clears reseed. An invalid result invalidates the cost baseline so
    // the next continuation check misses instead of holding stale data.
    _lastFullScanTime = Time::GetGameTime();
    _lastFullScanFrame = _policyFrame;
    _fastPathFrames = 0;
    _consecutiveContinuationMisses = 0;
    _fastPathReseed = false;
    _lastScanSignature = scanSignature;
    _hasLastScanSignature = true;
    _lastScanTotal = result.IsValid ? result.Cost : MAX_float;

    if (error.IsEmpty() == false || !result.IsValid)
    {
        char message[512];
        const StringAnsi errorAnsi(error);
        snprintf(message, sizeof(message), "Motion matching search produced nothing: %s", errorAnsi.GetText());
        WarnOnce(String(message));
        LastSwitchReason = String(TEXT("no-result:hold"));
        RecordTrace(planarSpeed, yawRate, tags, trajectory, settings,
            continuityClip, sinceSwitch, recovered, false, result, String(TEXT("no-result:hold")));
        return;
    }

    WinnerSample = result.SampleIndex;
    WinnerClip = result.ClipIndex;
    WinnerClipName = GetClipName(result.ClipIndex);
    WinnerCost = result.Cost;
    SecondBestCost = result.SecondBestCost;
    WinnerPoseCost = result.PoseCost;
    WinnerTrajectoryCost = result.TrajectoryCost;
    WinnerContinuityCost = result.ContinuityCost;
    WinnerLoopCost = result.LoopCost;
    WinnerIsLoop = _database != nullptr && result.ClipIndex >= 0 &&
        _database->GetClipLoop(result.ClipIndex);
    LoopPreferenceApplied = settings.LoopPreference;
    BestLoopCost = result.BestLoopCost;
    BestLoopClip = result.BestLoopClip;
    BestLoopClipName = result.BestLoopClip >= 0 ? GetClipName(result.BestLoopClip) : String::Empty;
    BestOneShotCost = result.BestOneShotCost;
    BestOneShotClip = result.BestOneShotClip;
    BestOneShotClipName = result.BestOneShotClip >= 0 ? GetClipName(result.BestOneShotClip) : String::Empty;
    LoopScoredCount = result.LoopScoredCount;
    OneShotScoredCount = result.OneShotScoredCount;
    LoopTotalCount = result.LoopTotalCount;
    OneShotTotalCount = result.OneShotTotalCount;
    // PLAN Ã‚Â§18.10: Compute timeError against ExpectedTime for continuity/telemetry trace
    float timeError = 0.0f;
    if (settings.ExpectedTime >= 0.0f && result.ClipIndex == continuityClip)
    {
        float diff = result.SampleTime - settings.ExpectedTime;
        if (WinnerIsLoop)
        {
            const float len = _playback != nullptr ? _playback->GetCurrentClipLength() : 0.0f;
            if (len > 0.0f)
            {
                diff = fmodf(diff, len);
                if (diff < 0.0f)
                    diff += len;
                if (diff > len * 0.5f)
                    diff -= len;
            }
        }
        timeError = diff;
    }
    LastTimeError = timeError;
    LogTelemetryLine(planarSpeed, yawRate, tags, result,
        Float3(actorWorld.M41, actorWorld.M42, actorWorld.M43), timeError);

    const bool didSwitch = result.ClipIndex != _currentClip;
    // Cost fuse: one absurd-cost winner never overturns steady tracking.
    // Corrupted query poses (~1-in-2000 frame glitches, e.g. cost 600+
    // mid-walk) self-heal next query; switching on them cascades. Forced
    // edges (take-off/landing) and end recovery always go through, and
    // with no current clip there is nothing to hold.
    const bool fused = didSwitch && !forced && !recovered && _currentClip >= 0 &&
        result.Cost > _tuning.CostFuseThreshold && result.Cost < 1e29f;
    if (fused)
    {
        LastSwitchReason = String(TEXT("held:cost-fuse"));
    }
    else if (didSwitch)
    {
        // Arm the reselect ban on the pose we just left (single slot):
        // it cannot re-win for ReselectHistoryDuration. Captured before
        // the switch from live playback state.
        const int32 leftClip = _currentClip;
        const float leftTime = _playback != nullptr ? _playback->CurrentTime : -1.0f;
        _currentClip = result.ClipIndex;
        _timeSinceSwitch = 0.0f;
        SwitchCount++;
        if (leftClip >= 0 && leftTime >= 0.0f)
        {
            _reselectClip = leftClip;
            _reselectSampleTime = leftTime;
            _reselectStartTime = Time::GetGameTime();
        }
        else
        {
            _reselectClip = -1;
        }
        _playback->Play(result.ClipIndex, result.SampleTime, _playback->IsAtEnd);
        if (recovered)
            LastSwitchReason = String(TEXT("end-recovery:looponly-retry"));
        else if (forced)
            LastSwitchReason = String(TEXT("edge-snap:new-clip"));
        else
            LastSwitchReason = String(TEXT("switched:new-clip"));
    }
    else
    {
        // Same-clip winner: continuation is the default (keep playing).
        // Exclusion in the search already removed nearby frames, so any
        // same-clip winner here is a far phase jump worth seeking Ã¢â‚¬â€ but it
        // still obeys MinSwitchTime hysteresis to avoid reseeking every
        // query, and end recovery always seeks.
        const bool atEnd = _playback->IsAtEnd;
        const float jump = AbsF(result.SampleTime - _playback->CurrentTime);
        if (atEnd)
        {
            _timeSinceSwitch = 0.0f;
            SeekCount++;
            _playback->Play(result.ClipIndex, result.SampleTime, true);
            LastSwitchReason = String(TEXT("seek:end-recovery"));
        }
        else if (jump > _tuning.SameClipSeekThreshold && _timeSinceSwitch >= minSwitch)
        {
            _timeSinceSwitch = 0.0f;
            SeekCount++;
            _playback->Play(result.ClipIndex, result.SampleTime, true);
            LastSwitchReason = String(TEXT("seek:same-clip-far"));
        }
        else
        {
            LastSwitchReason = _timeSinceSwitch < _tuning.MinSwitchTime
                ? String(TEXT("held:continuity-lock")) : String(TEXT("continued"));
        }
    }
    // Anomaly watchdog (throttled): only a switched anomaly is a visible
    // pop Ã¢â‚¬â€ spikes the continuity lock absorbs are the system working.
    {
        const float now = Time::GetGameTime();
        if (didSwitch && !fused && result.Cost > 0.5f && result.Cost < 1e29f && now - _lastSpikeWarn > 5.0f)
        {
            _lastSpikeWarn = now;
            const StringAnsi clipAnsi(WinnerClipName);
            const StringAnsi tagAnsi(ActiveTag);
            char message[512];
            snprintf(message, sizeof(message),
                "Motion matching anomaly: cost spike %.1f (pose=%.1f traj=%.1f tags=%s clip=%s speed=%.0f).",
                (double)result.Cost, (double)result.PoseCost, (double)result.TrajectoryCost,
                tagAnsi.GetText(), clipAnsi.GetText(), (double)planarSpeed);
            LOG(Warning, "{}", String(message));
        }
    }
    RecordTrace(planarSpeed, yawRate, tags, trajectory, settings,
        continuityClip, sinceSwitch, recovered, true, result, LastSwitchReason);
}

bool MotionMatchingRuntimePolicy::TryFastPath(float frameDt)
{
    // Ã‚Â§18.9: evaluating the continuation is itself a search decision.
    NotifySearch();
    ForcedReason = String::Empty;

    // Fresh input beats a lagging body: onset windows and intent-vector
    // flips always scan (this is the turn/flip latency path). Without
    // this the hold is self-fulfilling (no switch -> body keeps old
    // motion -> continuing keeps matching).
    if (_tickOneshotWindow || _tickFairFight)
        return MissFastPath("miss:intent");
    // Reseed/teleport: the incumbent was measured against the old
    // baseline; the first continuation check after a reset must scan.
    if (_fastPathReseed)
        return MissFastPath("forced:reseed");
    // No baseline yet (init/teleport reset _lastScanTotal to MaxValue):
    // MaxValue as threshold would hold ANYTHING, freezing the pre-reset
    // clip forever (observed: whole phases stuck in idle). Force a scan.
    if (_lastScanTotal >= 1e29f)
        return MissFastPath("miss:no-baseline");
    if (_currentClip < 0 || _playback == nullptr || _database == nullptr ||
        !_poseValid || _latestPose.Count() == 0 ||
        _tickTrajectory.Count() == 0)
        return MissFastPath("miss:no-state");
    MotionMatchingDatabase* db = _database;
    if (!db->GetClipLoop(_currentClip))
        return MissFastPath("miss:non-loop");

    // Ã‚Â§18.8 query signature: compare against the signature stored at the
    // last full scan. A changed database identity (rebake/reload) also
    // invalidates the baseline.
    const FastPathSignature sig = CaptureFastPathSignature();
    if (_hasLastScanSignature && sig.DatabaseId != _lastScanSignature.DatabaseId)
        return MissFastPath("miss:db-changed");

    // Finite incumbent lifetime: never hold past the age/frame caps even
    // when the continuation cost still looks good. _fastPathFrames counts
    // query opportunities; the frame cap scales with the query rate so it
    // still means ~MaxFastPathAge under per-frame search (at 10Hz cadence
    // this reduces to the original constant 5).
    if (Time::GetGameTime() - _lastFullScanTime > MaxFastPathAge)
        return MissFastPath("forced:age");
    const int32 frameCap = MaxI(MaxFastPathFrames,
        (int32)ceilf(MaxFastPathAge / MaxF(frameDt, 1e-4f)));
    if (_fastPathFrames > frameCap)
        return MissFastPath("forced:age-frames");
    // A streak of continuation misses means the incumbent keeps looking
    // worse; stop waiting out the caps and re-scan.
    if (_consecutiveContinuationMisses >= 3)
        return MissFastPath("miss:miss-streak");
    if (_hasLastScanSignature && FastPathSignaturesDiffer(sig, _lastScanSignature))
        return MissFastPath("forced:signature");

    EnsureSampleStarts();
    if (!_hasSampleStarts)
        return MissFastPath("miss:no-offsets");

    const Array<int32>& counts = db->GetClipSampleCounts();
    if (counts.Count() == 0 || _currentClip >= counts.Count())
        return MissFastPath("miss:no-counts");
    const int32 count = counts[_currentClip];
    if (count <= 0)
        return MissFastPath("miss:empty-clip");
    const float rate = _tuning.SampleRate;
    if (rate <= 0.0f)
        return MissFastPath("miss:no-rate");
    // Loops only (gated above): wrap time into the cycle using playback's play rate.
    const float currentPlayRate = _playback != nullptr ? _playback->GetPlayRate() : 1.0f;
    const float effectiveDt = frameDt * currentPlayRate;
    const float t = _playback->CurrentTime + effectiveDt;
    const float len = (float)count / rate;
    if (len <= 0.0f)
        return MissFastPath("miss:no-length");
    float wrapped = fmodf(t, len);
    if (wrapped < 0.0f)
        wrapped += len;
    // Clip nearly at the loop seam: hand off with a real scan instead of
    // scoring a sample that is about to wrap.
    if (len - wrapped < effectiveDt * 1.5f)
        return MissFastPath("miss:clip-near-end");
    const int32 sample = _clipSampleStarts[_currentClip] + ((int32)roundf(wrapped * rate) % count);

    const float yawRate = AbsF(_snapshot.AngularVelocity.Y);
    float fast = 0.0f;
    String sampleError;
    if (!MotionMatchingSearch::SampleCost(db, _latestPose, _tickTrajectory, sample,
            _tuning.PoseWeight, _tuning.TrajectoryWeight,
            yawRate, true, fast, sampleError))
        return MissFastPath("miss:sample-cost");

    const float threshold = _lastScanTotal + _tuning.SwitchMargin + _tuning.ContinuingBias;
    if (!(fast <= threshold))
        return MissFastPath("miss:cost");

    // Hit: hold the incumbent and record its age (telemetry/trace).
    _consecutiveContinuationMisses = 0;
    FastPathHits++;
    LastFastPathAge = Time::GetGameTime() - _lastFullScanTime;
    LastSwitchReason = String(TEXT("fast:continuing-good"));
    {
        const float planarSpeed = Float2(_snapshot.CurrentVelocity.X, _snapshot.CurrentVelocity.Z).Length();
        MotionMatchingTraceEntry entry;
        entry.Kind = String(TEXT("search"));
        entry.Time = Time::GetGameTime();
        entry.State = PolicyState;
        entry.Tags = ActiveTag;
        entry.Speed = planarSpeed;
        entry.YawRate = yawRate;
        entry.Grounded = _snapshot.IsGrounded;
        entry.Sprinting = _snapshot.IsSprinting;
        entry.DesiredSpeed = Float2(_snapshot.DesiredVelocity.X, _snapshot.DesiredVelocity.Z).Length();
        entry.PoseWeight = _tuning.PoseWeight;
        entry.TrajectoryWeight = _tuning.TrajectoryWeight;
        entry.Valid = true;
        entry.ClipIndex = _currentClip;
        entry.SampleIndex = sample;
        entry.SampleTime = t;
        entry.Cost = fast;
        entry.PoseCost = -1.0f;
        entry.TrajectoryCost = -1.0f;
        entry.SecondBestCost = threshold;
        entry.Reason = String(TEXT("fast:continuing-good"));
        entry.Chooser = ChooserStatusText;
        entry.QueryMs = 0.0f;
        Array<float> empty;
        _trace->Record(entry, empty, empty);
    }
    return true;
}

bool MotionMatchingRuntimePolicy::MissFastPath(const char* reason)
{
    ForcedReason = String(reason);
    _consecutiveContinuationMisses++;
    return false;
}

int32 MotionMatchingRuntimePolicy::Quantize(float value, float step)
{
    if (step <= 0.0f)
        return 0;
    return (int32)roundf(value / step);
}

MotionMatchingRuntimePolicy::FastPathSignature MotionMatchingRuntimePolicy::CaptureFastPathSignature() const
{
    const float planarSpeed = Float2(_snapshot.CurrentVelocity.X, _snapshot.CurrentVelocity.Z).Length();
    const float yawRate = AbsF(_snapshot.AngularVelocity.Y);
    // P2: lock-machine bits removed with the plugin solver (no pins exist).
    int32 contacts = (_snapshot.IsGrounded ? 1 : 0) | (_snapshot.IsSprinting ? 2 : 0);
    int32 dbId = 0;
    if (_database != nullptr)
        dbId = (int32)(((uintptr_t)_database) & 0xFFFFFFFF) ^ (_database->GetClipCount() * 397);
    FastPathSignature sig;
    sig.Speed = Quantize(planarSpeed, 50.0f);
    sig.IntentX = Quantize(_snapshot.DesiredVelocity.X, 50.0f);
    sig.IntentZ = Quantize(_snapshot.DesiredVelocity.Z, 50.0f);
    sig.YawRate = Quantize(yawRate, 15.0f);
    sig.Contacts = contacts;
    sig.Clip = _currentClip;
    sig.ClipTime = Quantize(_playback != nullptr ? _playback->CurrentTime : 0.0f, 0.5f);
    sig.DatabaseId = dbId;
    return sig;
}

bool MotionMatchingRuntimePolicy::FastPathSignaturesDiffer(const FastPathSignature& a, const FastPathSignature& b)
{
    return a.Speed != b.Speed || a.IntentX != b.IntentX || a.IntentZ != b.IntentZ ||
        a.YawRate != b.YawRate || a.Contacts != b.Contacts || a.Clip != b.Clip ||
        a.ClipTime != b.ClipTime || a.DatabaseId != b.DatabaseId;
}

void MotionMatchingRuntimePolicy::NotifySearch()
{
    if (_playback != nullptr)
        _playback->NotifySearchExecuted(_nextSearchFrameId);
}

void MotionMatchingRuntimePolicy::EnsureSampleStarts()
{
    if (_hasSampleStarts)
        return;
    MotionMatchingDatabase* db = _database;
    if (db == nullptr)
        return;
    const int32 n = db->GetClipCount();
    const Array<int32>& counts = db->GetClipSampleCounts();
    if (n <= 0 || counts.Count() != n)
        return;
    _clipSampleStarts.Resize(n + 1);
    _clipSampleStarts[0] = 0;
    for (int32 i = 0; i < n; i++)
    {
        if (counts[i] < 0)
            return;
        _clipSampleStarts[i + 1] = _clipSampleStarts[i] + counts[i];
    }
    _hasSampleStarts = true;
}

void MotionMatchingRuntimePolicy::RecordTrace(float speed, float yawRate, const Array<String>& tags,
    const Array<float>& trajectory, const MotionMatchingSearchSettings& settings,
    int32 continuityClip, float sinceSwitch, bool recovered, bool valid,
    const MotionMatchingResult& result, const String& reason)
{
    if (_latestPose.Count() == 0 || trajectory.Count() == 0)
        return;
    MotionMatchingTraceEntry entry;
    entry.Kind = String(TEXT("search"));
    entry.Time = Time::GetGameTime();
    entry.State = PolicyState;
    for (int32 i = 0; i < tags.Count(); i++)
    {
        if (i > 0)
            entry.Tags += String(TEXT("+"));
        entry.Tags += tags[i];
    }
    entry.Speed = speed;
    entry.YawRate = yawRate;
    entry.Grounded = _snapshot.IsGrounded;
    entry.Sprinting = _snapshot.IsSprinting;
    entry.DesiredSpeed = Float2(_snapshot.DesiredVelocity.X, _snapshot.DesiredVelocity.Z).Length();
    entry.PoseWeight = settings.PoseWeight;
    entry.TrajectoryWeight = settings.TrajectoryWeight;
    entry.ContinuityWeight = settings.ContinuityWeight;
    entry.MinSwitchTime = settings.MinSwitchTime;
    entry.CurrentTime = settings.CurrentTime;
    entry.ExclusionWindow = settings.SameClipExclusionWindow;
    entry.ExpectedTime = settings.ExpectedTime;
    entry.ContinuingBias = settings.ContinuingBias;
    entry.ContinuingWindow = settings.ContinuingWindow;
    entry.SwitchMargin = settings.SwitchMargin;
    entry.LoopPreference = settings.LoopPreference;
    entry.LoopFilter = (int32)settings.LoopFilter;
    entry.ReselectClip = settings.ReselectClip;
    entry.ReselectSampleTime = settings.ReselectSampleTime;
    entry.ReselectBanWindow = settings.ReselectBanWindow;
    entry.TwinBanWindow = settings.TwinBanWindow;
    entry.LoopTailExclusion = settings.LoopTailExclusion;
    entry.FootLockL = settings.FootLockL;
    entry.FootLockR = settings.FootLockR;
    entry.ContinuityClip = continuityClip;
    entry.SinceSwitch = sinceSwitch;
    entry.Recovered = recovered;
    entry.Valid = valid;
    entry.ClipIndex = valid ? result.ClipIndex : -1;
    entry.SampleIndex = valid ? result.SampleIndex : -1;
    entry.SampleTime = valid ? result.SampleTime : 0.0f;
    entry.Cost = valid ? result.Cost : MAX_float;
    entry.PoseCost = valid ? result.PoseCost : MAX_float;
    entry.TrajectoryCost = valid ? result.TrajectoryCost : MAX_float;
    entry.ContinuityCost = valid ? result.ContinuityCost : 0.0f;
    entry.LoopCost = valid ? result.LoopCost : 0.0f;
    entry.SecondBestCost = valid ? result.SecondBestCost : MAX_float;
    entry.Reason = reason;
    entry.Chooser = ChooserStatusText;
    entry.QueryMs = LastQueryMs;
    _trace->Record(entry, _latestPose, trajectory);
    _lastEntry = entry;
}









float MotionMatchingRuntimePolicy::Wrap180(float a)
{
    while (a > 180.0f)
        a -= 360.0f;
    while (a < -180.0f)
        a += 360.0f;
    return a;
}













// P2: legacy unconstrained chooser deleted (was here).


void MotionMatchingRuntimePolicy::LogTelemetryLine(float speed, float yawRate, const Array<String>& tags,
    const MotionMatchingResult& result, const Float3& actorPos, float timeError)
{
    if (!_tuning.EffectiveTelemetry)
        return;
    if (_cacheFolder.IsEmpty())
        return;
    const float rateB = _playback != nullptr ? _playback->GetPlayRateBefore() : 1.0f;
    const float rateA = _playback != nullptr ? _playback->GetPlayRateAfter() : 1.0f;
    const float effDt = _playback != nullptr ? _playback->LastEffectiveDt : 0.0f;
    const int32 cycle = _playback != nullptr ? _playback->ClipCycle : 0;
    char tagBuf[128];
    tagBuf[0] = '\0';
    for (int32 i = 0; i < tags.Count() && (int32)strlen(tagBuf) < 100; i++)
    {
        const StringAnsi part(tags[i]);
        strncat(tagBuf, part.GetText(), sizeof(tagBuf) - strlen(tagBuf) - 1);
        if (i + 1 < tags.Count())
            strncat(tagBuf, "+", sizeof(tagBuf) - strlen(tagBuf) - 1);
    }
    char line[2048];
    const StringAnsi stateAnsi(PolicyState);
    const StringAnsi clipAnsi(WinnerClipName);
    const StringAnsi loopAnsi(BestLoopClipName);
    const StringAnsi oneAnsi(BestOneShotClipName);
    snprintf(line, sizeof(line),
        "{\"t\":%.2f,\"state\":\"%s\",\"tags\":\"%s\",\"speed\":%.1f,\"yaw\":%.2f,\"grounded\":%d,"
        "\"clip\":%d,\"name\":\"%s\",\"sample\":%d,\"cost\":%.4f,\"pose\":%.4f,\"traj\":%.4f,\"cont\":%.4f,\"loop\":%.4f,"
        "\"second\":%.4f,\"bestLoop\":\"%s:%.4f\",\"bestOne\":\"%s:%.4f\",\"loopN\":%d,\"oneN\":%d,"
        "\"loopT\":%d,\"oneT\":%d,"
        "\"pref\":%.2f,\"switches\":%d,\"seeks\":%d,\"cur\":%d,\"ptime\":%.2f,\"atEnd\":%d,"
        "\"px\":%.0f,\"py\":%.0f,\"pz\":%.0f,\"des\":%.0f,"
        "\"rateB\":%.3f,\"rateA\":%.3f,\"effDt\":%.4f,\"cycle\":%d,\"tErr\":%.4f}",
        (double)Time::GetGameTime(), stateAnsi.GetText(), tagBuf, (double)speed, (double)yawRate,
        _snapshot.IsGrounded ? 1 : 0,
        result.ClipIndex, clipAnsi.GetText(), result.SampleIndex, (double)result.Cost,
        (double)result.PoseCost, (double)result.TrajectoryCost, (double)result.ContinuityCost, (double)result.LoopCost,
        (double)result.SecondBestCost, loopAnsi.GetText(), (double)BestLoopCost, oneAnsi.GetText(), (double)BestOneShotCost,
        result.LoopScoredCount, result.OneShotScoredCount,
        result.LoopTotalCount, result.OneShotTotalCount,
        (double)LoopPreferenceApplied, SwitchCount, SeekCount, _currentClip,
        _playback != nullptr ? (double)_playback->CurrentTime : -1.0,
        (_playback != nullptr && _playback->IsAtEnd) ? 1 : 0,
        (double)actorPos.X, (double)actorPos.Y, (double)actorPos.Z,
        (double)Float2(_snapshot.DesiredVelocity.X, _snapshot.DesiredVelocity.Z).Length(),
        // P2: play-rate and timing telemetry only (overlay/IK keys deleted
        // with the plugin solver; game-side post passes own their telemetry).
        (double)rateB, (double)rateA, (double)effDt, cycle, (double)timeError);
    const StringAnsi folderAnsi(_cacheFolder);
    char path[1024];
    snprintf(path, sizeof(path), "%s/Cache/MMTelemetry.jsonl", folderAnsi.GetText());
    FILE* file = fopen(path, "ab");
    if (file != nullptr)
    {
        const size_t length = strlen(line);
        fwrite(line, 1, length, file);
        fwrite("\n", 1, 1, file);
        fclose(file);
    }
    // Telemetry must never break gameplay: all failures above are silent.
}
