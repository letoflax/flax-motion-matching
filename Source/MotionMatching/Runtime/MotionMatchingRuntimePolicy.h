#pragma once

#include "Engine/Core/Collections/Array.h"
#include "Engine/Core/Math/Matrix.h"
#include "Engine/Core/Math/Vector2.h"
#include "Engine/Core/Math/Vector3.h"
#include "Engine/Scripting/ScriptingObject.h"
#include "MotionMatchingPlayback.h"
#include "MotionMatchingTrace.h"
#include "../Schema/MotionMatchingTypes.h"
#include "../Schema/PoseSearchSchema.h"
#include "../Schema/TrajectoryPoint.h"
#include "../Search/MotionMatchingQuery.h"

class AnimatedModel;
class MotionMatchingDatabase;

/// <summary>
/// Per-tick movement snapshot for the policy (plain data, supplied by the
/// C# input provider). Only the fields the policy reads are mirrored.
/// </summary>
API_STRUCT(Namespace="MotionMatching")
struct MOTIONMATCHING_API MotionMatchingPlayerSnapshot
{
    DECLARE_SCRIPTING_TYPE_MINIMAL(MotionMatchingPlayerSnapshot);

    API_FIELD() Float3 CurrentVelocity = Float3::Zero;
    API_FIELD() Float3 DesiredVelocity = Float3::Zero;
    API_FIELD() Float3 AngularVelocity = Float3::Zero;
    API_FIELD() float TargetYaw = 0.0f;
    API_FIELD() float CurrentYaw = 0.0f;
    API_FIELD() bool IsGrounded = false;
    API_FIELD() bool IsSprinting = false;

    /// <summary>
    /// Provider-decided tag scope, '+'-joined (e.g. "run+turn_travel").
    /// P2: always read (native legacy chooser deleted). An EMPTY set is
    /// an explicit no-match: no candidate may win and the current clip
    /// holds (never fall back to the full pool). Plain String transport:
    /// the scripting marshaller cannot marshal Array&lt;String&gt; struct
    /// fields.
    /// </summary>
    API_FIELD() String AllowedTagSet;
    /// <summary>
    /// Controller-decided exclusions, '+'-joined. Merged into the search
    /// settings every query. Empty disables.
    /// </summary>
    API_FIELD() String ExcludedTagSet;
};

/// <summary>
/// Live tunables pushed by the C# controller every tick (plain data). Lets
/// the C++ policy read scene tuning without referencing the C# controller.
/// </summary>
API_STRUCT(Namespace="MotionMatching")
struct MOTIONMATCHING_API MotionMatchingPolicyTuning
{
    DECLARE_SCRIPTING_TYPE_MINIMAL(MotionMatchingPolicyTuning);

    API_FIELD() float PoseWeight = 1.0f;
    API_FIELD() float TrajectoryWeight = 2.0f;
    API_FIELD() float SampleRate = 30.0f;
    API_FIELD() float FairFightAccum = 40.0f;
    API_FIELD() float FairFightWindow = 0.25f;
    API_FIELD() float SettleToLoopDelay = 0.3f;
    API_FIELD() float MinSwitchTime = 0.12f;
    API_FIELD() float SameClipExclusion = 0.25f;
    API_FIELD() float SameClipSeekThreshold = 0.25f;
    API_FIELD() float ContinuingBias = 0.012f;
    API_FIELD() float ContinuingWindow = 0.15f;
    API_FIELD() float SwitchMargin = 0.008f;
    API_FIELD() float RestLoopPreference = 0.02f;
    API_FIELD() int32 FrameSearchBudget = 12000;
    API_FIELD() bool SearchEveryFrame = true;
    API_FIELD() float QueryInterval = 0.1f;
    API_FIELD() bool UseIndexedSearch = true;
    API_FIELD() int32 TopKCandidates = 16;
    API_FIELD() bool VerifyIndexedSearch = false;
    API_FIELD() bool IndexedVerifyAuto = false;
    API_FIELD() bool EffectiveTelemetry = false;
    API_FIELD() float ContinuityWeight = 0.0025f;
    API_FIELD() float CostFuseThreshold = 2.0f;

};

/// <summary>
/// C# animation command layer (plan Milestone 4). Owns when to search, which
/// tags may win, and continuity — but never touches movement authority: it
/// only tells MotionMatchingPlayback which clip to show.
/// States are intentionally coarse (Locomotion/Airborne); start/stop/turn
/// emerge from the search itself.
/// </summary>
API_CLASS(Namespace="MotionMatching")
class MOTIONMATCHING_API MotionMatchingRuntimePolicy : public ScriptingObject
{
    DECLARE_SCRIPTING_TYPE_WITH_CONSTRUCTOR_IMPL(MotionMatchingRuntimePolicy, ScriptingObject);

public:
    API_FIELD() String PolicyState = TEXT("Disabled");
    API_FIELD() String ActiveTag;
    API_FIELD() int32 WinnerSample = -1;
    API_FIELD() int32 WinnerClip = -1;
    API_FIELD() String WinnerClipName;
    API_FIELD() float WinnerCost = MAX_float;
    API_FIELD() float SecondBestCost = MAX_float;
    API_FIELD() float WinnerPoseCost = MAX_float;
    API_FIELD() float WinnerTrajectoryCost = MAX_float;
    API_FIELD() float WinnerContinuityCost = 0.0f;
    API_FIELD() float WinnerLoopCost = 0.0f;
    API_FIELD() bool WinnerIsLoop = false;
    API_FIELD() float LoopPreferenceApplied = 0.0f;
    API_FIELD() float BestLoopCost = MAX_float;
    API_FIELD() int32 BestLoopClip = -1;
    API_FIELD() String BestLoopClipName;
    API_FIELD() float BestOneShotCost = MAX_float;
    API_FIELD() int32 BestOneShotClip = -1;
    API_FIELD() String BestOneShotClipName;
    API_FIELD() int32 LoopScoredCount = 0;
    API_FIELD() int32 OneShotScoredCount = 0;
    API_FIELD() int32 LoopTotalCount = 0;
    API_FIELD() int32 OneShotTotalCount = 0;
    API_FIELD() int32 SwitchCount = 0;
    API_FIELD() int32 SeekCount = 0;
    API_FIELD() float LastQueryMs = 0.0f;
    API_FIELD() String LastSwitchReason;
    API_FIELD() String ChooserStatusText;
    API_FIELD() int32 FastPathHits = 0;
    API_FIELD() float LastFastPathAge = 0.0f;
    API_FIELD() String ForcedReason;
    API_FIELD() float LastTimeError = 0.0f;
    // P0 additive profiling counters (record-only; never steer search or
    // playback). LastQueryMs above already measures the full-scan block
    // (FindBest + retries); the fields below split the remaining phases so
    // before/after runs can compare update/search/fast-path/I-O cost.
    // Whole-Tick wall time (pose sample + query/fast-path + playback).
    API_FIELD() float LastTickMs = 0.0f;
    // Continuation SampleCost block inside TryFastPath (hit path only).
    API_FIELD() float LastFastPathMs = 0.0f;
    // Synchronous telemetry/audit file I/O (LogTelemetryLine per query +
    // WriteClipAuditFile at init). Telemetry-off ticks record nothing.
    API_FIELD() float LastTelemetryMs = 0.0f;
    API_FIELD() int32 TickCount = 0;
    API_FIELD() int32 FullSearchCount = 0;
    API_FIELD() float TotalTickMs = 0.0f;
    API_FIELD() float TotalFullSearchMs = 0.0f;
    API_FIELD() float TotalFastPathMs = 0.0f;
    API_FIELD() float TotalTelemetryMs = 0.0f;

    API_PROPERTY() bool GetIsReady() const;

    ~MotionMatchingRuntimePolicy();

    API_FUNCTION() void Initialize(
        MotionMatchingDatabase* database,
        AnimatedModel* body,
        const PoseSearchSchema& schema,
        const String& cacheFolder,
        float defaultInertial);
    API_FUNCTION() void Shutdown();
    API_FUNCTION() void ResetOnTeleport();
    API_FUNCTION() void Tick(
        float deltaTime,
        const MotionMatchingPlayerSnapshot& state,
        const Array<TrajectoryPoint>& trajectoryPoints,
        const Matrix& actorWorld,
        const Array<float>& trajectorySampleTimes,
        const MotionMatchingPolicyTuning& tuning);
    API_FUNCTION() MotionMatchingPlayback* GetPlayback();
    API_FUNCTION() MotionMatchingTraceLog* GetTrace();
    API_FUNCTION() MotionMatchingTraceEntry GetLastEntry() const;

private:
    struct FastPathSignature
    {
        int32 Speed = 0;
        int32 IntentX = 0;
        int32 IntentZ = 0;
        int32 YawRate = 0;
        int32 Contacts = 0;
        int32 Clip = -1;
        int32 ClipTime = 0;
        int32 DatabaseId = 0;
    };

    MotionMatchingDatabase* _database = nullptr;
    AnimatedModel* _body = nullptr;
    MotionMatchingPlayback* _playback = nullptr;
    MotionMatchingTraceLog* _trace = nullptr;
    PoseSearchSchema _liveSchema;
    String _cacheFolder;
    NodeIndices _nodes;

    bool _samplingReady = false;
    Array<float> _prevPositions;
    bool _hasPrevPositions = false;
    bool _usedSearchSnapshot = false;
    int32 _searchSnapshotLength = 0;
    Array<float> _latestPose;
    Array<float> _packedTrajectory;
    Matrix _latestRootWorld = Matrix::Identity;
    bool _poseValid = false;
    Array<float> _tickTrajectory;
    float _lastScanTotal = MAX_float;
    float _lastFullScanTime = -99.0f;
    int32 _lastFullScanFrame = -1;
    int32 _consecutiveContinuationMisses = 0;
    FastPathSignature _lastScanSignature;
    bool _hasLastScanSignature = false;
    bool _fastPathReseed = true;
    int32 _policyFrame = 0;
    int64 _nextSearchFrameId = 1;
    int64 _lastObservedSampleFrameId = -1;
    int32 _fastPathFrames = 0;
    Array<int32> _clipSampleStarts;
    bool _hasSampleStarts = false;

    bool _tickOneshotWindow = false;
    bool _tickFairFight = false;
    float _intentAccum = 0.0f;
    float _fairFightUntil = -99.0f;
    float _queryTimer = 0.0f;
    float _timeSinceSwitch = 999.0f;
    float _timeSinceInit = 0.0f;
    int32 _currentClip = -1;
    bool _wasAirborne = false;
    bool _hasLastActorPos = false;
    Float3 _lastActorPos = Float3::Zero;
    bool _warnedOnce = false;
    bool _bindingWarned = false;

    bool _chooserReseed = false;

    Array<float> _clipTravel;
    Array<float> _clipTurnAngle;
    Array<float> _clipNet;
    Array<Array<String>> _clipTags;

    float _lastSpikeWarn = -99.0f;
    Float3 _prevDesVel = Float3::Zero;
    float _prevDesSpeed = 0.0f;
    float _oneshotUntil = -99.0f;
    int32 _reselectClip = -1;
    float _reselectSampleTime = 0.0f;
    float _reselectStartTime = -99.0f;


    MotionMatchingPlayerSnapshot _snapshot;
    MotionMatchingPolicyTuning _tuning;
    Array<TrajectoryPoint> _trajectoryPoints;
    Array<float> _sampleTimes;
    MotionMatchingTraceEntry _lastEntry;

    static float ClampF(float value, float minValue, float maxValue);
    static float MaxF(float a, float b);
    static float MinF(float a, float b);
    static float AbsF(float value);
    static int32 MaxI(int32 a, int32 b);

    float SelectPlayRate();
    void MeasureClipMotion();
    void LogAccelAudit();
    void WriteClipAuditFile();
    void CheckSchemaBinding();
    static bool AxesNear(const Float3& a, const Float3& b);
    static bool TimesEqual(const Array<float>& a, const Array<float>& b);
    void UpdateIntentSignals();
    int32 BandFromSpeed(float speed) const;
    void RecordPolicyEvent(const char* kind, int32 clipIndex, const String& reason);
    void RunQuery(const Array<TrajectoryPoint>& trajectoryPoints, const Matrix& actorWorld, float frameDt);
    bool TryFastPath(float frameDt);
    bool MissFastPath(const char* reason);
    static int32 Quantize(float value, float step);
    FastPathSignature CaptureFastPathSignature() const;
    static bool FastPathSignaturesDiffer(const FastPathSignature& a, const FastPathSignature& b);
    void NotifySearch();
    void EnsureSampleStarts();
    void RecordTrace(float speed, float yawRate, const Array<String>& tags,
        const Array<float>& trajectory, const MotionMatchingSearchSettings& settings,
        int32 continuityClip, float sinceSwitch, int32 retryKind, bool forcedEdge, bool reseeded,
        bool fairFight, bool oneshotWindow, float frameDt, float playRate,
        bool valid, const MotionMatchingResult& result, const String& reason);
    static float Wrap180(float a);
    void WarnOnce(const String& message);
    void LogTelemetryLine(float speed, float yawRate, const Array<String>& tags,
        const MotionMatchingResult& result, const Float3& actorPos, float timeError);
    String GetClipName(int32 clipIndex);
    // Normalized tag SET snapshot (MeasureClipMotion) with DB fallback.
    const Array<String>& ClipTags(int32 clipIndex);
    // Primary tag (element 0) for display/audit contexts.
    String ClipTag(int32 clipIndex);
};
