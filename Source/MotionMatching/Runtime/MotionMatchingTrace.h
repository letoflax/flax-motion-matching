#pragma once

#include "Engine/Core/Collections/Array.h"
#include "Engine/Core/Types/String.h"
#include "Engine/Core/Types/StringBuilder.h"
#include "Engine/Scripting/ScriptingObject.h"
#include "../Search/MotionMatchingQuery.h"

class MotionMatchingDatabase;

/// <summary>
/// One recorded search: everything needed to re-run it bit-exactly.
/// Kind separates search from policy/playback events: "search" ran FindBest
/// (replayable), "hold" skipped the query (turn hold), "replay" restarted a
/// clip with no search (turn replay). Replay verify only re-runs "search"
/// entries and reports the rest as skipped.
/// </summary>
API_STRUCT(Namespace="MotionMatching")
struct MOTIONMATCHING_API MotionMatchingTraceEntry
{
    DECLARE_SCRIPTING_TYPE_MINIMAL(MotionMatchingTraceEntry);

    API_FIELD() String Kind = TEXT("search");
    API_FIELD() float Time = 0.0f;
    API_FIELD() String State;
    API_FIELD() String Tags;
    API_FIELD() float Speed = 0.0f;
    API_FIELD() float YawRate = 0.0f;
    API_FIELD() bool Grounded = true;
    API_FIELD() bool Sprinting = false;
    API_FIELD() float DesiredSpeed = 0.0f;
    API_FIELD() Array<float> Pose;
    API_FIELD() Array<float> Trajectory;
    API_FIELD() float PoseWeight = 1.0f;
    API_FIELD() float TrajectoryWeight = 2.0f;
    API_FIELD() float ContinuityWeight = 0.25f;
    API_FIELD() float MinSwitchTime = 0.12f;
    API_FIELD() float CurrentTime = -1.0f;
    API_FIELD() float ExclusionWindow = 0.25f;
    API_FIELD() float ExpectedTime = -1.0f;
    API_FIELD() float ContinuingBias = 0.25f;
    API_FIELD() float ContinuingWindow = 0.15f;
    API_FIELD() float SwitchMargin = 0.1f;
    API_FIELD() float LoopPreference = 0.0f;
    API_FIELD() int32 LoopFilter = 0;
    API_FIELD() int32 ReselectClip = -1;
    API_FIELD() float ReselectSampleTime = 0.0f;
    API_FIELD() float ReselectBanWindow = 0.0f;
    API_FIELD() float TwinBanWindow = 0.0f;
    API_FIELD() float LoopTailExclusion = 0.0f;
    API_FIELD() float FootLockL = 0.0f;
    API_FIELD() float FootLockR = 0.0f;
    API_FIELD() int32 ContinuityClip = -1;
    API_FIELD() float SinceSwitch = 999.0f;
    API_FIELD() bool Recovered = false;
    API_FIELD() bool Valid = false;
    API_FIELD() int32 ClipIndex = -1;
    API_FIELD() int32 SampleIndex = -1;
    API_FIELD() float SampleTime = 0.0f;
    API_FIELD() float Cost = MAX_float;
    API_FIELD() float PoseCost = MAX_float;
    API_FIELD() float TrajectoryCost = MAX_float;
    API_FIELD() float ContinuityCost = 0.0f;
    API_FIELD() float LoopCost = 0.0f;
    API_FIELD() float SecondBestCost = MAX_float;
    API_FIELD() String Reason;
    API_FIELD() String Chooser;
    API_FIELD() float QueryMs = 0.0f;
};

/// <summary>
/// In-memory ring of recent queries plus JSON dump/load, CSV statistics
/// export, and deterministic replay verify (re-runs the native search from
/// recorded inputs and compares the winner sample). Runtime-safe (no editor
/// types); the tuning window drives it.
/// </summary>
API_CLASS(Namespace="MotionMatching")
class MOTIONMATCHING_API MotionMatchingTraceLog : public ScriptingObject
{
    DECLARE_SCRIPTING_TYPE_WITH_CONSTRUCTOR_IMPL(MotionMatchingTraceLog, ScriptingObject);

public:
    // Ring capacity: 4096 entries ≈ 60s at 60 Hz queries. Memory stays
    // modest for one tuned character.
    static constexpr int32 Capacity = 4096;
    static constexpr int32 SchemaVersion = 2;

private:
    MotionMatchingTraceEntry _entries[Capacity];
    int32 _head = 0;
    int32 _count = 0;

    static void FormatFloat(StringBuilder& sb, float value);
    static void FormatFloatText(float value, char* buffer, int32 capacity);
    static void EscapeInto(StringBuilder& sb, const String& value);
    static void AppendFloats(StringBuilder& sb, const Array<float>& values);
    static bool ParseEntry(const String& line, MotionMatchingTraceEntry& entry);
    static float ParseFloatToken(const Char* text, int32 start, int32 end);
    static int32 FindKey(const Char* text, int32 length, const char* key);
    static float GetFloat(const Char* text, int32 length, const char* key);
    static String GetString(const Char* text, int32 length, const char* key);
    static Array<float> GetFloatArray(const Char* text, int32 length, const char* key);

public:
    // Debug-only gate: recording costs a pose+trajectory clone per query.
    // The policy enables this only from EffectiveTelemetry.
    API_FIELD() bool Enabled = true;

    API_PROPERTY() int32 GetCount() const;
    API_FUNCTION() void Clear();
    // NOTE: pose/trajectory arrays travel as function params, NOT struct
    // fields — C#->C++ struct Array fields do not round-trip (verified:
    // file exports came out empty). Function Array params marshal fine
    // (same path as FindBest). The entry keeps the fields for reads.
    API_FUNCTION() void Record(const MotionMatchingTraceEntry& entry, const Array<float>& pose, const Array<float>& trajectory);
    API_FUNCTION() Array<MotionMatchingTraceEntry> Snapshot() const;
    API_FUNCTION() bool ExportJson(const String& path, API_PARAM(Out) String& error);
    API_FUNCTION() bool ImportJson(const String& path, API_PARAM(Out) String& error);
    API_FUNCTION() bool ExportCsv(const String& path, API_PARAM(Out) String& error, API_PARAM(Out) String& summary);
    API_FUNCTION() bool VerifyReplay(MotionMatchingDatabase* database, int32 maxEntries, API_PARAM(Out) String& report);
};
