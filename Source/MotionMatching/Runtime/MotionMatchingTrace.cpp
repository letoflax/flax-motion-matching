#include "MotionMatchingTrace.h"

#include "../Database/MotionMatchingDatabase.h"
#include "../Search/MotionMatchingSearch.h"
#include "Engine/Core/Log.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace
{
    bool IsNumChar(Char c)
    {
        return (c >= '0' && c <= '9') || c == '+' || c == '-' || c == '.' || c == 'e' || c == 'E';
    }

    int32 CompareKey(const Char* text, int32 pos, const char* key)
    {
        for (int32 k = 0; key[k] != '\0'; k++)
        {
            if (text[pos + k] != (Char)key[k])
                return key[k] - (char)text[pos + k];
        }
        return 0;
    }
}

int32 MotionMatchingTraceLog::GetCount() const
{
    return _count;
}

void MotionMatchingTraceLog::Clear()
{
    for (int32 i = 0; i < Capacity; i++)
        _entries[i] = MotionMatchingTraceEntry();
    _head = 0;
    _count = 0;
}

void MotionMatchingTraceLog::Record(const MotionMatchingTraceEntry& entry, const Array<float>& pose, const Array<float>& trajectory)
{
    if (!Enabled)
        return;
    MotionMatchingTraceEntry stored = entry;
    stored.Pose = pose;
    stored.Trajectory = trajectory;
    _entries[_head] = stored;
    _head = (_head + 1) % Capacity;
    if (_count < Capacity)
        _count++;
}

Array<MotionMatchingTraceEntry> MotionMatchingTraceLog::Snapshot() const
{
    Array<MotionMatchingTraceEntry> result;
    result.Resize(_count);
    for (int32 i = 0; i < _count; i++)
        result[i] = _entries[(_head - _count + i + Capacity * 2) % Capacity];
    return result;
}

void MotionMatchingTraceLog::FormatFloatText(float value, char* buffer, int32 capacity)
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

void MotionMatchingTraceLog::FormatFloat(StringBuilder& sb, float value)
{
    char buffer[32];
    FormatFloatText(value, buffer, sizeof(buffer));
    sb.Append(buffer);
}

void MotionMatchingTraceLog::EscapeInto(StringBuilder& sb, const String& value)
{
    if (value.IsEmpty())
        return;
    const Char* text = value.GetText();
    const int32 length = value.Length();
    for (int32 i = 0; i < length; i++)
    {
        if (text[i] == '\\' || text[i] == '"')
            sb.Append('\\');
        char c = (char)text[i];
        sb.Append(c);
    }
}

void MotionMatchingTraceLog::AppendFloats(StringBuilder& sb, const Array<float>& values)
{
    for (int32 i = 0; i < values.Count(); i++)
    {
        if (i > 0)
            sb.Append(',');
        FormatFloat(sb, values[i]);
    }
}

bool MotionMatchingTraceLog::ExportJson(const String& path, String& error)
{
    error = String::Empty;
    StringBuilder sb;
    const Array<MotionMatchingTraceEntry> snap = Snapshot();
    for (int32 n = 0; n < snap.Count(); n++)
    {
        const MotionMatchingTraceEntry& e = snap[n];
        sb.Append("{\"v\":");
        sb.Append(SchemaVersion);
        sb.Append(",\"kind\":\"");
        EscapeInto(sb, e.Kind);
        sb.Append("\",\"t\":");
        FormatFloat(sb, e.Time);
        sb.Append(",\"state\":\"");
        EscapeInto(sb, e.State);
        sb.Append("\",\"tags\":\"");
        EscapeInto(sb, e.Tags);
        sb.Append("\",\"speed\":");
        FormatFloat(sb, e.Speed);
        sb.Append(",\"yaw\":");
        FormatFloat(sb, e.YawRate);
        sb.Append(",\"grounded\":");
        sb.Append(e.Grounded ? '1' : '0');
        sb.Append(",\"sprint\":");
        sb.Append(e.Sprinting ? '1' : '0');
        sb.Append(",\"des\":");
        FormatFloat(sb, e.DesiredSpeed);
        sb.Append(",\"pose\":[");
        AppendFloats(sb, e.Pose);
        sb.Append("],\"traj\":[");
        AppendFloats(sb, e.Trajectory);
        sb.Append("],\"pw\":");
        FormatFloat(sb, e.PoseWeight);
        sb.Append(",\"tw\":");
        FormatFloat(sb, e.TrajectoryWeight);
        sb.Append(",\"cw\":");
        FormatFloat(sb, e.ContinuityWeight);
        sb.Append(",\"minsw\":");
        FormatFloat(sb, e.MinSwitchTime);
        sb.Append(",\"curt\":");
        FormatFloat(sb, e.CurrentTime);
        sb.Append(",\"excl\":");
        FormatFloat(sb, e.ExclusionWindow);
        sb.Append(",\"expt\":");
        FormatFloat(sb, e.ExpectedTime);
        sb.Append(",\"bias\":");
        FormatFloat(sb, e.ContinuingBias);
        sb.Append(",\"biasw\":");
        FormatFloat(sb, e.ContinuingWindow);
        sb.Append(",\"margin\":");
        FormatFloat(sb, e.SwitchMargin);
        sb.Append(",\"looppref\":");
        FormatFloat(sb, e.LoopPreference);
        sb.Append(",\"loopfilter\":");
        sb.Append(e.LoopFilter);
        sb.Append(",\"rsclip\":");
        sb.Append(e.ReselectClip);
        sb.Append(",\"rsstime\":");
        FormatFloat(sb, e.ReselectSampleTime);
        sb.Append(",\"rswin\":");
        FormatFloat(sb, e.ReselectBanWindow);
        sb.Append(",\"twinban\":");
        FormatFloat(sb, e.TwinBanWindow);
        sb.Append(",\"tailexcl\":");
        FormatFloat(sb, e.LoopTailExclusion);
        sb.Append(",\"flockL\":");
        FormatFloat(sb, e.FootLockL);
        sb.Append(",\"flockR\":");
        FormatFloat(sb, e.FootLockR);
        sb.Append(",\"contclip\":");
        sb.Append(e.ContinuityClip);
        sb.Append(",\"sincesw\":");
        FormatFloat(sb, e.SinceSwitch);
        sb.Append(",\"recovered\":");
        sb.Append(e.Recovered ? '1' : '0');
        sb.Append(",\"valid\":");
        sb.Append(e.Valid ? '1' : '0');
        sb.Append(",\"clip\":");
        sb.Append(e.ClipIndex);
        sb.Append(",\"sample\":");
        sb.Append(e.SampleIndex);
        sb.Append(",\"stime\":");
        FormatFloat(sb, e.SampleTime);
        sb.Append(",\"cost\":");
        FormatFloat(sb, e.Cost);
        sb.Append(",\"poseC\":");
        FormatFloat(sb, e.PoseCost);
        sb.Append(",\"trajC\":");
        FormatFloat(sb, e.TrajectoryCost);
        sb.Append(",\"contC\":");
        FormatFloat(sb, e.ContinuityCost);
        sb.Append(",\"loopC\":");
        FormatFloat(sb, e.LoopCost);
        sb.Append(",\"second\":");
        FormatFloat(sb, e.SecondBestCost);
        sb.Append(",\"reason\":\"");
        EscapeInto(sb, e.Reason);
        sb.Append("\",\"chooser\":\"");
        EscapeInto(sb, e.Chooser);
        sb.Append("\",\"qms\":");
        FormatFloat(sb, e.QueryMs);
        sb.Append("}\n");
    }
    String text;
    sb.ToString(text);
    const StringAnsi pathAnsi(path);
    FILE* file = fopen(pathAnsi.GetText(), "wb");
    if (file == nullptr)
    {
        error = String(TEXT("Cannot write trace file."));
        return false;
    }
    const StringAnsi bodyAnsi(text);
    const char* body = bodyAnsi.GetText();
    const int32 bodyLength = bodyAnsi.Length();
    const size_t written = fwrite(body, 1, (size_t)bodyLength, file);
    fclose(file);
    if ((int32)written != bodyLength)
    {
        error = String(TEXT("Trace file write incomplete."));
        return false;
    }
    return true;
}

int32 MotionMatchingTraceLog::FindKey(const Char* text, int32 length, const char* key)
{
    const int32 keyLength = (int32)strlen(key);
    if (keyLength == 0 || length < keyLength)
        return -1;
    for (int32 i = 0; i + keyLength <= length; i++)
    {
        if (CompareKey(text, i, key) == 0)
            return i;
    }
    return -1;
}

float MotionMatchingTraceLog::ParseFloatToken(const Char* text, int32 start, int32 end)
{
    char buffer[64];
    int32 count = 0;
    for (int32 i = start; i < end && count < 63; i++)
    {
        const char c = (char)text[i];
        if ((c >= '0' && c <= '9') || c == '+' || c == '-' || c == '.' || c == 'e' || c == 'E')
            buffer[count++] = c;
        else
            break;
    }
    if (count == 0)
        return 0.0f;
    buffer[count] = '\0';
    return strtof(buffer, nullptr);
}

float MotionMatchingTraceLog::GetFloat(const Char* text, int32 length, const char* key)
{
    const int32 at = FindKey(text, length, key);
    if (at < 0)
        return 0.0f;
    const int32 start = at + (int32)strlen(key);
    int32 end = start;
    while (end < length && IsNumChar(text[end]))
        end++;
    if (end <= start)
        return 0.0f;
    return ParseFloatToken(text, start, end);
}

String MotionMatchingTraceLog::GetString(const Char* text, int32 length, const char* key)
{
    const int32 at = FindKey(text, length, key);
    if (at < 0)
        return String::Empty;
    int32 i = at + (int32)strlen(key);
    StringBuilder sb;
    while (i < length && text[i] != '"')
    {
        if (text[i] == '\\' && i + 1 < length)
        {
            i++;
            sb.Append((char)text[i]);
        }
        else
        {
            // StringBuilder has no Char-wide append-all overload used here;
            // values are ASCII (tags, kind, state, reason, chooser).
            sb.Append((char)text[i]);
        }
        i++;
    }
    String result;
    sb.ToString(result);
    return result;
}

Array<float> MotionMatchingTraceLog::GetFloatArray(const Char* text, int32 length, const char* key)
{
    Array<float> values;
    const int32 at = FindKey(text, length, key);
    if (at < 0)
        return values;
    int32 i = at + (int32)strlen(key);
    int32 close = -1;
    for (int32 j = i; j < length; j++)
    {
        if (text[j] == ']')
        {
            close = j;
            break;
        }
    }
    if (close < 0)
        return values;
    int32 tokenStart = -1;
    for (int32 j = i; j <= close; j++)
    {
        const bool isNum = j < close && IsNumChar(text[j]);
        if (isNum && tokenStart < 0)
            tokenStart = j;
        if (!isNum && tokenStart >= 0)
        {
            values.Add(ParseFloatToken(text, tokenStart, j));
            tokenStart = -1;
        }
    }
    return values;
}

bool MotionMatchingTraceLog::ParseEntry(const String& line, MotionMatchingTraceEntry& entry)
{
    entry = MotionMatchingTraceEntry();
    const Char* text = line.GetText();
    const int32 length = line.Length();
    const int32 version = (int32)GetFloat(text, length, "\"v\":");
    if (version != 1 && version != SchemaVersion)
        return false;
    entry.Kind = GetString(text, length, "\"kind\":\"");
    if (entry.Kind != String(TEXT("search")) && entry.Kind != String(TEXT("hold")) && entry.Kind != String(TEXT("replay")))
        return false;
    entry.Time = GetFloat(text, length, "\"t\":");
    entry.State = GetString(text, length, "\"state\":\"");
    entry.Tags = GetString(text, length, "\"tags\":\"");
    entry.Speed = GetFloat(text, length, "\"speed\":");
    entry.YawRate = GetFloat(text, length, "\"yaw\":");
    entry.Grounded = GetFloat(text, length, "\"grounded\":") > 0.5f;
    entry.Sprinting = GetFloat(text, length, "\"sprint\":") > 0.5f;
    entry.DesiredSpeed = GetFloat(text, length, "\"des\":");
    entry.Pose = GetFloatArray(text, length, "\"pose\":[");
    entry.Trajectory = GetFloatArray(text, length, "\"traj\":[");
    entry.PoseWeight = GetFloat(text, length, "\"pw\":");
    entry.TrajectoryWeight = GetFloat(text, length, "\"tw\":");
    entry.ContinuityWeight = GetFloat(text, length, "\"cw\":");
    entry.MinSwitchTime = GetFloat(text, length, "\"minsw\":");
    entry.CurrentTime = GetFloat(text, length, "\"curt\":");
    entry.ExclusionWindow = GetFloat(text, length, "\"excl\":");
    entry.ExpectedTime = GetFloat(text, length, "\"expt\":");
    entry.ContinuingBias = GetFloat(text, length, "\"bias\":");
    entry.ContinuingWindow = GetFloat(text, length, "\"biasw\":");
    entry.SwitchMargin = GetFloat(text, length, "\"margin\":");
    entry.LoopPreference = GetFloat(text, length, "\"looppref\":");
    entry.LoopFilter = (int32)GetFloat(text, length, "\"loopfilter\":");
    if (FindKey(text, length, "\"rsclip\":") >= 0)
    {
        entry.ReselectClip = (int32)GetFloat(text, length, "\"rsclip\":");
        entry.ReselectSampleTime = GetFloat(text, length, "\"rsstime\":");
        entry.ReselectBanWindow = GetFloat(text, length, "\"rswin\":");
        entry.TwinBanWindow = GetFloat(text, length, "\"twinban\":");
        entry.LoopTailExclusion = GetFloat(text, length, "\"tailexcl\":");
    }
    else
    {
        entry.ReselectClip = -1;
        entry.ReselectBanWindow = 0.0f;
        entry.TwinBanWindow = 0.0f;
        entry.LoopTailExclusion = 0.0f;
    }
    entry.FootLockL = GetFloat(text, length, "\"flockL\":");
    entry.FootLockR = GetFloat(text, length, "\"flockR\":");
    entry.ContinuityClip = (int32)GetFloat(text, length, "\"contclip\":");
    entry.SinceSwitch = GetFloat(text, length, "\"sincesw\":");
    entry.Recovered = GetFloat(text, length, "\"recovered\":") > 0.5f;
    entry.Valid = GetFloat(text, length, "\"valid\":") > 0.5f;
    entry.ClipIndex = (int32)GetFloat(text, length, "\"clip\":");
    entry.SampleIndex = (int32)GetFloat(text, length, "\"sample\":");
    entry.SampleTime = GetFloat(text, length, "\"stime\":");
    entry.Cost = GetFloat(text, length, "\"cost\":");
    entry.PoseCost = GetFloat(text, length, "\"poseC\":");
    entry.TrajectoryCost = GetFloat(text, length, "\"trajC\":");
    entry.ContinuityCost = GetFloat(text, length, "\"contC\":");
    entry.LoopCost = GetFloat(text, length, "\"loopC\":");
    entry.SecondBestCost = GetFloat(text, length, "\"second\":");
    entry.Reason = GetString(text, length, "\"reason\":\"");
    entry.Chooser = GetString(text, length, "\"chooser\":\"");
    entry.QueryMs = GetFloat(text, length, "\"qms\":");
    if (entry.Kind == String(TEXT("search")) && (entry.Pose.Count() == 0 || entry.Trajectory.Count() == 0))
        return false;
    return true;
}

bool MotionMatchingTraceLog::ImportJson(const String& path, String& error)
{
    error = String::Empty;
    const StringAnsi pathAnsi(path);
    FILE* file = fopen(pathAnsi.GetText(), "rb");
    if (file == nullptr)
    {
        error = String(TEXT("Cannot open trace file."));
        return false;
    }
    fseek(file, 0, SEEK_END);
    const long size = ftell(file);
    fseek(file, 0, SEEK_SET);
    if (size <= 0 || size > 256 * 1024 * 1024)
    {
        fclose(file);
        error = String(TEXT("Trace file is empty or too large."));
        return false;
    }
    char* bytes = (char*)malloc((size_t)size + 1);
    if (bytes == nullptr)
    {
        fclose(file);
        error = String(TEXT("Out of memory reading trace file."));
        return false;
    }
    const size_t read = fread(bytes, 1, (size_t)size, file);
    fclose(file);
    bytes[read] = '\0';
    Clear();
    int32 loaded = 0;
    int32 invalid = 0;
    int32 lineStart = 0;
    for (int32 i = 0; i <= (int32)read; i++)
    {
        if (i == (int32)read || bytes[i] == '\n')
        {
            int32 lineEnd = i;
            while (lineEnd > lineStart && (bytes[lineEnd - 1] == '\r' || bytes[lineEnd - 1] == ' ' || bytes[lineEnd - 1] == '\t'))
                lineEnd--;
            while (lineStart < lineEnd && (bytes[lineStart] == ' ' || bytes[lineStart] == '\t'))
                lineStart++;
            if (lineEnd - lineStart >= 2 && bytes[lineStart] == '{')
            {
                String line(bytes + lineStart, lineEnd - lineStart);
                MotionMatchingTraceEntry entry;
                if (ParseEntry(line, entry))
                {
                    Record(entry, entry.Pose, entry.Trajectory);
                    loaded++;
                }
                else
                {
                    invalid++;
                }
            }
            lineStart = i + 1;
        }
    }
    free(bytes);
    if (loaded == 0)
    {
        if (invalid > 0)
        {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "No valid trace entries in file (%d invalid lines rejected: bad version or malformed).", invalid);
            error = String(buffer);
        }
        else
        {
            error = String(TEXT("No trace entries found in file."));
        }
        return false;
    }
    if (invalid > 0)
    {
        LOG(Warning, "Motion matching trace import: {} entries loaded, {} invalid lines rejected.", loaded, invalid);
    }
    return true;
}

bool MotionMatchingTraceLog::ExportCsv(const String& path, String& error, String& summary)
{
    error = String::Empty;
    summary = String::Empty;
    const Array<MotionMatchingTraceEntry> snap = Snapshot();
    if (snap.Count() == 0)
    {
        error = String(TEXT("Trace is empty: play first, then export."));
        return false;
    }
    double costSum = 0.0;
    int32 costN = 0;
    double msSum = 0.0;
    int32 switches = 0;
    int32 lastClip = -2;
    Array<String> tagNames;
    Array<int32> tagCounts;
    for (int32 n = 0; n < snap.Count(); n++)
    {
        const MotionMatchingTraceEntry& e = snap[n];
        if (e.Valid && e.Cost < 1e29f)
        {
            costSum += (double)e.Cost;
            costN++;
        }
        msSum += (double)e.QueryMs;
        if (e.ClipIndex != lastClip)
        {
            if (lastClip != -2)
                switches++;
            lastClip = e.ClipIndex;
        }
        const String tag = e.Tags.IsEmpty() ? String(TEXT("?")) : e.Tags;
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
            tagCounts.Add(1);
        }
        else
        {
            tagCounts[found]++;
        }
    }
    const float span = snap[snap.Count() - 1].Time - snap[0].Time;
    const double switchRate = span > 0.01 ? (double)switches / (double)span : 0.0;
    char buffer[256];
    StringBuilder sb;
    snprintf(buffer, sizeof(buffer), "# queries=%d span_s=%.2f avg_cost=%.4f switches=%d switches_per_s=%.2f avg_query_ms=%.3f\n",
        snap.Count(), (double)span, costN > 0 ? costSum / (double)costN : 0.0, switches, switchRate,
        snap.Count() > 0 ? msSum / (double)snap.Count() : 0.0);
    sb.Append(buffer);
    for (int32 k = 0; k < tagNames.Count(); k++)
    {
        StringAnsi nameAnsi(tagNames[k]);
        snprintf(buffer, sizeof(buffer), "# tag %s=%d\n", nameAnsi.GetText(), tagCounts[k]);
        sb.Append(buffer);
    }
    sb.Append("t,kind,state,tags,speed,yaw,grounded,sprint,des,valid,clip,sample,cost,poseC,trajC,contC,loopC,second,reason,chooser,qms\n");
    for (int32 n = 0; n < snap.Count(); n++)
    {
        const MotionMatchingTraceEntry& e = snap[n];
        StringAnsi kindAnsi(e.Kind);
        StringAnsi stateAnsi(e.State);
        StringAnsi tagsAnsi(e.Tags);
        StringAnsi reasonAnsi(e.Reason);
        StringAnsi chooserAnsi(e.Chooser);
        char fTime[32], fSpeed[32], fYaw[32], fDes[32];
        char fCost[32], fPoseC[32], fTrajC[32], fContC[32], fLoopC[32], fSecond[32], fQms[32];
        FormatFloatText(e.Time, fTime, sizeof(fTime));
        FormatFloatText(e.Speed, fSpeed, sizeof(fSpeed));
        FormatFloatText(e.YawRate, fYaw, sizeof(fYaw));
        FormatFloatText(e.DesiredSpeed, fDes, sizeof(fDes));
        FormatFloatText(e.Cost, fCost, sizeof(fCost));
        FormatFloatText(e.PoseCost, fPoseC, sizeof(fPoseC));
        FormatFloatText(e.TrajectoryCost, fTrajC, sizeof(fTrajC));
        FormatFloatText(e.ContinuityCost, fContC, sizeof(fContC));
        FormatFloatText(e.LoopCost, fLoopC, sizeof(fLoopC));
        FormatFloatText(e.SecondBestCost, fSecond, sizeof(fSecond));
        FormatFloatText(e.QueryMs, fQms, sizeof(fQms));
        snprintf(buffer, sizeof(buffer),
            "%s,%s,%s,%s,%s,%s,%d,%d,%s,%d,%d,%d,%s,%s,%s,%s,%s,%s,\"%s\",\"%s\",%s\n",
            fTime, kindAnsi.GetText(), stateAnsi.GetText(), tagsAnsi.GetText(),
            fSpeed, fYaw,
            e.Grounded ? 1 : 0, e.Sprinting ? 1 : 0,
            fDes, e.Valid ? 1 : 0,
            e.ClipIndex, e.SampleIndex,
            fCost, fPoseC, fTrajC, fContC, fLoopC, fSecond,
            reasonAnsi.GetText(), chooserAnsi.GetText(), fQms);
        sb.Append(buffer);
    }
    String text;
    sb.ToString(text);
    const StringAnsi pathAnsi(path);
    FILE* file = fopen(pathAnsi.GetText(), "wb");
    if (file == nullptr)
    {
        error = String(TEXT("Cannot write trace CSV file."));
        return false;
    }
    const StringAnsi bodyAnsi(text);
    const size_t written = fwrite(bodyAnsi.GetText(), 1, (size_t)bodyAnsi.Length(), file);
    fclose(file);
    if ((int32)written != bodyAnsi.Length())
    {
        error = String(TEXT("Trace CSV write incomplete."));
        return false;
    }
    snprintf(buffer, sizeof(buffer), "queries=%d avg_cost=%.4f switches=%d (%.2f/s) avg_ms=%.3f",
        snap.Count(), costN > 0 ? costSum / (double)costN : 0.0, switches, switchRate,
        snap.Count() > 0 ? msSum / (double)snap.Count() : 0.0);
    summary = String(buffer);
    return true;
}

bool MotionMatchingTraceLog::VerifyReplay(MotionMatchingDatabase* database, int32 maxEntries, String& report)
{
    report = String::Empty;
    if (database == nullptr || !database->IsBaked())
    {
        report = String(TEXT("Replay stopped: database is not baked."));
        return false;
    }
    const Array<MotionMatchingTraceEntry> snap = Snapshot();
    int32 total = snap.Count();
    if (maxEntries > 0 && maxEntries < total)
        total = maxEntries;
    if (total == 0)
    {
        report = String(TEXT("Replay stopped: trace is empty."));
        return false;
    }
    const int32 start = snap.Count() - total;
    int32 matched = 0;
    int32 checkedN = 0;
    int32 skippedN = 0;
    Array<String> mismatchLines;
    char buffer[512];
    for (int32 i = start; i < snap.Count(); i++)
    {
        const MotionMatchingTraceEntry& e = snap[i];
        if (e.Kind != String(TEXT("search")))
        {
            skippedN++;
            continue;
        }
        if (e.Pose.Count() == 0 || e.Trajectory.Count() == 0)
            continue;
        MotionMatchingSearchSettings settings;
        settings.PoseWeight = e.PoseWeight;
        settings.TrajectoryWeight = e.TrajectoryWeight;
        settings.ContinuityWeight = e.ContinuityWeight;
        settings.MinSwitchTime = e.MinSwitchTime;
        settings.CurrentTime = e.CurrentTime;
        settings.SameClipExclusionWindow = e.ExclusionWindow;
        settings.ExpectedTime = e.ExpectedTime;
        settings.ContinuingBias = e.ContinuingBias;
        settings.ContinuingWindow = e.ContinuingWindow;
        settings.SwitchMargin = e.SwitchMargin;
        settings.LoopPreference = e.LoopPreference;
        settings.LoopFilter = (MotionLoopFilter)e.LoopFilter;
        settings.ReselectClip = e.ReselectClip;
        settings.ReselectSampleTime = e.ReselectSampleTime;
        settings.ReselectBanWindow = e.ReselectBanWindow;
        settings.TwinBanWindow = e.TwinBanWindow;
        settings.LoopTailExclusion = e.LoopTailExclusion;
        settings.FootLockL = e.FootLockL;
        settings.FootLockR = e.FootLockR;
        settings.QueryYawRate = e.YawRate;
        Array<String> tags;
        if (!e.Tags.IsEmpty())
            e.Tags.Split('+', tags);
        MotionMatchingResult result;
        String error;
        MotionMatchingSearch::FindBest(database, e.Pose, e.Trajectory, settings,
            e.ContinuityClip, e.SinceSwitch, tags, result, error);
        const bool wantValid = e.Valid;
        bool gotValid = result.IsValid;
        bool same = wantValid == gotValid && (!wantValid || result.SampleIndex == e.SampleIndex);
        if (!same && e.Recovered)
        {
            settings.CurrentTime = -1.0f;
            settings.SameClipExclusionWindow = 0.0f;
            settings.LoopFilter = MotionLoopFilter::LoopOnly;
            MotionMatchingSearch::FindBest(database, e.Pose, e.Trajectory, settings,
                -1, 999.0f, tags, result, error);
            gotValid = result.IsValid;
            same = wantValid == gotValid && (!wantValid || result.SampleIndex == e.SampleIndex);
        }
        checkedN++;
        if (same)
        {
            matched++;
        }
        else if (mismatchLines.Count() < 8)
        {
            const StringAnsi tagsAnsi(e.Tags);
            const StringAnsi reasonAnsi(e.Reason);
            if (wantValid)
                snprintf(buffer, sizeof(buffer), "t=%.2f want=%d ", (double)e.Time, e.SampleIndex);
            else
                snprintf(buffer, sizeof(buffer), "t=%.2f want=invalid ", (double)e.Time);
            String line(buffer);
            if (gotValid)
            {
                snprintf(buffer, sizeof(buffer), "got=%d ", result.SampleIndex);
                line += String(buffer);
            }
            else
            {
                line += String(TEXT("got=invalid "));
            }
            snprintf(buffer, sizeof(buffer), "tags=%s reason=%s", tagsAnsi.GetText(), reasonAnsi.GetText());
            line += String(buffer);
            mismatchLines.Add(line);
        }
    }
    {
        char head[256];
        snprintf(head, sizeof(head), "search replay %d/%d matched", matched, checkedN);
        report = String(head);
    }
    if (skippedN > 0)
    {
        char tail[128];
        snprintf(tail, sizeof(tail), " (%d hold/replay policy events skipped)", skippedN);
        report += String(tail);
    }
    for (int32 m = 0; m < mismatchLines.Count(); m++)
    {
        report += String(TEXT("\n"));
        report += mismatchLines[m];
    }
    return matched == checkedN && checkedN > 0;
}
