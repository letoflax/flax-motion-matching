#include "MotionMatchingSkeletonFingerprint.h"

#include "../Runtime/MotionMatchingChannelMap.h"
#include "Engine/Content/Assets/Animation.h"
#include "Engine/Content/Assets/SkinnedModel.h"
#include "Engine/Core/Math/Math.h"
#include "Engine/Serialization/ReadStream.h"
#include "Engine/Serialization/WriteStream.h"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr float MinimumBoneLength = 0.0001f;
    constexpr int32 MaxListedNodes = 8;
    constexpr int32 MaxFingerprintNodes = 65536;

    int32 Quantize(float value)
    {
        const double steps = std::round(static_cast<double>(value) / MotionMatchingSkeletonFingerprint::TranslationQuantum);
        if (!(steps > -2147483000.0))
            return -2147483000;
        if (!(steps < 2147483000.0))
            return 2147483000;
        return static_cast<int32>(steps);
    }

    float Dequantize(int32 value)
    {
        return static_cast<float>(value) * MotionMatchingSkeletonFingerprint::TranslationQuantum;
    }

    float Median(Array<float>& values)
    {
        if (values.IsEmpty())
            return 1.0f;
        std::sort(values.Get(), values.Get() + values.Count());
        return values[values.Count() / 2];
    }

    void AppendList(String& report, const Char* label, const Array<String>& items)
    {
        if (items.IsEmpty())
            return;
        report += String::Format(TEXT("; {} {}: "), items.Count(), label);
        for (int32 i = 0; i < items.Count() && i < MaxListedNodes; i++)
        {
            if (i > 0)
                report += TEXT(", ");
            report += items[i];
        }
        if (items.Count() > MaxListedNodes)
            report += String::Format(TEXT(" (+{} more)"), items.Count() - MaxListedNodes);
    }
}

void MotionMatchingSkeletonFingerprint::Clear()
{
    NodeNames.Clear();
    ParentIndices.Clear();
    Translations.Clear();
    NodeFlags.Clear();
}

bool MotionMatchingSkeletonFingerprint::IsConsistent() const
{
    const int32 count = NodeNames.Count();
    if (ParentIndices.Count() != count || NodeFlags.Count() != count || Translations.Count() != count * 3)
        return false;
    for (int32 i = 0; i < count; i++)
    {
        if (ParentIndices[i] < -1 || ParentIndices[i] >= count || ParentIndices[i] == i)
            return false;
    }
    return true;
}

void MotionMatchingSkeletonFingerprint::Serialize(WriteStream& stream) const
{
    stream.Write(NodeNames);
    stream.Write(ParentIndices);
    stream.Write(Translations);
    stream.Write(NodeFlags);
}

namespace
{
    // Same byte layout as ReadStream::Read(Array<T>) (int32 count, then the
    // items), but a count outside [0, limit] or past the end of the stream
    // fails the read instead of allocating it.
    bool ReadBoundedInts(ReadStream& stream, Array<int32>& out, int32 limit)
    {
        out.Clear();
        int32 count = 0;
        stream.ReadInt32(&count);
        if (stream.HasError() || count < 0 || count > limit)
            return false;
        const uint32 bytes = static_cast<uint32>(count) * sizeof(int32);
        if (stream.GetLength() - stream.GetPosition() < bytes)
            return false;
        out.Resize(count, false);
        if (count > 0)
            stream.ReadBytes(out.Get(), bytes);
        return !stream.HasError();
    }

    bool ReadBoundedStrings(ReadStream& stream, Array<String>& out, int32 limit)
    {
        out.Clear();
        int32 count = 0;
        stream.ReadInt32(&count);
        if (stream.HasError() || count < 0 || count > limit)
            return false;
        // Every string costs at least its 4-byte length prefix.
        if ((stream.GetLength() - stream.GetPosition()) / 4u < static_cast<uint32>(count))
            return false;
        out.Resize(count);
        for (int32 i = 0; i < count && !stream.HasError(); i++)
            stream.Read(out[i]);
        return !stream.HasError();
    }
}

bool MotionMatchingSkeletonFingerprint::Deserialize(ReadStream& stream)
{
    const bool ok =
        ReadBoundedStrings(stream, NodeNames, MaxFingerprintNodes) &&
        ReadBoundedInts(stream, ParentIndices, MaxFingerprintNodes) &&
        ReadBoundedInts(stream, Translations, MaxFingerprintNodes * 3) &&
        ReadBoundedInts(stream, NodeFlags, MaxFingerprintNodes);
    if (!ok)
        Clear();
    return ok;
}

int64 MotionMatchingSkeletonFingerprint::ComputeHash() const
{
    if (IsEmpty())
        return 0;
    uint64 hash = 14695981039346656037ull;
    auto mix = [&hash](const void* data, size_t size)
    {
        const uint8* bytes = static_cast<const uint8*>(data);
        for (size_t i = 0; i < size; i++)
        {
            hash ^= bytes[i];
            hash *= 1099511628211ull;
        }
    };
    const int32 count = NodeNames.Count();
    mix(&count, sizeof(count));
    for (const String& name : NodeNames)
    {
        const int32 length = name.Length();
        mix(&length, sizeof(length));
        if (length > 0)
            mix(name.Get(), sizeof(Char) * static_cast<size_t>(length));
    }
    if (ParentIndices.HasItems())
        mix(ParentIndices.Get(), sizeof(int32) * ParentIndices.Count());
    if (Translations.HasItems())
        mix(Translations.Get(), sizeof(int32) * Translations.Count());
    if (NodeFlags.HasItems())
        mix(NodeFlags.Get(), sizeof(int32) * NodeFlags.Count());
    if (hash == 0ull)
        hash = 1ull;
    return static_cast<int64>(hash);
}

bool MotionMatchingSkeletonFingerprint::Check(SkinnedModel* model, String& report) const
{
    report = String::Empty;
    if (IsEmpty())
    {
        report = TEXT("database has no skeleton fingerprint; skeleton not verified");
        return true;
    }
    if (!IsConsistent())
    {
        report = TEXT("the stored skeleton fingerprint is inconsistent; rebake the database");
        return false;
    }
    if (model == nullptr || model->WaitForLoaded())
    {
        report = TEXT("the character model is not loaded");
        return false;
    }

    const auto& nodes = model->GetNodes();
    const int32 count = NodeNames.Count();
    Array<float> lengths;
    for (int32 i = 0; i < count; i++)
    {
        const Float3 t(Dequantize(Translations[i * 3]), Dequantize(Translations[i * 3 + 1]), Dequantize(Translations[i * 3 + 2]));
        const float length = t.Length();
        if (length > MinimumBoneLength)
            lengths.Add(length);
    }
    const float medianLength = Median(lengths);

    Array<String> missing;
    Array<String> structure;
    Array<String> proportions;
    int32 compared = 0;
    float worst = 0.0f;
    String worstName;
    for (int32 i = 0; i < count; i++)
    {
        if ((NodeFlags[i] & Relevant) == 0)
            continue;
        const String& name = NodeNames[i];
        const int32 modelIndex = model->FindNode(name);
        if (modelIndex < 0 || modelIndex >= nodes.Count())
        {
            missing.Add(name);
            continue;
        }
        const String expectedParent = ParentIndices[i] >= 0 ? NodeNames[ParentIndices[i]] : String::Empty;
        const int32 modelParentIndex = nodes[modelIndex].ParentIndex;
        const String modelParent = modelParentIndex >= 0 && modelParentIndex < nodes.Count()
            ? nodes[modelParentIndex].Name
            : String::Empty;
        if (expectedParent != modelParent)
        {
            structure.Add(String::Format(TEXT("{} (parent '{}', model has '{}')"), name, expectedParent, modelParent));
            continue;
        }
        if ((NodeFlags[i] & AnimatedTranslation) != 0)
            continue;
        const Float3 expected(Dequantize(Translations[i * 3]), Dequantize(Translations[i * 3 + 1]), Dequantize(Translations[i * 3 + 2]));
        const Float3 bind = nodes[modelIndex].LocalTransform.Translation;
        const float scale = Math::Max(expected.Length(), medianLength);
        const float relative = scale > MinimumBoneLength ? (expected - bind).Length() / scale : 0.0f;
        compared++;
        if (relative > worst)
        {
            worst = relative;
            worstName = name;
        }
        if (!(relative <= ProportionTolerance))
            proportions.Add(String::Format(TEXT("{} ({:.1f}%)"), name, relative * 100.0f));
    }

    const bool ok = missing.IsEmpty() && structure.IsEmpty() && proportions.IsEmpty();
    report = String::Format(
        TEXT("skeleton {}: {} relevant nodes, {} translations compared, worst difference {:.2f}% on '{}' (tolerance {:.0f}%)"),
        String(ok ? TEXT("matches") : TEXT("MISMATCH")),
        missing.Count() + structure.Count() + compared,
        compared,
        worst * 100.0f,
        worstName,
        ProportionTolerance * 100.0f);
    AppendList(report, TEXT("nodes missing from the model"), missing);
    AppendList(report, TEXT("nodes with another parent"), structure);
    AppendList(report, TEXT("nodes with other proportions"), proportions);
    if (!ok)
    {
        report += TEXT(". Fix: when only names differ, map them with Schema.BoneAliases (clip channel -> model node) and rebake; "
            "when the hierarchy or proportions differ, retarget the clips offline onto this skeleton "
            "(for example with fbx-animation-retargeter) and rebake on this model");
    }
    return ok;
}

#if USE_EDITOR

bool MotionMatchingSkeletonFingerprint::Build(
    SkinnedModel* model,
    const Array<Animation*>& clips,
    const Array<BoneAlias>& aliases,
    MotionMatchingSkeletonFingerprint& result,
    String& proportionReport)
{
    result.Clear();
    proportionReport = String::Empty;
    if (model == nullptr || model->WaitForLoaded())
        return false;

    const auto& nodes = model->GetNodes();
    const int32 count = nodes.Count();
    if (count <= 0 || count > MaxFingerprintNodes)
        return false;

    Array<int32> flags;
    Array<Float3> firstTranslation;
    Array<float> spread;
    Array<bool> hasTranslation;
    flags.Resize(count);
    firstTranslation.Resize(count);
    spread.Resize(count);
    hasTranslation.Resize(count);
    for (int32 i = 0; i < count; i++)
    {
        flags[i] = 0;
        firstTranslation[i] = Float3::Zero;
        spread[i] = 0.0f;
        hasTranslation[i] = false;
    }

    Array<int32> nodeToChannel;
    for (Animation* clip : clips)
    {
        if (clip == nullptr || clip->WaitForLoaded())
            continue;
        const auto& channels = clip->Data.Channels;
        MotionMatchingChannelMap::Build(nodes, channels, aliases, nodeToChannel);
        for (int32 n = 0; n < count; n++)
        {
            const int32 c = nodeToChannel[n];
            if (c < 0)
                continue;
            flags[n] |= Driven;
            // Spread over every position key of every clip: a translation that
            // moves inside one clip (root motion, a crouch offset) is animated
            // even when all clips start from the same value.
            const auto& keys = channels[c].Position.GetKeyframes();
            if (keys.IsEmpty())
                continue;
            if (!hasTranslation[n])
            {
                firstTranslation[n] = keys[0].Value;
                hasTranslation[n] = true;
            }
            const Float3 reference = firstTranslation[n];
            float nodeSpread = spread[n];
            for (int32 k = 0; k < keys.Count(); k++)
            {
                const Float3 value = keys[k].Value;
                nodeSpread = Math::Max(nodeSpread, (value - reference).Length());
            }
            spread[n] = nodeSpread;
        }
    }

    // Ancestors of driven nodes shape the driven nodes' model-space pose.
    for (int32 n = 0; n < count; n++)
    {
        if ((flags[n] & Driven) == 0)
            continue;
        int32 walk = n;
        int32 guard = 0;
        while (walk >= 0 && walk < count && guard++ < count)
        {
            flags[walk] |= Relevant;
            walk = nodes[walk].ParentIndex;
        }
    }

    Array<float> lengths;
    for (int32 n = 0; n < count; n++)
    {
        const float length = nodes[n].LocalTransform.Translation.Length();
        if (length > MinimumBoneLength)
            lengths.Add(length);
    }
    const float medianLength = Median(lengths);

    Array<String> mismatched;
    float worst = 0.0f;
    String worstName;
    int32 compared = 0;
    result.NodeNames.Resize(count);
    result.ParentIndices.Resize(count);
    result.Translations.Resize(count * 3);
    result.NodeFlags.Resize(count);
    for (int32 n = 0; n < count; n++)
    {
        const Float3 bind = nodes[n].LocalTransform.Translation;
        Float3 stored = bind;
        if (hasTranslation[n])
        {
            const float tolerance = ProportionTolerance * Math::Max(bind.Length(), medianLength);
            if (spread[n] > tolerance)
            {
                flags[n] |= AnimatedTranslation;
            }
            else
            {
                stored = firstTranslation[n];
                flags[n] |= FromClips;
                const float scale = Math::Max(bind.Length(), medianLength);
                const float relative = scale > MinimumBoneLength ? (stored - bind).Length() / scale : 0.0f;
                compared++;
                if (relative > worst)
                {
                    worst = relative;
                    worstName = nodes[n].Name;
                }
                if (!(relative <= ProportionTolerance))
                    mismatched.Add(String::Format(TEXT("{} ({:.1f}%)"), nodes[n].Name, relative * 100.0f));
            }
        }
        const int32 parent = nodes[n].ParentIndex;
        result.NodeNames[n] = nodes[n].Name;
        result.ParentIndices[n] = parent >= 0 && parent < count && parent != n ? parent : -1;
        result.Translations[n * 3] = Quantize(stored.X);
        result.Translations[n * 3 + 1] = Quantize(stored.Y);
        result.Translations[n * 3 + 2] = Quantize(stored.Z);
        result.NodeFlags[n] = flags[n];
    }

    if (mismatched.HasItems())
    {
        proportionReport = String::Format(
            TEXT("{} of {} clip-driven bone translations differ from the bake model's bind pose (worst {:.1f}% on '{}')"),
            mismatched.Count(), compared, worst * 100.0f, worstName);
        AppendList(proportionReport, TEXT("bones"), mismatched);
    }
    return true;
}

#endif

// C++14 (Flax Linux/clang): ODR-used static constexpr members need an out-of-class definition (undefined symbol at load otherwise).
#if __cplusplus < 201703L
constexpr float MotionMatchingSkeletonFingerprint::TranslationQuantum;
constexpr float MotionMatchingSkeletonFingerprint::ProportionTolerance;
#endif
