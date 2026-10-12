#pragma once

#include "Engine/Animations/AnimationData.h"
#include "Engine/Core/Collections/Array.h"
#include "Engine/Core/Collections/Dictionary.h"
#include "Engine/Core/Math/Transform.h"
#include "Engine/Core/Types/String.h"
#include "Engine/Graphics/Models/SkeletonData.h"
#include "../Schema/MotionMatchingTypes.h"

/// <summary>
/// The one channel resolver shared by the baker (Baker/MotionMatchingBaker.cpp)
/// and the runtime sampler (Runtime/MotionMatchingSampler.cpp). A clip channel
/// drives the model node whose name equals the channel name, or the alias
/// target when PoseSearchSchema::BoneAliases maps the channel. The first
/// channel that resolves to a node wins (same rule as the old per-node linear
/// scan, so maps built without aliases sample bit-identical poses).
///
/// Build the node-to-channel table once per clip/model pair; sampling then
/// costs one table lookup per node instead of a scan over every channel.
///
/// The name normalization below is used ONLY to suggest aliases in the bake
/// coverage report and the editor; binding never matches fuzzy names.
/// </summary>
namespace MotionMatchingChannelMap
{
    /// <summary>
    /// Model node name driven by a clip channel: the alias target when the
    /// channel is aliased, else the channel name itself.
    /// </summary>
    inline const String& ResolveTarget(const String& channelName, const Array<BoneAlias>& aliases)
    {
        for (const BoneAlias& alias : aliases)
        {
            if (alias.Channel == channelName)
                return alias.Node;
        }
        return channelName;
    }

    /// <summary>
    /// Fills outNodeToChannel (one entry per model node): the index of the
    /// clip channel that drives the node, or -1 when no channel does (the
    /// node keeps its bind local transform).
    /// </summary>
    inline void Build(
        const Array<SkeletonNode>& nodes,
        const Array<NodeAnimationData>& channels,
        const Array<BoneAlias>& aliases,
        Array<int32>& outNodeToChannel)
    {
        Dictionary<String, int32> firstChannelByTarget;
        for (int32 c = 0; c < channels.Count(); c++)
        {
            const String& target = ResolveTarget(channels[c].NodeName, aliases);
            if (!firstChannelByTarget.ContainsKey(target))
                firstChannelByTarget.Add(target, c);
        }
        outNodeToChannel.Resize(nodes.Count());
        for (int32 n = 0; n < nodes.Count(); n++)
        {
            int32 channel = -1;
            if (!firstChannelByTarget.TryGet(nodes[n].Name, channel))
                channel = -1;
            outNodeToChannel[n] = channel;
        }
    }

    /// <summary>
    /// Two channels of one clip that resolve to the same target (an alias
    /// target that is also a raw channel name, or two raw duplicates). Build
    /// keeps the first channel; the later one is ignored.
    /// </summary>
    struct TargetCollision
    {
        String Node;
        String Winner;
        String Shadowed;
    };

    /// <summary>
    /// Lists every channel whose target an earlier channel of the same clip
    /// already drives (same first-wins rule as Build). Reporting only.
    /// </summary>
    inline void FindTargetCollisions(
        const Array<NodeAnimationData>& channels,
        const Array<BoneAlias>& aliases,
        Array<TargetCollision>& outCollisions)
    {
        outCollisions.Clear();
        Dictionary<String, int32> firstChannelByTarget;
        for (int32 c = 0; c < channels.Count(); c++)
        {
            const String& target = ResolveTarget(channels[c].NodeName, aliases);
            int32 winner = -1;
            if (firstChannelByTarget.TryGet(target, winner))
            {
                TargetCollision collision;
                collision.Node = target;
                collision.Winner = channels[winner].NodeName;
                collision.Shadowed = channels[c].NodeName;
                outCollisions.Add(collision);
            }
            else
            {
                firstChannelByTarget.Add(target, c);
            }
        }
    }

    /// <summary>
    /// Local transform of one node at a frame: the bind local transform with
    /// the channel's non-empty sub-tracks evaluated on top (sub-tracks removed
    /// by import or keyframe optimization keep their bind values).
    /// </summary>
    inline void EvaluateLocal(const SkeletonNode& node, const NodeAnimationData* channel, float frame, Transform& outLocal)
    {
        outLocal = node.LocalTransform;
        if (channel == nullptr)
            return;
        if (channel->Position.GetKeyframes().HasItems())
            channel->Position.Evaluate(outLocal.Translation, frame, false);
        if (channel->Rotation.GetKeyframes().HasItems())
            channel->Rotation.Evaluate(outLocal.Orientation, frame, false);
        if (channel->Scale.GetKeyframes().HasItems())
            channel->Scale.Evaluate(outLocal.Scale, frame, false);
    }

    // ---- Alias suggestions (normalized names, never used for binding) ----

    inline bool IsUpperAscii(Char c) { return c >= 'A' && c <= 'Z'; }
    inline bool IsLowerAscii(Char c) { return c >= 'a' && c <= 'z'; }
    inline bool IsDigitAscii(Char c) { return c >= '0' && c <= '9'; }
    inline bool IsAlnumAscii(Char c) { return IsUpperAscii(c) || IsLowerAscii(c) || IsDigitAscii(c); }

    /// <summary>
    /// Splits a bone name into lowercase tokens: at every non-alphanumeric
    /// character, at lower-to-upper case changes ("LeftFoot"), before the last
    /// capital of an upper-case run followed by lower case ("IKFoot" -> ik,
    /// foot) and at letter/digit boundaries ("Spine01"). Numbers drop leading
    /// zeros ("01" -> "1").
    /// </summary>
    inline void Tokenize(const String& name, Array<String>& outTokens)
    {
        outTokens.Clear();
        const int32 length = name.Length();
        String current;
        auto flush = [&]()
        {
            if (current.IsEmpty())
                return;
            if (IsDigitAscii(current[0]))
            {
                int32 start = 0;
                while (start < current.Length() - 1 && current[start] == '0')
                    start++;
                current = current.Substring(start);
            }
            outTokens.Add(current.ToLower());
            current.Clear();
        };
        for (int32 i = 0; i < length; i++)
        {
            const Char c = name[i];
            if (!IsAlnumAscii(c))
            {
                flush();
                continue;
            }
            if (!current.IsEmpty())
            {
                const Char prev = name[i - 1];
                const bool caseBreak = IsUpperAscii(c) && IsLowerAscii(prev);
                const bool runBreak = IsUpperAscii(c) && IsUpperAscii(prev) &&
                    i + 1 < length && IsLowerAscii(name[i + 1]);
                const bool digitBreak = IsDigitAscii(c) != IsDigitAscii(prev);
                if (caseBreak || runBreak || digitBreak)
                    flush();
            }
            current += c;
        }
        flush();
    }

    /// <summary>
    /// Strips rig prefixes shared by most names of one set ("CC_Base_",
    /// "mixamorig:"): while the most common leading token starts at least
    /// half of the names, it is removed from every name that starts with it
    /// and still has another token. At most three passes.
    /// </summary>
    inline void StripDominantPrefix(Array<Array<String>>& tokenSets)
    {
        const int32 total = tokenSets.Count();
        if (total == 0)
            return;
        for (int32 pass = 0; pass < 3; pass++)
        {
            Dictionary<String, int32> counts;
            String best;
            int32 bestCount = 0;
            for (const Array<String>& tokens : tokenSets)
            {
                if (tokens.Count() < 2)
                    continue;
                int32 count = 0;
                counts.TryGet(tokens[0], count);
                count++;
                counts[tokens[0]] = count;
                if (count > bestCount)
                {
                    bestCount = count;
                    best = tokens[0];
                }
            }
            if (bestCount * 2 < total || best.IsEmpty())
                return;
            for (Array<String>& tokens : tokenSets)
            {
                if (tokens.Count() >= 2 && tokens[0] == best)
                    tokens.RemoveAtKeepOrder(0);
            }
        }
    }

    /// <summary>
    /// Comparison key of one tokenized name: side tokens (l/left, r/right)
    /// pulled into a prefix, the remaining tokens joined ("l:foot" for both
    /// "foot_l" and "CC_Base_L_Foot" after prefix stripping).
    /// </summary>
    inline String MakeKey(const Array<String>& tokens)
    {
        String side;
        String rest;
        for (const String& token : tokens)
        {
            if (side.IsEmpty() && (token == TEXT("l") || token == TEXT("left")))
            {
                side = TEXT("l");
                continue;
            }
            if (side.IsEmpty() && (token == TEXT("r") || token == TEXT("right")))
            {
                side = TEXT("r");
                continue;
            }
            if (!rest.IsEmpty())
                rest += TEXT("_");
            rest += token;
        }
        return side + TEXT(":") + rest;
    }

    /// <summary>
    /// Normalized keys for a whole name set (prefix stripping is set-wide).
    /// </summary>
    inline void MakeKeys(const Array<String>& names, Array<String>& outKeys)
    {
        Array<Array<String>> tokenSets;
        tokenSets.Resize(names.Count());
        for (int32 i = 0; i < names.Count(); i++)
            Tokenize(names[i], tokenSets[i]);
        StripDominantPrefix(tokenSets);
        outKeys.Resize(names.Count());
        for (int32 i = 0; i < names.Count(); i++)
            outKeys[i] = MakeKey(tokenSets[i]);
    }

    /// <summary>
    /// Alias suggestions: every channel that resolves to no node is paired
    /// with the single not-yet-driven node that has the same normalized key.
    /// Ambiguous keys (several candidate nodes or channels) are skipped.
    /// </summary>
    inline void SuggestAliases(
        const Array<String>& channelNames,
        const Array<String>& nodeNames,
        const Array<BoneAlias>& aliases,
        Array<BoneAlias>& outSuggestions)
    {
        outSuggestions.Clear();
        Array<String> channelKeys;
        Array<String> nodeKeys;
        MakeKeys(channelNames, channelKeys);
        MakeKeys(nodeNames, nodeKeys);

        // Nodes already driven (exact name or existing alias) are taken.
        Array<bool> nodeTaken;
        nodeTaken.Resize(nodeNames.Count());
        for (int32 n = 0; n < nodeNames.Count(); n++)
            nodeTaken[n] = false;
        Array<bool> channelResolved;
        channelResolved.Resize(channelNames.Count());
        for (int32 c = 0; c < channelNames.Count(); c++)
        {
            const String& target = ResolveTarget(channelNames[c], aliases);
            const int32 node = nodeNames.Find(target);
            channelResolved[c] = node >= 0;
            if (node >= 0)
                nodeTaken[node] = true;
        }

        for (int32 c = 0; c < channelNames.Count(); c++)
        {
            if (channelResolved[c])
                continue;
            int32 match = -1;
            int32 matches = 0;
            for (int32 n = 0; n < nodeNames.Count(); n++)
            {
                if (!nodeTaken[n] && nodeKeys[n] == channelKeys[c])
                {
                    match = n;
                    matches++;
                }
            }
            if (matches != 1)
                continue;
            // Another unresolved channel with the same key makes it ambiguous.
            bool ambiguous = false;
            for (int32 other = 0; other < channelNames.Count() && !ambiguous; other++)
                ambiguous = other != c && !channelResolved[other] && channelKeys[other] == channelKeys[c];
            if (ambiguous)
                continue;
            BoneAlias suggestion;
            suggestion.Channel = channelNames[c];
            suggestion.Node = nodeNames[match];
            outSuggestions.Add(suggestion);
            nodeTaken[match] = true;
        }
    }
}
