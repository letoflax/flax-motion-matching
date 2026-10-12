#pragma once

#include "Engine/Core/Collections/Array.h"
#include "Engine/Core/Types/String.h"
#include "../Schema/MotionMatchingTypes.h"

class Animation;
class SkinnedModel;
class ReadStream;
class WriteStream;

/// <summary>
/// Skeleton the baked features were computed on, stored in the database
/// (format v4) so a database never binds silently to a model it does not fit.
///
/// One entry per node of the bake model: name, parent index, a quantized local
/// translation and flags. The translation is the one the clips really drive:
/// the clips' constant channel translation when every position key of every
/// clip stays within ProportionTolerance of the first key, otherwise the bake
/// model's bind translation. Nodes whose translation moves inside a clip or
/// differs between clips (root travel, pelvis bob, a crouch offset) are flagged
/// animated and skipped by the proportion check. Clips authored for another skeleton
/// therefore leave their own proportions in the fingerprint even when aliases
/// made every channel resolve, and the runtime check rejects the pair.
///
/// The check covers only "relevant" nodes: nodes driven by a clip channel and
/// their ancestors. Extra model nodes (mesh helpers, attachments) are ignored.
/// </summary>
struct MOTIONMATCHING_API MotionMatchingSkeletonFingerprint
{
    enum NodeFlag : int32
    {
        Driven = 1,              // some clip channel drives the node
        AnimatedTranslation = 2, // some position key (in or across clips) moves: not a proportion
        Relevant = 4,            // driven, or an ancestor of a driven node
        FromClips = 8,           // translation taken from the clips, not the bind pose
    };

    /// <summary>World units per quantization step of the stored translations.</summary>
    static constexpr float TranslationQuantum = 0.001f;

    /// <summary>
    /// Allowed translation difference, relative to max(bone length, median
    /// bone length of the skeleton). Measured on the retargeted Soldier set:
    /// largest non-animated difference 2.0 percent (spine), every other bone
    /// below 0.4 percent.
    /// </summary>
    static constexpr float ProportionTolerance = 0.05f;

    Array<String> NodeNames;
    Array<int32> ParentIndices;
    Array<int32> Translations; // 3 per node, quantized by TranslationQuantum
    Array<int32> NodeFlags;

    bool IsEmpty() const
    {
        return NodeNames.IsEmpty();
    }

    void Clear();
    bool IsConsistent() const;
    void Serialize(WriteStream& stream) const;
    /// <summary>False (fingerprint cleared) when a list count is out of range or the stream ends early.</summary>
    bool Deserialize(ReadStream& stream);

    /// <summary>64-bit FNV-1a over names, parents, translations and flags (0 when empty).</summary>
    int64 ComputeHash() const;

    /// <summary>
    /// Runtime binding check against a model. False when a relevant node is
    /// missing, has another parent, or its bind translation differs beyond
    /// ProportionTolerance. The report names each mismatch class, the first
    /// offending nodes and the fix. True with an informational report when
    /// the fingerprint is empty (nothing to compare).
    /// </summary>
    bool Check(SkinnedModel* model, String& report) const;

#if USE_EDITOR
    /// <summary>
    /// Builds the fingerprint from the bake model and the clips (aliases
    /// applied through the shared channel resolver). proportionReport lists
    /// nodes whose clip translation differs from the bake model's bind
    /// translation (empty when the clips fit the model).
    /// </summary>
    static bool Build(
        SkinnedModel* model,
        const Array<Animation*>& clips,
        const Array<BoneAlias>& aliases,
        MotionMatchingSkeletonFingerprint& result,
        String& proportionReport);
#endif
};
