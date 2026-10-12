#pragma once

#if USE_EDITOR

#include "Engine/Content/Assets/Animation.h"
#include "Engine/Content/Assets/SkinnedModel.h"
#include "../Schema/PoseSearchSchema.h"

class MotionMatchingDatabase;

/// <summary>
/// Editor-side native motion-matching database baker.
/// </summary>
API_CLASS(Static, Namespace="MotionMatching")
class MOTIONMATCHING_API MotionMatchingBaker
{
    DECLARE_SCRIPTING_TYPE_NO_SPAWN(MotionMatchingBaker);

public:
    /// <summary>
    /// Samples the database source clips, bakes normalized pose and trajectory
    /// features, stores the schema snapshot, then saves the database asset.
    /// The feature layout is declared by the schema (pose: one pos+vel per
    /// PoseBones entry + one flag per ContactBones entry, trajectory: one
    /// planar pos+dir+yawrate point per TrajectorySampleTimes entry).
    /// Schema weights are stored but not applied. The bake FAILS when a
    /// declared node is missing in the model; before sampling it logs a
    /// channel coverage report (clip channels vs model nodes, alias
    /// suggestions) and FAILS when RootBone or a pose bone has no channel
    /// in some clip. The skeleton fingerprint stored with the data
    /// lets a controller refuse a model the clips do not fit.
    /// </summary>
    API_FUNCTION()
    static bool Bake(
        SkinnedModel* model,
        const PoseSearchSchema& schema,
        MotionMatchingDatabase* database,
        API_PARAM(Out) int32& bakedClipCount,
        API_PARAM(Out) int32& bakedSampleCount);

    /// <summary>
    /// Alias suggestions for baking clips on a model: every clip channel that
    /// resolves to no model node (schema aliases applied) paired with the one
    /// undriven node of the same normalized name (case, separators and a
    /// dominant rig prefix ignored, side tokens l/left and r/right unified).
    /// Ambiguous names get no suggestion. Review before adding them to
    /// Schema.BoneAliases: aliases rename only, they never fix proportions.
    /// </summary>
    API_FUNCTION()
    static Array<BoneAlias> SuggestBoneAliases(
        SkinnedModel* model,
        const Array<Animation*>& clips,
        const PoseSearchSchema& schema);

    /// <summary>
    /// The coverage report the bake logs (channels matching no node, nodes
    /// driven by no clip, alias suggestions, missing required roles, clip
    /// proportions that differ from the model), without baking.
    /// </summary>
    API_FUNCTION()
    static String DescribeChannelCoverage(
        SkinnedModel* model,
        const Array<Animation*>& clips,
        const PoseSearchSchema& schema);
};

#endif
