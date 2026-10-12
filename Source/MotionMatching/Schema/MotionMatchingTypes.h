#pragma once

#include "Engine/Core/Collections/Array.h"
#include "Engine/Core/Types/String.h"
#include "Engine/Content/Assets/SkinnedModel.h"

/// <summary>
/// One pose feature bone of the schema layout. Every declared bone adds a
/// root-relative position (3 floats) and velocity (3 floats) to the pose
/// block, in declaration order.
/// </summary>
API_STRUCT(Namespace="MotionMatching") struct MOTIONMATCHING_API PoseFeatureBone
{
    DECLARE_SCRIPTING_TYPE_MINIMAL(PoseFeatureBone);

    /// <summary>
    /// Model node name. A declared node that the model does not have is an
    /// error (bake and bind fail); it is never baked as zeros.
    /// </summary>
    API_FIELD() String Node;

    /// <summary>
    /// Optional normalization group label. Bones with the same non-empty
    /// label share one deviation, for positions and velocities separately
    /// (for example "Foot" on both feet, so left/right statistics cannot skew
    /// turns). Empty: the bone normalizes alone. Labels compare exactly.
    /// </summary>
    API_FIELD() String Group;

    bool operator==(const PoseFeatureBone& other) const
    {
        return Node == other.Node && Group == other.Group;
    }
};

/// <summary>
/// One foot-contact feature. The node must also be a pose bone: the planted
/// flag is computed from that pose bone's root-relative height and velocity.
/// Contacts follow the pose velocities in declaration order.
/// </summary>
API_STRUCT(Namespace="MotionMatching") struct MOTIONMATCHING_API ContactFeatureBone
{
    DECLARE_SCRIPTING_TYPE_MINIMAL(ContactFeatureBone);

    /// <summary>Model node name; must equal the Node of one PoseBones entry.</summary>
    API_FIELD() String Node;

    bool operator==(const ContactFeatureBone& other) const
    {
        return Node == other.Node;
    }
};

/// <summary>
/// Model node indices resolved for a schema layout
/// (PoseSearchSchema::TryResolveNodes).
/// </summary>
API_STRUCT(Namespace="MotionMatching") struct MOTIONMATCHING_API NodeIndices
{
    DECLARE_SCRIPTING_TYPE_MINIMAL(NodeIndices);

    /// <summary>Root node: origin and frame of every feature.</summary>
    API_FIELD() int32 Root = -1;

    /// <summary>One model node per Schema.PoseBones entry, in declaration order.</summary>
    API_FIELD() Array<int32> PoseBones;

    /// <summary>
    /// True when the root and every declared pose bone resolved.
    /// </summary>
    bool IsValid() const
    {
        if (Root < 0 || PoseBones.IsEmpty())
            return false;
        for (int32 index : PoseBones)
        {
            if (index < 0)
                return false;
        }
        return true;
    }
};

/// <summary>
/// One bone alias: an animation channel name of the source clips mapped onto
/// a node name of the skinned model the database is baked and played on.
/// Aliases rename only; they do not retarget rest pose or proportions (use an
/// offline retarget tool for that, the plugin refuses mismatched skeletons).
/// </summary>
API_STRUCT(Namespace="MotionMatching") struct MOTIONMATCHING_API BoneAlias
{
    DECLARE_SCRIPTING_TYPE_MINIMAL(BoneAlias);

    /// <summary>Animation channel name as stored in the clips.</summary>
    API_FIELD() String Channel;

    /// <summary>Model skeleton node name the channel drives.</summary>
    API_FIELD() String Node;

    bool operator==(const BoneAlias& other) const
    {
        return Channel == other.Channel && Node == other.Node;
    }
};
