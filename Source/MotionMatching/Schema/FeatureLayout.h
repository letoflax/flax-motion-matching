#pragma once

#include "Engine/Core/Collections/Array.h"
#include "Engine/Core/Types/String.h"
#include "FeatureChannel.h"

/// <summary>
/// Runtime feature layout derived from a PoseSearchSchema declaration
/// (PoseSearchSchema::TryBuildLayout). Single source of truth for feature
/// sizes, offsets and normalization groups: the baker, the query builder,
/// the search and the database all read it instead of compile-time
/// constants.
///
/// Pose block: PoseBoneCount root-relative positions (xyz, declaration
/// order), then PoseBoneCount root-relative velocities (xyz), then
/// ContactCount planted flags (1 = planted). Trajectory block:
/// TrajectoryPointCount points (one per Schema.TrajectorySampleTimes entry)
/// of [posRight, posForward, dirRight, dirForward, yawRate].
/// </summary>
struct MOTIONMATCHING_API FeatureLayout
{
    static constexpr int32 PoseFloatsPerBone = 6;        // position xyz + velocity xyz
    static constexpr int32 TrajectoryFloatsPerPoint = 5; // planar position xy + planar facing xy + yaw rate

    // Declaration limits (validated by PoseSearchSchema::TryBuildLayout).
    // They also bound the stack scratch of the query builder and the baker.
    static constexpr int32 MaxPoseBones = 64;
    static constexpr int32 MaxContacts = 16;
    static constexpr int32 MaxTrajectoryPoints = 32;
    static constexpr int32 MaxPoseFeatureSize = MaxPoseBones * PoseFloatsPerBone + MaxContacts;
    static constexpr int32 MaxTrajectoryFeatureSize = MaxTrajectoryPoints * TrajectoryFloatsPerPoint;

    // Reference biped layout: 7 pose bones, 2 contacts, 6 trajectory points.
    // Nothing assumes it. The search uses it only to select loops
    // specialized for these sizes (same arithmetic, constant trip count);
    // every other layout runs the generic loops.
    static constexpr int32 ReferencePoseFeatureSize = 7 * PoseFloatsPerBone + 2;
    static constexpr int32 ReferenceTrajectoryFeatureSize = 6 * TrajectoryFloatsPerPoint;

    int32 PoseBoneCount = 0;
    int32 ContactCount = 0;
    int32 TrajectoryPointCount = 0;
    int32 PoseFeatureSize = 0;
    int32 TrajectoryFeatureSize = 0;

    /// <summary>Index of the trajectory point at time zero (-1 when unset).</summary>
    int32 ZeroTimePoint = -1;

    /// <summary>Node name per pose bone (declaration order).</summary>
    Array<String> PoseBoneNames;

    /// <summary>Effective group label per pose bone: the declared label, or the node name for a solo bone.</summary>
    Array<String> PoseBoneGroupLabels;

    /// <summary>Pose bone index per contact (declaration order of Schema.ContactBones).</summary>
    Array<int32> ContactPoseBones;

    /// <summary>Normalization group id per pose dimension.</summary>
    Array<int32> PoseGroups;

    bool IsValid() const
    {
        return PoseFeatureSize > 0 && TrajectoryFeatureSize > 0;
    }

    int32 GetPositionBlockSize() const
    {
        return PoseBoneCount * 3;
    }

    int32 GetVelocityOffset(int32 bone) const
    {
        return PoseBoneCount * 3 + bone * 3;
    }

    /// <summary>
    /// Block-local pose offset of a planted flag, or -1 for an invalid
    /// contact index. Bake, query and search resolve contacts through this.
    /// </summary>
    int32 GetContactFeatureOffset(int32 contact) const
    {
        if (contact < 0 || contact >= ContactCount)
            return -1;
        return PoseBoneCount * PoseFloatsPerBone + contact;
    }

    int32 GetPoseNormalizationGroup(int32 dim) const
    {
        return dim >= 0 && dim < PoseGroups.Count() ? PoseGroups[dim] : -1;
    }

    /// <summary>
    /// Trajectory dims stay scalar with identity groups (each dim normalizes
    /// independently).
    /// </summary>
    static int32 GetTrajectoryNormalizationGroup(int32 dim)
    {
        return dim;
    }

    /// <summary>
    /// True for the reference biped sizes (44 pose floats, 30 trajectory floats).
    /// </summary>
    bool HasReferenceSizes() const
    {
        return PoseFeatureSize == ReferencePoseFeatureSize &&
               TrajectoryFeatureSize == ReferenceTrajectoryFeatureSize;
    }

    /// <summary>
    /// Channel metadata. Pose block: one position and one velocity vector
    /// channel per bone (dim 3), one scalar channel per contact. Trajectory
    /// block: one scalar channel per dim (identity groups). Offsets are
    /// block-local.
    /// </summary>
    void DescribePoseChannels(Array<FeatureChannel>& outChannels) const;
    void DescribeTrajectoryChannels(Array<FeatureChannel>& outChannels) const;

    /// <summary>
    /// FNV-1a over the block sizes and the full channel metadata. Stored per
    /// database; a mismatch on load drops the baked arrays (REBAKE REQUIRED).
    /// Never 0 (0 = unknown/legacy).
    /// </summary>
    uint32 ComputeHash() const;

    /// <summary>
    /// Every channel inside its block, no overlaps, full coverage, contact
    /// channels exactly at GetContactFeatureOffset.
    /// </summary>
    bool Validate(String& error) const;

    void Clear();
};
