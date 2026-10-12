#pragma once

#include "Engine/Core/Types/String.h"

/// <summary>
/// Scalar-vs-vector contract of one pose-search feature channel (Phase 3 N1
/// UE-style). Scalar channels normalize per-dimension (mean + population
/// variance); vector channels share one deviation (mean norm of the centered
/// vector, UE floor 0.1) across their dimensions.
/// </summary>
enum class FeatureChannelKind : int32
{
    Scalar = 0,
    Vector = 1,
};

/// <summary>
/// Data-only description of one pose-search feature channel.
///
/// The native baker and database keep their existing flat-array API. This
/// descriptor is the metadata single source of truth for layout (block,
/// offset, dimension), scalar/vector contract, normalization group,
/// symmetry group, semantic and contact role. See
/// FeatureLayout::DescribePoseChannels/DescribeTrajectoryChannels, built
/// from the schema declaration (PoseSearchSchema::TryBuildLayout).
/// </summary>
struct FeatureChannel
{
    String Name;
    /// <summary>Feature block: 0 = pose, 1 = trajectory.</summary>
    int32 Block = 0;
    /// <summary>Block-local offset of the first float of this channel.</summary>
    int32 Offset = 0;
    int32 Dimension = 0;
    float Weight = 1.0f;
    /// <summary>
    /// Normalization group: dimensions sharing a group share one deviation.
    /// Pose bones with the same declared group label share a group (positions
    /// and velocities separately); all contacts share one group; trajectory
    /// groups are per-dimension identity (scalar contract).
    /// See FeatureLayout::GetPoseNormalizationGroup.
    /// </summary>
    int32 NormalizationGroup = 0;
    /// <summary>Scalar/vector contract (N1 UE-style).</summary>
    FeatureChannelKind Kind = FeatureChannelKind::Scalar;
    /// <summary>
    /// Symmetry group. Only channels of the same semantic ever share a
    /// symmetry group (mirrored limbs must not skew turns).
    /// </summary>
    int32 SymmetryGroup = 0;
    /// <summary>Semantic tag, e.g. "position.Foot", "velocity.CC_Base_Hip", "contact".</summary>
    String Semantic;
    /// <summary>Contact index (declaration order of Schema.ContactBones), -1 for non-contact channels.</summary>
    int32 ContactIndex = -1;
};
