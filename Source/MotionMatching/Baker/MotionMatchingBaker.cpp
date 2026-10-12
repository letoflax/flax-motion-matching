#include "MotionMatchingBaker.h"

#if USE_EDITOR

#include "../Database/MotionMatchingDatabase.h"
#include "../Database/MotionMatchingSkeletonFingerprint.h"
#include "../Runtime/MotionMatchingChannelMap.h"
#include "../Runtime/MotionMatchingSampler.h"
#include "../Schema/PlanarFrame.h"
#include "Engine/Core/Log.h"
#include "Engine/Core/Math/Math.h"
#include "Engine/Core/Math/Matrix.h"
#include "Engine/Platform/StringUtils.h"

#include <cmath>

namespace
{
    // The feature layout comes from the schema declaration
    // (PoseSearchSchema::TryBuildLayout), shared with the database validator,
    // the live query builder and the search.
    constexpr int32 TrajectoryPointSize = FeatureLayout::TrajectoryFloatsPerPoint;
    constexpr float MinimumDeltaTime = 0.000001f;

    struct ClipBakeInfo
    {
        Animation* Clip;
        int32 SourceIndex;
        int32 SampleStart;
        int32 SampleCount;
        float Length;
        bool Loop;
        Matrix StartRootTransform;
        Matrix StartRootInverse;
        Matrix CycleRootTransform;
    };

    struct SampleBakeInfo
    {
        int32 ClipIndex;
        float Time;
        Matrix RootTransform;
    };

    bool IsFinite(const Float3& value)
    {
        return std::isfinite(value.X) &&
               std::isfinite(value.Y) &&
               std::isfinite(value.Z);
    }

    void IncludeRange(float* minimums, float* maximums, int32 index, float value)
    {
        minimums[index] = Math::Min(minimums[index], value);
        maximums[index] = Math::Max(maximums[index], value);
    }

    void IncludeVectorRange(float* minimums, float* maximums, int32 offset, const Float3& value)
    {
        IncludeRange(minimums, maximums, offset, value.X);
        IncludeRange(minimums, maximums, offset + 1, value.Y);
        IncludeRange(minimums, maximums, offset + 2, value.Z);
    }

    // nodeToChannel comes from MotionMatchingChannelMap::Build (once per
    // clip, schema aliases applied): the same resolver the runtime sampler
    // uses, so bake and playback drive identical nodes.
    bool SampleAnimationModelPose(
        Animation* animation,
        SkinnedModel* model,
        const Array<int32>& nodeToChannel,
        float time,
        Array<Matrix>& nodeTransforms)
    {
        nodeTransforms.Clear();

        if (animation == nullptr || animation->WaitForLoaded() ||
            model == nullptr || model->WaitForLoaded())
        {
            return false;
        }

        const float framesPerSecond = static_cast<float>(animation->Data.FramesPerSecond);
        if (framesPerSecond <= 0.0f)
            return false;

        const auto& nodes = model->GetNodes();
        const auto& channels = animation->Data.Channels;
        const int32 nodeCount = nodes.Count();
        const float sampleFrame = time * framesPerSecond;
        if (nodeToChannel.Count() != nodeCount)
            return false;

        nodeTransforms.Resize(nodeCount);

        Transform local;
        for (int32 nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++)
        {
            const auto& node = nodes[nodeIndex];
            const int32 channelIndex = nodeToChannel[nodeIndex];
            // Sub-tracks removed by import or keyframe optimization keep
            // their bind-pose values (inside EvaluateLocal).
            MotionMatchingChannelMap::EvaluateLocal(
                node,
                channelIndex >= 0 && channelIndex < channels.Count() ? &channels[channelIndex] : nullptr,
                sampleFrame,
                local);

            const Matrix localMatrix = local.GetWorld();
            if (node.ParentIndex >= 0 && node.ParentIndex < nodeIndex)
            {
                Matrix::Multiply(
                    localMatrix,
                    nodeTransforms[node.ParentIndex],
                    nodeTransforms[nodeIndex]);
            }
            else
            {
                nodeTransforms[nodeIndex] = localMatrix;
            }
        }

        return true;
    }

    // ---- Channel coverage (bake report + editor alias suggestions) ----

    struct ChannelCoverage
    {
        int32 ClipCount = 0;
        Array<String> ChannelNames;      // unique raw channel names over all clips
        Array<String> UnmatchedChannels; // channels whose target is no model node
        Array<String> UndrivenNodes;     // model nodes no clip channel drives
        Array<String> MissingRequired;   // required roles no clip drives at all (bake fails)
        Array<String> PartialRequired;   // required roles some clips do not drive (bind pose; report only)
        Array<String> InvalidAliases;    // aliases whose Node is not a model node (typo; bake fails)
        Array<String> UnusedAliases;     // aliases whose Channel is in no clip (report only)
        Array<String> Collisions;        // two channels of one clip driving one node (report only)
        Array<BoneAlias> Suggestions;
    };

    String JoinNames(const Array<String>& names, int32 maxCount)
    {
        String out;
        for (int32 i = 0; i < names.Count() && i < maxCount; i++)
        {
            if (i > 0)
                out += TEXT(", ");
            out += names[i];
        }
        if (names.Count() > maxCount)
            out += String::Format(TEXT(" (+{} more)"), names.Count() - maxCount);
        return out;
    }

    String JoinAliases(const Array<BoneAlias>& aliases, int32 maxCount)
    {
        String out;
        for (int32 i = 0; i < aliases.Count() && i < maxCount; i++)
        {
            if (i > 0)
                out += TEXT(", ");
            out += aliases[i].Channel + TEXT(" -> ") + aliases[i].Node;
        }
        if (aliases.Count() > maxCount)
            out += String::Format(TEXT(" (+{} more)"), aliases.Count() - maxCount);
        return out;
    }

    // Channel/node coverage of every clip against the model through the
    // shared resolver. Required nodes (RootBone and every declared pose
    // bone) must be driven by a channel in at least one clip: a model whose
    // required nodes no clip drives would bake bind-pose features only. A role
    // missing in some clips keeps the bind pose there and is only reported.
    void AnalyzeCoverage(
        SkinnedModel* model,
        const Array<Animation*>& clips,
        const PoseSearchSchema& schema,
        ChannelCoverage& out)
    {
        out = ChannelCoverage();
        if (model == nullptr || model->WaitForLoaded())
            return;
        const auto& nodes = model->GetNodes();
        Array<String> nodeNames;
        nodeNames.Resize(nodes.Count());
        for (int32 n = 0; n < nodes.Count(); n++)
            nodeNames[n] = nodes[n].Name;
        Array<bool> nodeDriven;
        nodeDriven.Resize(nodes.Count());
        for (int32 n = 0; n < nodes.Count(); n++)
            nodeDriven[n] = false;

        Array<String> roleNames;
        Array<String> roleNodes;
        roleNames.Add(String(TEXT("RootBone")));
        roleNodes.Add(schema.RootBone);
        for (int32 i = 0; i < schema.PoseBones.Count(); i++)
        {
            roleNames.Add(String::Format(TEXT("PoseBones[{}]"), i));
            roleNodes.Add(schema.PoseBones[i].Node);
        }
        const int32 roleCount = roleNodes.Count();
        Array<int32> roleIndex;
        Array<int32> roleMissing;
        Array<String> roleFirstClip;
        roleIndex.Resize(roleCount);
        roleMissing.Resize(roleCount);
        roleFirstClip.Resize(roleCount);
        for (int32 r = 0; r < roleCount; r++)
        {
            roleIndex[r] = model->FindNode(roleNodes[r]);
            roleMissing[r] = 0;
        }

        Array<int32> nodeToChannel;
        Array<MotionMatchingChannelMap::TargetCollision> clipCollisions;
        Array<String> collisionKeys;
        Array<int32> collisionClips;
        Array<String> collisionFirstClip;
        for (Animation* clip : clips)
        {
            if (clip == nullptr || clip->WaitForLoaded())
                continue;
            out.ClipCount++;
            const auto& channels = clip->Data.Channels;
            for (const NodeAnimationData& channel : channels)
            {
                if (!out.ChannelNames.Contains(channel.NodeName))
                    out.ChannelNames.Add(channel.NodeName);
            }
            MotionMatchingChannelMap::FindTargetCollisions(channels, schema.BoneAliases, clipCollisions);
            for (const auto& collision : clipCollisions)
            {
                if (!nodeNames.Contains(collision.Node))
                    continue;
                const String key = String::Format(TEXT("node '{}' is driven by channel '{}'; channel '{}' also resolves to it and is ignored"),
                    collision.Node, collision.Winner, collision.Shadowed);
                const int32 k = collisionKeys.Find(key);
                if (k >= 0)
                {
                    collisionClips[k]++;
                    continue;
                }
                collisionKeys.Add(key);
                collisionClips.Add(1);
                collisionFirstClip.Add(StringUtils::GetFileNameWithoutExtension(clip->GetPath()));
            }
            MotionMatchingChannelMap::Build(nodes, channels, schema.BoneAliases, nodeToChannel);
            for (int32 n = 0; n < nodes.Count(); n++)
            {
                if (nodeToChannel[n] >= 0)
                    nodeDriven[n] = true;
            }
            for (int32 r = 0; r < roleCount; r++)
            {
                if (roleIndex[r] >= 0 && nodeToChannel[roleIndex[r]] >= 0)
                    continue;
                if (roleMissing[r]++ == 0)
                    roleFirstClip[r] = StringUtils::GetFileNameWithoutExtension(clip->GetPath());
            }
        }

        for (const String& channel : out.ChannelNames)
        {
            const String& target = MotionMatchingChannelMap::ResolveTarget(channel, schema.BoneAliases);
            if (!nodeNames.Contains(target))
                out.UnmatchedChannels.Add(channel);
        }
        for (int32 n = 0; n < nodes.Count(); n++)
        {
            if (!nodeDriven[n])
                out.UndrivenNodes.Add(nodeNames[n]);
        }
        for (int32 r = 0; r < roleCount; r++)
        {
            if (roleMissing[r] == 0)
                continue;
            // A clip may legitimately lack a channel (constant tracks are dropped
            // on import and keep the bind pose). Only a role that NO clip drives
            // means the names do not match the model.
            Array<String>& target = roleMissing[r] >= out.ClipCount ? out.MissingRequired : out.PartialRequired;
            target.Add(String::Format(
                TEXT("{} '{}' has no channel in {} of {} clips (first: {})"),
                roleNames[r], roleNodes[r], roleMissing[r], out.ClipCount, roleFirstClip[r]));
        }
        for (int32 k = 0; k < collisionKeys.Count(); k++)
        {
            out.Collisions.Add(String::Format(TEXT("{} in {} of {} clips (first: {})"),
                collisionKeys[k], collisionClips[k], out.ClipCount, collisionFirstClip[k]));
        }
        for (const BoneAlias& alias : schema.BoneAliases)
        {
            if (!nodeNames.Contains(alias.Node))
            {
                out.InvalidAliases.Add(String::Format(TEXT("alias '{}' -> '{}': the model has no node '{}'"),
                    alias.Channel, alias.Node, alias.Node));
            }
            if (!out.ChannelNames.Contains(alias.Channel))
            {
                out.UnusedAliases.Add(String::Format(TEXT("alias '{}' -> '{}': no clip has a channel '{}'"),
                    alias.Channel, alias.Node, alias.Channel));
            }
        }
        MotionMatchingChannelMap::SuggestAliases(out.ChannelNames, nodeNames, schema.BoneAliases, out.Suggestions);
    }

    String DescribeCoverage(const ChannelCoverage& coverage)
    {
        String text = String::Format(
            TEXT("Bake channel coverage: {} clips, {} channel names, {} match no model node, {} model nodes driven by no clip"),
            coverage.ClipCount,
            coverage.ChannelNames.Count(),
            coverage.UnmatchedChannels.Count(),
            coverage.UndrivenNodes.Count());
        if (coverage.UnmatchedChannels.HasItems())
            text += String(TEXT("\n  channels matching no node: ")) + JoinNames(coverage.UnmatchedChannels, 16);
        if (coverage.UndrivenNodes.HasItems())
            text += String(TEXT("\n  model nodes driven by no clip (keep the bind pose): ")) + JoinNames(coverage.UndrivenNodes, 16);
        if (coverage.Suggestions.HasItems())
            text += String(TEXT("\n  suggested Schema.BoneAliases (channel -> node, by normalized name): ")) + JoinAliases(coverage.Suggestions, 24);
        for (const String& line : coverage.PartialRequired)
            text += String(TEXT("\n  NOTE (bind pose is used in those clips): ")) + line;
        for (const String& line : coverage.UnusedAliases)
            text += String(TEXT("\n  NOTE (unused alias): ")) + line;
        for (const String& line : coverage.Collisions)
            text += String(TEXT("\n  NOTE (channel collision, the first channel wins): ")) + line;
        for (const String& line : coverage.InvalidAliases)
            text += String(TEXT("\n  ALIAS ERROR: ")) + line;
        for (const String& line : coverage.MissingRequired)
            text += String(TEXT("\n  REQUIRED: ")) + line;
        return text;
    }

    Float3 GetRelativePosition(const Matrix& transform, const Matrix& referenceInverse)
    {
        return Float3::Transform(transform.GetTranslation(), referenceInverse);
    }

    Float3 GetModelDirection(const Matrix& transform, const Float3& localDirection)
    {
        Float3 result;
        Float3::TransformNormal(localDirection, transform, result);
        if (result.LengthSquared() > MinimumDeltaTime)
            result.Normalize();
        return result;
    }

    Matrix MultiplyMatrices(const Matrix& left, const Matrix& right)
    {
        Matrix result;
        Matrix::Multiply(left, right, result);
        return result;
    }

    Matrix MatrixPower(Matrix value, int32 exponent)
    {
        if (exponent < 0)
        {
            value = Matrix::Invert(value);
            exponent = -exponent;
        }

        Matrix result = Matrix::Identity;
        while (exponent > 0)
        {
            if ((exponent & 1) != 0)
                result = MultiplyMatrices(result, value);
            exponent >>= 1;
            if (exponent > 0)
                value = MultiplyMatrices(value, value);
        }
        return result;
    }

    // Model-space ground plane for the yaw-rate channel and the database's
    // turn/travel metadata: the image of the schema up axis under the root's
    // bind pose (local * parent, same composition as the sampled poses),
    // snapped to a cardinal axis. A Y-up model gives exactly (0, 1, 0).
    Float3 ComputePlanarUpAxis(SkinnedModel* model, int32 rootIndex, const Float3& rootUpAxis)
    {
        const auto& nodes = model->GetNodes();
        Array<int32> chain;
        for (int32 n = rootIndex; n >= 0 && n < nodes.Count() && chain.Count() < nodes.Count(); n = nodes[n].ParentIndex)
            chain.Add(n);
        Matrix world = Matrix::Identity;
        for (int32 i = chain.Count() - 1; i >= 0; i--)
            world = MultiplyMatrices(nodes[chain[i]].LocalTransform.GetWorld(), world);
        Float3 up;
        Float3::TransformNormal(rootUpAxis, world, up);
        return PlanarFrame::SnapAxis(up);
    }

    // T1: loop wrap lives in MotionMatchingSampler::WrapLoopTime (single
    // definition shared with the live query side); the baker only adds the
    // cycle-transform application on top. t±dt stencil sides each resolve
    // their own cycle independently, so seam neighbors come from the adjacent
    // cycle with the matching CycleRootTransform power — never from a
    // duplicated endpoint.
    Matrix GetRootAtTimeWithStatus(
        const ClipBakeInfo& clip,
        const Array<SampleBakeInfo>& samples,
        float time,
        int32 sampleRate,
        MotionSamplingStatus& outStatus)
    {
        int32 localIndex = -1;
        outStatus = MotionMatchingSampler::TimeToSampleIndex(
            time, clip.Length, clip.SampleCount, sampleRate, clip.Loop, localIndex);
        if (outStatus == MotionSamplingStatus::Invalid || localIndex < 0)
            return clip.StartRootTransform;
        if (!clip.Loop)
            return samples[clip.SampleStart + localIndex].RootTransform;

        int32 cycleIndex = 0;
        MotionMatchingSampler::WrapLoopTime(time, clip.Length, cycleIndex);
        const Matrix rootDelta = MultiplyMatrices(
            samples[clip.SampleStart + localIndex].RootTransform,
            clip.StartRootInverse);
        const Matrix accumulatedCycle = MatrixPower(clip.CycleRootTransform, cycleIndex);
        return MultiplyMatrices(
            MultiplyMatrices(rootDelta, accumulatedCycle),
            clip.StartRootTransform);
    }

    Matrix GetRootAtTime(
        const ClipBakeInfo& clip,
        const Array<SampleBakeInfo>& samples,
        float time,
        int32 sampleRate)
    {
        // Clamped-for-playback wrapper: keeps the endpoint pose for display.
        // Feature code (velocity/trajectory/yaw) must use
        // GetRootAtTimeWithStatus + MotionMatchingSampler validity helpers so
        // a clamped non-loop tail never reads as a stopped future (§18.6).
        MotionSamplingStatus status = MotionSamplingStatus::Invalid;
        return GetRootAtTimeWithStatus(clip, samples, time, sampleRate, status);
    }

    // N1b UE-style deviation (NormalizationRevision 3). Baked features stay
    // RAW; only a per-dimension deviation is computed here and stored. One
    // formula for every normalization group regardless of cardinality:
    // - mu[d] = column mean (centroid);
    // - center every column;
    // - deviation = MEAN OVER SAMPLES OF THE CENTERED ROW NORM (mean absolute
    //   deviation for a cardinality-1 scalar group — there is NO separate
    //   scalar path);
    // - floor contract: dev > 0.1f ? dev : 1.0f (NOT max(dev, 0.1)); a true
    //   0.05 deviation divides by 1.0.
    // Symmetry safety: only channels of one declared group label share a
    // group, positions and velocities separately, and all contacts share one
    // (FeatureLayout); trajectory scalars stay per-dimension (identity). Weights are applied once at query/cost time
    // (sqrt of sum-normalized weight, divided by this deviation), never
    // baked, so retunes never need a rebake. Returns the count of dimensions
    // whose PRE-FLOOR deviation sat at/below the unit floor (log-only).
    int32 ComputeGroupDeviations(
        const Array<float>& features,
        int32 sampleCount,
        int32 featureCount,
        const Array<int32>& groupOf,
        Array<float>& deviations)
    {
        constexpr double DeviationFloor = 0.1;
        deviations.Resize(featureCount);
        int32 constantFeatureCount = 0;

        // Collect distinct groups in first-seen order (stable logs).
        Array<int32> groups;
        for (int32 dim = 0; dim < featureCount; dim++)
        {
            const int32 g = groupOf[dim];
            if (!groups.Contains(g))
                groups.Add(g);
        }

        Array<int32> groupDims;
        Array<double> centroid;
        for (int32 gi = 0; gi < groups.Count(); gi++)
        {
            const int32 group = groups[gi];
            groupDims.Clear();
            for (int32 dim = 0; dim < featureCount; dim++)
            {
                if (groupOf[dim] == group)
                    groupDims.Add(dim);
            }
            if (groupDims.IsEmpty() || sampleCount <= 0)
                continue;

            const int32 dimCount = groupDims.Count();
            centroid.Resize(dimCount);
            for (int32 i = 0; i < dimCount; i++)
            {
                const int32 dim = groupDims[i];
                double sum = 0.0;
                for (int32 sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++)
                    sum += static_cast<double>(features[sampleIndex * featureCount + dim]);
                centroid[i] = sum / static_cast<double>(sampleCount);
                if (!std::isfinite(centroid[i]))
                    centroid[i] = 0.0;
            }
            double normSum = 0.0;
            for (int32 sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++)
            {
                double squaredNorm = 0.0;
                for (int32 i = 0; i < dimCount; i++)
                {
                    const int32 dim = groupDims[i];
                    const double centered = static_cast<double>(
                        features[sampleIndex * featureCount + dim]) - centroid[i];
                    squaredNorm += centered * centered;
                }
                normSum += std::sqrt(squaredNorm > 0.0 ? squaredNorm : 0.0);
            }
            const double preFloorDeviation = normSum / static_cast<double>(sampleCount);
            double deviation = preFloorDeviation;
            // !(dev > floor) also catches NaN/non-finite: never divide by
            // zero and never propagate a bad stat into the database.
            if (!(deviation > DeviationFloor))
                deviation = 1.0;
            if (!(preFloorDeviation > DeviationFloor))
                constantFeatureCount += dimCount;
            for (int32 i = 0; i < dimCount; i++)
                deviations[groupDims[i]] = static_cast<float>(deviation);
        }

        return constantFeatureCount;
    }
}

bool MotionMatchingBaker::Bake(
    SkinnedModel* model,
    const PoseSearchSchema& schema,
    MotionMatchingDatabase* database,
    int32& bakedClipCount,
    int32& bakedSampleCount)
{
    bakedClipCount = 0;
    bakedSampleCount = 0;

    if (database == nullptr || database->WaitForLoaded())
    {
        LOG(Error, "Motion matching bake failed: database is not loaded.");
        return false;
    }

    Array<Animation*> animations;
    Array<Array<String>> tags;
    Array<int32> loopingClips;
    Array<int32> clipGroups;
    database->GetSourceData(animations, tags, loopingClips, clipGroups);

    Array<int32> sourceClipIndices;
    Array<float> clipLengths;
    Array<int32> clipSampleCounts;
    Array<int32> sampleClipIndices;
    Array<float> sampleTimes;
    Array<Float3> rootPositions;
    Array<Float3> rootVelocities;
    Array<Float3> rootForwards;
    Array<float> poseFeatures;
    Array<float> trajectoryFeatures;
    Array<float> poseFeatureDeviations;
    Array<float> trajectoryFeatureDeviations;

    if (model == nullptr || model->WaitForLoaded())
    {
        LOG(Error, "Motion matching bake failed: player model is not loaded.");
        return false;
    }
    String schemaError;
    if (!schema.TryValidate(schemaError))
    {
        LOG(Error, "Motion matching bake failed: invalid schema. {}", schemaError);
        return false;
    }
    if (animations.Count() != tags.Count() ||
        animations.Count() != loopingClips.Count() ||
        animations.Count() != clipGroups.Count())
    {
        LOG(Error, "Motion matching bake failed: animation, tag, loop-flag and group counts differ.");
        return false;
    }

    // Snapshot the schema with normalized axes so the stored contract matches what was baked.
    PoseSearchSchema bakedSchema = schema;
    bakedSchema.RootForwardAxis.Normalize();
    bakedSchema.RootUpAxis.Normalize();

    FeatureLayout layout;
    String layoutError;
    if (!bakedSchema.TryBuildLayout(layout, layoutError))
    {
        LOG(Error, "Motion matching bake failed: invalid feature layout. {}", layoutError);
        return false;
    }
    const int32 poseBoneCount = layout.PoseBoneCount;
    const int32 contactCount = layout.ContactCount;
    const int32 trajectoryPointCount = layout.TrajectoryPointCount;
    const int32 positionFeatureSize = layout.GetPositionBlockSize();
    const int32 poseFeatureSize = layout.PoseFeatureSize;
    const int32 trajectoryFeatureSize = layout.TrajectoryFeatureSize;
    const int32 sampleRate = bakedSchema.SampleRate;
    const Array<float>& schemaTrajectorySampleTimes = bakedSchema.TrajectorySampleTimes;
    const Float3 rootForwardAxis = bakedSchema.RootForwardAxis;
    // C1: normalized root-up axis from the schema snapshot (manifest records
    // the raw axis; bake uses the normalized copy). All contact heights go
    // through PoseSearchSchema::ComputeContactHeight with this axis.
    const Float3 rootUpAxis = bakedSchema.RootUpAxis;
    Float3 localRight;
    if (!bakedSchema.TryGetRightAxis(localRight))
    {
        LOG(Error, "Motion matching bake failed: root forward and up axes cannot be parallel.");
        return false;
    }

    NodeIndices nodeIndices;
    String skeletonError;
    if (!bakedSchema.TryResolveNodes(model, nodeIndices, skeletonError))
    {
        LOG(Error, "Motion matching bake failed: {}", skeletonError);
        return false;
    }

    const int32 rootIndex = nodeIndices.Root;
    const Array<int32>& poseBoneIndices = nodeIndices.PoseBones;
    const Float3 planarUpAxis = ComputePlanarUpAxis(model, rootIndex, rootUpAxis);
    const bool planarYUp = PlanarFrame::IsYUp(planarUpAxis);
    const float contactMaxSpeed = bakedSchema.ContactMaxSpeed;
    const float contactMaxHeight = bakedSchema.ContactMaxHeight;

    // Channel coverage through the shared resolver (aliases applied). The
    // bake stops when a required role has no channel in ANY clip (names do
    // not match the model); partial gaps keep the bind pose and are reported; the
    // report suggests aliases by normalized name.
    {
        ChannelCoverage coverage;
        AnalyzeCoverage(model, animations, bakedSchema, coverage);
        const String coverageText = DescribeCoverage(coverage);
        if (coverage.InvalidAliases.HasItems())
        {
            LOG(Error, "{}", coverageText);
            LOG(Error, "Motion matching bake failed: Schema.BoneAliases names model nodes that do not exist (see ALIAS ERROR above). "
                "Fix the Node names of those aliases and bake again.");
            return false;
        }
        if (coverage.MissingRequired.HasItems())
        {
            LOG(Error, "{}", coverageText);
            LOG(Error, "Motion matching bake failed: required skeleton nodes have no animation channel (see coverage above). "
                "Map clip channel names onto model nodes with Schema.BoneAliases (suggestions above), or retarget the clips "
                "offline onto this skeleton and bake again.");
            return false;
        }
        LOG(Info, "{}", coverageText);
    }

    // Skeleton fingerprint: what the features were baked on, checked again
    // when a controller binds the database to a model.
    MotionMatchingSkeletonFingerprint skeletonFingerprint;
    {
        String proportionReport;
        if (!MotionMatchingSkeletonFingerprint::Build(model, animations, bakedSchema.BoneAliases, skeletonFingerprint, proportionReport))
        {
            LOG(Error, "Motion matching bake failed: cannot build the skeleton fingerprint of the model.");
            return false;
        }
        if (proportionReport.HasChars())
        {
            LOG(Warning, "Motion matching bake: the clips do not fit this model's proportions: {}. The baked features keep the "
                "clip proportions, and a controller will refuse to bind this database to a model with this bind pose "
                "(Schema.BoneAliases only rename). Retarget the clips offline onto this skeleton and bake again.", proportionReport);
        }
        int32 relevant = 0;
        for (int32 flags : skeletonFingerprint.NodeFlags)
        {
            if ((flags & MotionMatchingSkeletonFingerprint::Relevant) != 0)
                relevant++;
        }
        LOG(Info, "Skeleton fingerprint: {} nodes, {} relevant (clip-driven or ancestors), hash {}.",
            skeletonFingerprint.NodeNames.Count(), relevant, skeletonFingerprint.ComputeHash());
    }

    {
        String bones;
        for (int32 i = 0; i < poseBoneCount; i++)
        {
            if (i > 0)
                bones += TEXT(", ");
            bones += layout.PoseBoneNames[i];
            if (bakedSchema.PoseBones[i].Group.HasChars())
                bones += String(TEXT(" [")) + bakedSchema.PoseBones[i].Group + TEXT("]");
        }
        String contacts;
        for (int32 c = 0; c < contactCount; c++)
        {
            if (c > 0)
                contacts += TEXT(", ");
            contacts += layout.PoseBoneNames[layout.ContactPoseBones[c]];
        }
        LOG(
            Info,
            "Bake schema: sampleRate={}, forward=({:.2f}, {:.2f}, {:.2f}), up=({:.2f}, {:.2f}, {:.2f}), root='{}', "
            "planar up (model space)=({:.2f}, {:.2f}, {:.2f}).",
            sampleRate,
            rootForwardAxis.X, rootForwardAxis.Y, rootForwardAxis.Z,
            bakedSchema.RootUpAxis.X, bakedSchema.RootUpAxis.Y, bakedSchema.RootUpAxis.Z,
            bakedSchema.RootBone,
            planarUpAxis.X, planarUpAxis.Y, planarUpAxis.Z);
        LOG(
            Info,
            "Feature layout: {} pose bones [{}], {} contacts [{}], {} trajectory points; pose={} floats, trajectory={} floats, layout hash {}.",
            poseBoneCount, bones, contactCount, contacts, trajectoryPointCount,
            poseFeatureSize, trajectoryFeatureSize, layout.ComputeHash());
    }
    // Design note: weights are stored for the runtime cost function only. They are
    // intentionally NOT multiplied into the baked features, so changing them later
    // does not require a re-bake.
    LOG(
        Info,
        "Schema weights stored (not applied at bake time): pose={:.2f}, trajectory={:.2f}.",
        bakedSchema.PoseWeight,
        bakedSchema.TrajectoryWeight);

    Array<ClipBakeInfo> clips;
    int32 totalSampleCount = 0;
    for (int32 sourceIndex = 0; sourceIndex < animations.Count(); sourceIndex++)
    {
        Animation* animation = animations[sourceIndex];
        if (animation == nullptr || animation->WaitForLoaded())
        {
            LOG(Warning, "Skipping null or unloaded animation at source index {}.", sourceIndex);
            continue;
        }

        // T1 sampling contract: count = ceil(length*rate), duration = true clip
        // length, time = min(index/rate, length). count/rate >= length; the
        // last interval may be short. Same helpers as the live query side.
        const float length = animation->GetLength();
        const int32 count = MotionMatchingSampler::ComputeSampleCount(length, sampleRate);
        if (length <= 0.0f || count <= 0)
        {
            LOG(Warning, "Skipping zero-length animation at source index {}.", sourceIndex);
            continue;
        }

        ClipBakeInfo info;
        info.Clip = animation;
        info.SourceIndex = sourceIndex;
        info.SampleStart = totalSampleCount;
        info.SampleCount = count;
        info.Length = length;
        info.Loop = loopingClips[sourceIndex] != 0;
        clips.Add(info);
        totalSampleCount += count;

        sourceClipIndices.Add(sourceIndex);
        clipLengths.Add(length);
        clipSampleCounts.Add(count);
        LOG(
            Info,
            "Added clip '{}' for tags '{}' with length {}, sample count {}, loop={}, group={}.",
            StringUtils::GetFileNameWithoutExtension(animation->GetPath()),
            MotionMatchingDatabase::JoinTags(tags[sourceIndex]),
            length,
            count,
            info.Loop,
            clipGroups[sourceIndex]);
    }

    if (clips.IsEmpty() || totalSampleCount <= 0)
    {
        LOG(Error, "Motion matching bake failed: no valid animation samples.");
        return false;
    }

    LOG(Info, "Motion clips counted: {}", clips.Count());
    LOG(Info, "Motion samples generated: {}", totalSampleCount);
    LOG(
        Info,
        "Sampling contract T1: count=ceil(length*rate), time=min(index/rate,length), "
        "duration=true clip length; loop wraps [0,length), non-loop endpoints one-sided "
        "with Valid/OutsideNonLoop/Invalid status (PLAN Phase 2 §3, §18.6).");

    Array<SampleBakeInfo> samples;
    samples.Resize(totalSampleCount);
    // Root-relative pose bone positions, poseBoneCount per sample.
    Array<Float3> samplePositions;
    samplePositions.Resize(totalSampleCount * poseBoneCount);
    sampleClipIndices.Resize(totalSampleCount);
    sampleTimes.Resize(totalSampleCount);
    rootPositions.Resize(totalSampleCount);
    rootVelocities.Resize(totalSampleCount);
    rootForwards.Resize(totalSampleCount);
    poseFeatures.Resize(totalSampleCount * poseFeatureSize);
    trajectoryFeatures.Resize(totalSampleCount * trajectoryFeatureSize);

    Array<float> featureMinimums;
    Array<float> featureMaximums;
    featureMinimums.Resize(poseFeatureSize);
    featureMaximums.Resize(poseFeatureSize);
    float rootPositionMinimums[3] = { MAX_float, MAX_float, MAX_float };
    float rootPositionMaximums[3] = { -MAX_float, -MAX_float, -MAX_float };
    float rootVelocityMinimums[3] = { MAX_float, MAX_float, MAX_float };
    float rootVelocityMaximums[3] = { -MAX_float, -MAX_float, -MAX_float };
    Array<float> trajectoryMinimums;
    Array<float> trajectoryMaximums;
    trajectoryMinimums.Resize(trajectoryFeatureSize);
    trajectoryMaximums.Resize(trajectoryFeatureSize);
    for (int32 i = 0; i < poseFeatureSize; i++)
    {
        featureMinimums[i] = MAX_float;
        featureMaximums[i] = -MAX_float;
    }
    for (int32 i = 0; i < trajectoryFeatureSize; i++)
    {
        trajectoryMinimums[i] = MAX_float;
        trajectoryMaximums[i] = -MAX_float;
    }

    Array<Matrix> pose;
    Array<int32> nodeToChannel;
    for (int32 clipIndex = 0; clipIndex < clips.Count(); clipIndex++)
    {
        ClipBakeInfo& clip = clips[clipIndex];
        // One node-to-channel table per clip (shared resolver, aliases applied).
        MotionMatchingChannelMap::Build(model->GetNodes(), clip.Clip->Data.Channels, bakedSchema.BoneAliases, nodeToChannel);
        for (int32 localSampleIndex = 0; localSampleIndex < clip.SampleCount; localSampleIndex++)
        {
            const int32 sampleIndex = clip.SampleStart + localSampleIndex;
            const float time = MotionMatchingSampler::ComputeSampleTime(
                localSampleIndex, clip.Length, sampleRate);

            if (!SampleAnimationModelPose(clip.Clip, model, nodeToChannel, time, pose))
            {
                LOG(
                    Error,
                    "Motion matching bake failed while sampling clip {} at {:.4f}s.",
                    clipIndex,
                    time);
                return false;
            }

            const int32 poseCount = pose.Count();
            if (rootIndex < 0 || rootIndex >= poseCount)
            {
                LOG(Error, "Motion matching bake failed: root node index is outside pose array.");
                return false;
            }

            SampleBakeInfo& sample = samples[sampleIndex];
            sample.ClipIndex = clipIndex;
            sample.Time = time;
            sample.RootTransform = pose[rootIndex];
            const Matrix rootInverse = Matrix::Invert(sample.RootTransform);

            Matrix relative;
            Float3* positions = &samplePositions[sampleIndex * poseBoneCount];
            for (int32 boneIndex = 0; boneIndex < poseBoneCount; boneIndex++)
            {
                // Every declared bone resolved (TryResolveNodes): a missing
                // node fails the bake, it is never baked as zeros.
                const int32 nodeIndex = poseBoneIndices[boneIndex];
                if (nodeIndex < 0 || nodeIndex >= poseCount)
                {
                    LOG(Error, "Motion matching bake failed: pose bone '{}' is outside the pose array.", layout.PoseBoneNames[boneIndex]);
                    return false;
                }
                Matrix::Multiply(pose[nodeIndex], rootInverse, relative);
                positions[boneIndex] = relative.GetTranslation();
                if (!IsFinite(positions[boneIndex]))
                {
                    LOG(Error, "Motion matching bake failed: non-finite position at sample {}.", sampleIndex);
                    return false;
                }
            }

            sampleClipIndices[sampleIndex] = clipIndex;
            sampleTimes[sampleIndex] = time;
            rootPositions[sampleIndex] = sample.RootTransform.GetTranslation();
            rootForwards[sampleIndex] = GetModelDirection(sample.RootTransform, rootForwardAxis);
            if (!IsFinite(rootPositions[sampleIndex]) ||
                !IsFinite(rootForwards[sampleIndex]) ||
                rootForwards[sampleIndex].LengthSquared() <= MinimumDeltaTime)
            {
                LOG(Error, "Motion matching bake failed: invalid root transform at sample {}.", sampleIndex);
                return false;
            }

            const int32 featureOffset = sampleIndex * poseFeatureSize;
            for (int32 boneIndex = 0; boneIndex < poseBoneCount; boneIndex++)
            {
                const int32 boneOffset = featureOffset + boneIndex * 3;
                const Float3& position = positions[boneIndex];
                poseFeatures[boneOffset] = position.X;
                poseFeatures[boneOffset + 1] = position.Y;
                poseFeatures[boneOffset + 2] = position.Z;
                IncludeVectorRange(featureMinimums.Get(), featureMaximums.Get(), boneIndex * 3, position);
            }

            IncludeVectorRange(rootPositionMinimums, rootPositionMaximums, 0, rootPositions[sampleIndex]);
        }

        clip.StartRootTransform = samples[clip.SampleStart].RootTransform;
        clip.StartRootInverse = Matrix::Invert(clip.StartRootTransform);
        if (!SampleAnimationModelPose(clip.Clip, model, nodeToChannel, clip.Length, pose) ||
            rootIndex >= pose.Count())
        {
            LOG(
                Error,
                "Motion matching bake failed while sampling clip {} endpoint at {:.4f}s.",
                clipIndex,
                clip.Length);
            return false;
        }
        clip.CycleRootTransform = MultiplyMatrices(pose[rootIndex], clip.StartRootInverse);
    }

    LOG(Info, "Baking pose positions: {}/{}", totalSampleCount, totalSampleCount);

    Array<float> maximumSpeeds;
    maximumSpeeds.Resize(poseBoneCount);
    for (int32 i = 0; i < poseBoneCount; i++)
        maximumSpeeds[i] = 0.0f;
    int32 contactSamples = 0;
    Float3 poseVelocityRootRelative[FeatureLayout::MaxPoseBones];
    for (int32 sampleIndex = 0; sampleIndex < totalSampleCount; sampleIndex++)
    {
        const SampleBakeInfo& current = samples[sampleIndex];
        const ClipBakeInfo& clip = clips[current.ClipIndex];
        const int32 localIndex = sampleIndex - clip.SampleStart;
        // T1 endpoint contract (Phase 2 §18.6, option 1: interior central,
        // edges one-sided). Indices come from the shared helper — no ad-hoc
        // prev/next literals here; the live query documents the same stencil.
        int32 previousLocalIndex = 0;
        int32 nextLocalIndex = 0;
        MotionMatchingSampler::GetCentralDiffIndices(
            localIndex, clip.SampleCount, clip.Loop, previousLocalIndex, nextLocalIndex);
        const int32 previousIndex = clip.SampleStart + previousLocalIndex;
        const int32 nextIndex = clip.SampleStart + nextLocalIndex;
        const SampleBakeInfo& previous = samples[previousIndex];
        const SampleBakeInfo& next = samples[nextIndex];
        const Float3* currentPositions = &samplePositions[sampleIndex * poseBoneCount];
        const Float3* previousPositions = &samplePositions[previousIndex * poseBoneCount];
        const Float3* nextPositions = &samplePositions[nextIndex * poseBoneCount];
        const float frameDeltaTime = MotionMatchingSampler::GetFrameDt(sampleRate);
        // Central dt spans two frames; at non-loop edges the clamped neighbor
        // collapses (next.Time - previous.Time) to a single frame, which IS
        // the one-sided stencil — never a fabricated zero future.
        const float deltaTime = clip.Loop
            ? frameDeltaTime * 2.0f
            : next.Time - previous.Time;

        // C1: root-relative pose velocity (bake/query feature). Positions are
        // already root-relative, so this is the feature-space velocity — NOT
        // the world-space foot velocity used by foot-lock. Named distinctly
        // so the two semantics can never share a threshold blindly.
        for (int32 boneIndex = 0; boneIndex < poseBoneCount; boneIndex++)
            poseVelocityRootRelative[boneIndex] = Float3::Zero;
        Float3 rootVelocity = Float3::Zero;
        if (deltaTime > MinimumDeltaTime)
        {
            for (int32 boneIndex = 0; boneIndex < poseBoneCount; boneIndex++)
                poseVelocityRootRelative[boneIndex] = (nextPositions[boneIndex] - previousPositions[boneIndex]) / deltaTime;
            if (clip.Loop)
            {
                // T1: each stencil side wraps in its own cycle with the
                // matching CycleRootTransform power, so seam neighbors are
                // cycle-correct (no endpoint duplication).
                const Matrix previousRoot = GetRootAtTime(
                    clip,
                    samples,
                    current.Time - frameDeltaTime,
                    sampleRate);
                const Matrix nextRoot = GetRootAtTime(
                    clip,
                    samples,
                    current.Time + frameDeltaTime,
                    sampleRate);
                rootVelocity =
                    (nextRoot.GetTranslation() - previousRoot.GetTranslation()) /
                    deltaTime;
            }
            else
            {
                rootVelocity =
                    (next.RootTransform.GetTranslation() - previous.RootTransform.GetTranslation()) /
                    deltaTime;
            }
        }

        bool velocitiesFinite = IsFinite(rootVelocity);
        for (int32 boneIndex = 0; boneIndex < poseBoneCount; boneIndex++)
            velocitiesFinite &= IsFinite(poseVelocityRootRelative[boneIndex]);
        if (!velocitiesFinite)
        {
            LOG(Error, "Motion matching bake failed: non-finite velocity at sample {}.", sampleIndex);
            return false;
        }

        rootVelocities[sampleIndex] = rootVelocity;
        IncludeVectorRange(rootVelocityMinimums, rootVelocityMaximums, 0, rootVelocity);

        const int32 featureOffset = sampleIndex * poseFeatureSize + positionFeatureSize;
        for (int32 boneIndex = 0; boneIndex < poseBoneCount; boneIndex++)
        {
            const int32 boneOffset = featureOffset + boneIndex * 3;
            const Float3& velocity = poseVelocityRootRelative[boneIndex];
            poseFeatures[boneOffset] = velocity.X;
            poseFeatures[boneOffset + 1] = velocity.Y;
            poseFeatures[boneOffset + 2] = velocity.Z;
            IncludeVectorRange(
                featureMinimums.Get(),
                featureMaximums.Get(),
                positionFeatureSize + boneIndex * 3,
                velocity);
            maximumSpeeds[boneIndex] = Math::Max(
                maximumSpeeds[boneIndex],
                Math::Sqrt(velocity.LengthSquared()));
        }

        // Contact flags (C1): planted = slow and low in the root frame, one
        // per declared contact, from the root-relative position and velocity
        // of its pose bone. Height = dot(relativeBone, normalizedRootUpAxis)
        // via the shared helper (never a hardcoded axis); positions are
        // root-relative, so the root sits at the origin. Offsets come from
        // the layout, never from size-N arithmetic.
        const Float3 rootOrigin = Float3::Zero;
        for (int32 contact = 0; contact < contactCount; contact++)
        {
            const int32 bone = layout.ContactPoseBones[contact];
            const int32 contactOffset = layout.GetContactFeatureOffset(contact);
            const float contactHeight = PoseSearchSchema::ComputeContactHeight(
                currentPositions[bone], rootOrigin, rootUpAxis);
            const float planted = PoseSearchSchema::IsPlantedContact(
                poseVelocityRootRelative[bone].LengthSquared(), contactMaxSpeed,
                contactHeight, contactMaxHeight) ? 1.0f : 0.0f;
            poseFeatures[sampleIndex * poseFeatureSize + contactOffset] = planted;
            IncludeRange(featureMinimums.Get(), featureMaximums.Get(), contactOffset, planted);
            if (planted > 0.5f)
                contactSamples++;
        }
    }

    LOG(Info, "Baking pose velocities: {}/{}", totalSampleCount, totalSampleCount);

    // T1 tail-validity telemetry (§18.6): non-loop future targets past the clip
    // end keep a clamped endpoint pose for playback, but search must treat them
    // as invalid/penalized via MotionMatchingSampler::IsNonLoopFutureOutside
    // (S1 integration, runtime horizon check, no rebake).
    int32 tailClampedTargets = 0;

    for (int32 sampleIndex = 0; sampleIndex < totalSampleCount; sampleIndex++)
    {
        const SampleBakeInfo& current = samples[sampleIndex];
        const ClipBakeInfo& clip = clips[current.ClipIndex];
        const Matrix currentRootInverse = Matrix::Invert(current.RootTransform);
        const int32 trajectoryOffset = sampleIndex * trajectoryFeatureSize;
        const float frameDt = MotionMatchingSampler::GetFrameDt(sampleRate);

        for (int32 pointIndex = 0; pointIndex < trajectoryPointCount; pointIndex++)
        {
            const float targetTime = current.Time + schemaTrajectorySampleTimes[pointIndex];
            if (!MotionMatchingSampler::IsTrajectoryTargetValid(targetTime, clip.Length, clip.Loop))
                tailClampedTargets++;
            const Matrix targetRoot = GetRootAtTime(
                clip,
                samples,
                targetTime,
                sampleRate);

            const Float3 relativePosition = GetRelativePosition(
                targetRoot,
                currentRootInverse);
            const Float3 targetForward = GetModelDirection(targetRoot, rootForwardAxis);
            Float3 relativeForward;
            Float3::TransformNormal(targetForward, currentRootInverse, relativeForward);

            if (!IsFinite(relativePosition) || !IsFinite(relativeForward))
            {
                LOG(Error, "Motion matching bake failed: invalid trajectory at sample {}.", sampleIndex);
                return false;
            }

            Float2 planarPosition(
                Float3::Dot(relativePosition, localRight),
                Float3::Dot(relativePosition, rootForwardAxis));
            Float2 planarDirection(
                Float3::Dot(relativeForward, localRight),
                Float3::Dot(relativeForward, rootForwardAxis));
            if (planarDirection.LengthSquared() > MinimumDeltaTime)
                planarDirection.Normalize();
            else
                planarDirection = Float2(0.0f, 1.0f);

            // Yaw-rate channel (M3 + T1): explicit turn signal from baked root
            // motion, measured in the model ground plane (PlanarFrame; a Y-up
            // model keeps the (x, z) yaw exactly) - the same physical quantity (rad/s, unwrap [-pi,pi]) the
            // live query derives from predicted facings, through the SAME
            // MotionMatchingSampler unwrap/rate helpers. Stencil differs by
            // construction and is documented, not hidden: bake = central ±1
            // bake sample (2*frameDt); query = central ±1 trajectory interval
            // (schema times). Non-loop edges use one-sided differences over a
            // single true frame instead of halving the rate against a clamped
            // twin, so a moving tail never bakes a fake slowdown.
            const float pastTime = targetTime - frameDt;
            const float futureTime = targetTime + frameDt;
            const bool pastValid = MotionMatchingSampler::IsTrajectoryTargetValid(pastTime, clip.Length, clip.Loop);
            const bool futureValid = MotionMatchingSampler::IsTrajectoryTargetValid(futureTime, clip.Length, clip.Loop);
            const Float3 pastForward = GetModelDirection(
                GetRootAtTime(clip, samples, pastTime, sampleRate), rootForwardAxis);
            const Float3 futureForward = GetModelDirection(
                GetRootAtTime(clip, samples, futureTime, sampleRate), rootForwardAxis);
            float yawRate = 0.0f;
            if (IsFinite(pastForward) && IsFinite(futureForward) && IsFinite(targetForward))
            {
                const Float2 pastPlanar = planarYUp ? Float2(pastForward.X, pastForward.Z) : PlanarFrame::Project(pastForward, planarUpAxis);
                const Float2 targetPlanar = planarYUp ? Float2(targetForward.X, targetForward.Z) : PlanarFrame::Project(targetForward, planarUpAxis);
                const Float2 futurePlanar = planarYUp ? Float2(futureForward.X, futureForward.Z) : PlanarFrame::Project(futureForward, planarUpAxis);
                if (pastValid && futureValid)
                {
                    yawRate = MotionMatchingSampler::ComputeYawRate(
                        MotionMatchingSampler::YawFromForwardXZ(pastPlanar.X, pastPlanar.Y),
                        MotionMatchingSampler::YawFromForwardXZ(futurePlanar.X, futurePlanar.Y),
                        2.0f * frameDt);
                }
                else if (pastValid)
                {
                    yawRate = MotionMatchingSampler::ComputeYawRate(
                        MotionMatchingSampler::YawFromForwardXZ(pastPlanar.X, pastPlanar.Y),
                        MotionMatchingSampler::YawFromForwardXZ(targetPlanar.X, targetPlanar.Y),
                        frameDt);
                }
                else if (futureValid)
                {
                    yawRate = MotionMatchingSampler::ComputeYawRate(
                        MotionMatchingSampler::YawFromForwardXZ(targetPlanar.X, targetPlanar.Y),
                        MotionMatchingSampler::YawFromForwardXZ(futurePlanar.X, futurePlanar.Y),
                        frameDt);
                }
                // Neither side valid (clip shorter than one frame): leave 0;
                // pose/root velocity is likewise degenerate there, consistently.
            }

            const int32 pointOffset = trajectoryOffset + pointIndex * TrajectoryPointSize;
            trajectoryFeatures[pointOffset] = planarPosition.X;
            trajectoryFeatures[pointOffset + 1] = planarPosition.Y;
            trajectoryFeatures[pointOffset + 2] = planarDirection.X;
            trajectoryFeatures[pointOffset + 3] = planarDirection.Y;
            trajectoryFeatures[pointOffset + 4] = yawRate;
            for (int32 component = 0; component < TrajectoryPointSize; component++)
            {
                IncludeRange(
                    trajectoryMinimums.Get(),
                    trajectoryMaximums.Get(),
                    pointIndex * TrajectoryPointSize + component,
                    trajectoryFeatures[pointOffset + component]);
            }
        }
    }

    LOG(Info, "Baking trajectory features: {}/{}", totalSampleCount, totalSampleCount);
    LOG(
        Info,
        "Non-loop trajectory targets clamped for playback (validity for search via "
        "MotionMatchingSampler::IsNonLoopFutureOutside, S1): {}.",
        tailClampedTargets);
    LOG(
        Info,
        "Pose feature baking finished: {}/{}, dimensions={}.",
        totalSampleCount,
        totalSampleCount,
        poseFeatureSize);
    // Per-bone ranges in the root frame (positions, then speed maxima).
    for (int32 boneIndex = 0; boneIndex < poseBoneCount; boneIndex++)
    {
        const int32 p = boneIndex * 3;
        const int32 v = positionFeatureSize + boneIndex * 3;
        LOG(
            Info,
            "pose bone '{}': pos=[{:.2f}, {:.2f}][{:.2f}, {:.2f}][{:.2f}, {:.2f}], vel=[{:.2f}, {:.2f}][{:.2f}, {:.2f}][{:.2f}, {:.2f}], max speed {:.2f} units/s.",
            layout.PoseBoneNames[boneIndex],
            featureMinimums[p], featureMaximums[p],
            featureMinimums[p + 1], featureMaximums[p + 1],
            featureMinimums[p + 2], featureMaximums[p + 2],
            featureMinimums[v], featureMaximums[v],
            featureMinimums[v + 1], featureMaximums[v + 1],
            featureMinimums[v + 2], featureMaximums[v + 2],
            maximumSpeeds[boneIndex]);
    }
    for (int32 contact = 0; contact < contactCount; contact++)
    {
        const int32 offset = layout.GetContactFeatureOffset(contact);
        LOG(
            Info,
            "contact '{}'=[{:.2f}, {:.2f}].",
            layout.PoseBoneNames[layout.ContactPoseBones[contact]],
            featureMinimums[offset], featureMaximums[offset]);
    }
    if (contactCount > 0)
    {
        LOG(
            Info,
            "contact planted fraction={:.3f}.",
            static_cast<float>(contactSamples) / static_cast<float>(totalSampleCount * contactCount));
    }
    LOG(
        Info,
        "root_pos_x=[{:.2f}, {:.2f}], root_pos_y=[{:.2f}, {:.2f}], root_pos_z=[{:.2f}, {:.2f}]",
        rootPositionMinimums[0], rootPositionMaximums[0],
        rootPositionMinimums[1], rootPositionMaximums[1],
        rootPositionMinimums[2], rootPositionMaximums[2]);
    LOG(
        Info,
        "root_vel_x=[{:.2f}, {:.2f}], root_vel_y=[{:.2f}, {:.2f}], root_vel_z=[{:.2f}, {:.2f}]",
        rootVelocityMinimums[0], rootVelocityMaximums[0],
        rootVelocityMinimums[1], rootVelocityMaximums[1],
        rootVelocityMinimums[2], rootVelocityMaximums[2]);

    for (int32 pointIndex = 0; pointIndex < trajectoryPointCount; pointIndex++)
    {
        const int32 offset = pointIndex * TrajectoryPointSize;
        LOG(
            Info,
            "trajectory_{:.2f}s_pos_right=[{:.2f}, {:.2f}], trajectory_{:.2f}s_pos_forward=[{:.2f}, {:.2f}], "
            "trajectory_{:.2f}s_dir_right=[{:.2f}, {:.2f}], trajectory_{:.2f}s_dir_forward=[{:.2f}, {:.2f}], "
            "trajectory_{:.2f}s_yawrate=[{:.2f}, {:.2f}]",
            schemaTrajectorySampleTimes[pointIndex], trajectoryMinimums[offset], trajectoryMaximums[offset],
            schemaTrajectorySampleTimes[pointIndex], trajectoryMinimums[offset + 1], trajectoryMaximums[offset + 1],
            schemaTrajectorySampleTimes[pointIndex], trajectoryMinimums[offset + 2], trajectoryMaximums[offset + 2],
            schemaTrajectorySampleTimes[pointIndex], trajectoryMinimums[offset + 3], trajectoryMaximums[offset + 3],
            schemaTrajectorySampleTimes[pointIndex], trajectoryMinimums[offset + 4], trajectoryMaximums[offset + 4]);
    }

    // N1b rev 3: one mean-row-norm deviation per normalization group; baked
    // features stay raw. Groups come from the layout derived from the
    // declared group labels (single source of truth), never from hardcoded
    // dim ranges or bone names.
    Array<int32> trajectoryGroups;
    trajectoryGroups.Resize(trajectoryFeatureSize);
    for (int32 dim = 0; dim < trajectoryFeatureSize; dim++)
        trajectoryGroups[dim] = FeatureLayout::GetTrajectoryNormalizationGroup(dim);
    const int32 constantPoseFeatures = ComputeGroupDeviations(
        poseFeatures,
        totalSampleCount,
        poseFeatureSize,
        layout.PoseGroups,
        poseFeatureDeviations);
    const int32 constantTrajectoryFeatures = ComputeGroupDeviations(
        trajectoryFeatures,
        totalSampleCount,
        trajectoryFeatureSize,
        trajectoryGroups,
        trajectoryFeatureDeviations);
    LOG(
        Info,
        "Feature deviations computed (N1b UE-style rev {}): raw features kept, pose dimensions={} ({} floored), trajectory dimensions={} ({} floored).",
        PoseSearchSchema::NormalizationRevision,
        poseFeatureSize,
        constantPoseFeatures,
        trajectoryFeatureSize,
        constantTrajectoryFeatures);
    // Grouped-stats audit: one line per pose group proves bones sharing a
    // label share the group deviation by construction (symmetric costs for
    // mirrored queries). All groups use the same mean-row-norm formula.
    {
        Array<int32> loggedGroups;
        for (int32 dim = 0; dim < poseFeatureSize; dim++)
        {
            const int32 group = layout.GetPoseNormalizationGroup(dim);
            if (loggedGroups.Contains(group))
                continue;
            loggedGroups.Add(group);
            int32 dimCount = 0;
            for (int32 other = dim; other < poseFeatureSize; other++)
            {
                if (layout.GetPoseNormalizationGroup(other) == group)
                    dimCount++;
            }
            LOG(
                Info,
                "pose norm group {}: dims={} deviation={:.4f}.",
                group,
                dimCount,
                poseFeatureDeviations[dim]);
        }
    }

    database->SetBakedData(
        bakedSchema,
        sourceClipIndices,
        clipLengths,
        clipSampleCounts,
        sampleClipIndices,
        sampleTimes,
        rootPositions,
        rootVelocities,
        rootForwards,
        poseFeatures,
        trajectoryFeatures,
        poseFeatureDeviations,
        trajectoryFeatureDeviations,
        skeletonFingerprint,
        planarUpAxis);
    if (!database->ValidateData())
    {
        LOG(Error, "Motion matching bake failed: generated database did not pass validation.");
        return false;
    }
    const int32 expectedClipCount = database->GetClipCount();
    const int32 expectedSampleCount = database->GetSampleCount();
    if (database->Save())
    {
        LOG(Error, "Motion matching bake failed while saving database '{}'.", database->GetPath());
        return false;
    }

    // Persistence check with retries: SaveAsset triggers an async engine
    // reimport ("Creating package ... file is in use") and an immediate
    // Reload can read pre-save bytes (observed: revision tail missing on
    // first read). Retry until the reloaded database matches, revision and
    // layout hash included — a stale read fails the checks, not silently.
    bool persisted = false;
    for (int32 attempt = 0; attempt < 6 && !persisted; attempt++)
    {
        if (attempt > 0)
            LOG(Info, "Motion matching persistence check retry {}/5.", attempt);
        database->Reload();
        if (database->WaitForLoaded(10000) ||
            !database->ValidateData() ||
            database->GetClipCount() != expectedClipCount ||
            database->GetSampleCount() != expectedSampleCount ||
            database->GetSchema() != bakedSchema ||
            database->GetNormalizationRevision() != PoseSearchSchema::NormalizationRevision ||
            database->GetFeatureLayoutHash() != static_cast<int32>(layout.ComputeHash()))
            continue;
        persisted = true;
    }
    if (!persisted)
    {
        LOG(Error, "Motion matching bake failed persistence reload check for '{}'.", database->GetPath());
        return false;
    }

    LOG(
        Info,
        "Motion matching database saved: clips={}, samples={}, path='{}'.",
        database->GetClipCount(),
        database->GetSampleCount(),
        database->GetPath());
    bakedClipCount = expectedClipCount;
    bakedSampleCount = expectedSampleCount;
    return true;
}

Array<BoneAlias> MotionMatchingBaker::SuggestBoneAliases(
    SkinnedModel* model,
    const Array<Animation*>& clips,
    const PoseSearchSchema& schema)
{
    ChannelCoverage coverage;
    AnalyzeCoverage(model, clips, schema, coverage);
    return coverage.Suggestions;
}

String MotionMatchingBaker::DescribeChannelCoverage(
    SkinnedModel* model,
    const Array<Animation*>& clips,
    const PoseSearchSchema& schema)
{
    if (model == nullptr || model->WaitForLoaded())
        return String(TEXT("Channel coverage unavailable: the model is not loaded."));
    ChannelCoverage coverage;
    AnalyzeCoverage(model, clips, schema, coverage);
    String text = DescribeCoverage(coverage);
    MotionMatchingSkeletonFingerprint fingerprint;
    String proportionReport;
    if (MotionMatchingSkeletonFingerprint::Build(model, clips, schema.BoneAliases, fingerprint, proportionReport) &&
        proportionReport.HasChars())
        text += String(TEXT("\n  PROPORTIONS: ")) + proportionReport;
    return text;
}

#endif
