#include "MotionMatchingDatabase.h"

#if USE_EDITOR

#include "Engine/Core/Log.h"
#include "Engine/Serialization/MemoryWriteStream.h"
#include "Engine/Threading/Threading.h"

bool MotionMatchingDatabase::AddSourceClips(
    const Array<String>& tags,
    const Array<Animation*>& animations,
    bool loop,
    int32 group)
{
    const Array<String> set = NormalizeTags(tags);
    if (set.IsEmpty())
        return false;

    bool changed = false;
    for (Animation* animation : animations)
    {
        if (animation == nullptr)
            continue;

        int32 existingIndex = -1;
        for (int32 i = 0; i < _sourceAnimations.Count(); i++)
        {
            if (_sourceAnimations[i].GetID() == animation->GetID() &&
                TagSetsEqual(_sourceTags[i], set))
            {
                existingIndex = i;
                break;
            }
        }

        if (existingIndex >= 0)
        {
            const int32 loopValue = loop ? 1 : 0;
            if (_sourceLooping[existingIndex] != loopValue)
            {
                _sourceLooping[existingIndex] = loopValue;
                changed = true;
            }
            if (_sourceGroups[existingIndex] != group)
            {
                _sourceGroups[existingIndex] = group;
                changed = true;
            }
            continue;
        }

        _sourceAnimations.Add(animation);
        _sourceTags.Add(set);
        _sourceLooping.Add(loop ? 1 : 0);
        _sourceGroups.Add(group);
        changed = true;
    }

    if (changed)
    {
        RebuildTagCounts();
        ClearBakedData();
    }
    return changed;
}

void MotionMatchingDatabase::ClearSourceClips()
{
    _sourceAnimations.Clear();
    _sourceTags.Clear();
    _sourceLooping.Clear();
    _sourceGroups.Clear();
    _tagNames.Clear();
    _tagClipCounts.Clear();
    ClearBakedData();
}

void MotionMatchingDatabase::SetBakedData(
    const PoseSearchSchema& schema,
    const Array<int32>& sourceClipIndices,
    const Array<float>& clipLengths,
    const Array<int32>& clipSampleCounts,
    const Array<int32>& sampleClipIndices,
    const Array<float>& sampleTimes,
    const Array<Float3>& rootPositions,
    const Array<Float3>& rootVelocities,
    const Array<Float3>& rootForwards,
    const Array<float>& poseFeatures,
    const Array<float>& trajectoryFeatures,
    const Array<float>& poseFeatureDeviations,
    const Array<float>& trajectoryFeatureDeviations,
    const MotionMatchingSkeletonFingerprint& skeletonFingerprint,
    const Float3& planarUpAxis)
{
    ClearBakedData();
    _schema = schema;
    RebuildLayout();
    _planarUpAxis = planarUpAxis;

    _clipAnimations.EnsureCapacity(sourceClipIndices.Count());
    _clipTags.EnsureCapacity(sourceClipIndices.Count());
    _clipLooping.EnsureCapacity(sourceClipIndices.Count());
    _clipGroups.EnsureCapacity(sourceClipIndices.Count());
    for (int32 sourceIndex : sourceClipIndices)
    {
        // Registry copy: baked clips carry the tag set, loop flag and group
        // the source had at bake time (registry edits clear baked data, so
        // no drift). The baker never derives metadata from asset names.
        _clipAnimations.Add(_sourceAnimations[sourceIndex]);
        _clipTags.Add(_sourceTags[sourceIndex]);
        _clipLooping.Add(_sourceLooping[sourceIndex]);
        _clipGroups.Add(GetSourceClipGroup(sourceIndex));
    }
    _skeletonFingerprint = skeletonFingerprint;

    _clipLengths = clipLengths;
    _clipSampleCounts = clipSampleCounts;
    _sampleClipIndices = sampleClipIndices;
    _sampleTimes = sampleTimes;
    _rootPositions = rootPositions;
    _rootVelocities = rootVelocities;
    _rootForwards = rootForwards;
    _poseFeatures = poseFeatures;
    _trajectoryFeatures = trajectoryFeatures;
    _poseFeatureDeviations = poseFeatureDeviations;
    _trajectoryFeatureDeviations = trajectoryFeatureDeviations;
    // Freshly baked by this build: stamp the current statistics revision and
    // the feature-layout hash of the baked schema.
    _normalizationRevision = PoseSearchSchema::NormalizationRevision;
    _featureLayoutHash = _currentLayoutHash;
    // D1 §18.5 provenance: schema contract revision (class const), sampling
    // contract hash (schema sampling values + SamplingContractRevision),
    // source manifest identity (registered source asset IDs/tags/loop) and the
    // bake-tool build id. ValidateDataStatus recomputes these and drops the
    // baked arrays to registry-only on any mismatch.
    _schemaContractRevision = SchemaContractRevision;
    _samplingContractHash = ComputeSamplingContractHash();
    _sourceManifestHash = ComputeSourceManifestHash();
    _bakeToolBuildId = BakeToolBuildId;
    ComputeClipTurnAngles();
}

void MotionMatchingDatabase::GetReferences(
    Array<Guid>& assets,
    Array<String>& files) const
{
    BinaryAsset::GetReferences(assets, files);
    for (const AssetReference<Animation>& animation : _sourceAnimations)
    {
        if (animation.GetID().IsValid())
            assets.Add(animation.GetID());
    }
}

bool MotionMatchingDatabase::Save(const StringView& path)
{
    if (OnCheckSave(path))
        return true;
    ScopeLock lock(Locker);

    if (!ValidateData())
    {
        LOG(Error, "Cannot save motion matching database: data validation failed.");
        return true;
    }

    MemoryWriteStream stream(4096);
    _schema.Serialize(stream);
    stream.Write(_sourceAnimations);
    stream.Write(_sourceTags);
    stream.Write(_sourceLooping);
    stream.Write(_tagNames);
    stream.Write(_tagClipCounts);
    stream.Write(_clipAnimations);
    stream.Write(_clipTags);
    stream.Write(_clipLooping);
    stream.Write(_clipLengths);
    stream.Write(_clipSampleCounts);
    stream.Write(_sampleClipIndices);
    stream.Write(_sampleTimes);
    stream.Write(_poseFeatures);
    stream.Write(_trajectoryFeatures);
    stream.Write(_rootPositions);
    stream.Write(_rootVelocities);
    stream.Write(_rootForwards);
    // N1b rev 3: the legacy value-mean slots stay in the layout as empty
    // arrays so the byte stream keeps its shape (load() still reads them) and
    // rev-3 files stay distinguishable from rev-2 by the stored revision.
    // Only deviations are persisted as real statistics.
    const Array<float> noMeans;
    stream.Write(noMeans);
    stream.Write(_poseFeatureDeviations);
    stream.Write(noMeans);
    stream.Write(_trajectoryFeatureDeviations);
    // Legacy turn-like slot (v3), written empty so the layout never reorders.
    const Array<int32> noTurnLike;
    stream.Write(noTurnLike);
    stream.WriteInt32(_normalizationRevision);
    stream.WriteInt32(_featureLayoutHash);
    // D1 §18.5 provenance tails (read back conditionally, so pre-D1 files stay
    // loadable as registry-only).
    stream.WriteInt32(_schemaContractRevision);
    stream.WriteInt32(_samplingContractHash);
    stream.WriteInt32(_sourceManifestHash);
    stream.WriteInt32(_bakeToolBuildId);
    // Per-clip groups in the v3 speed-band slot, then the v4 tail: per-source
    // groups and the skeleton fingerprint. New data is only ever appended.
    stream.Write(_clipGroups);
    stream.Write(_sourceGroups);
    _skeletonFingerprint.Serialize(stream);
    // v5 tail: model-space planar up axis.
    stream.Write(_planarUpAxis);

    // Every save writes the current type name, also for a file that still carried
    // the legacy name in its header.
    _header.TypeName = TypeName;
    FlaxChunk* chunk;
    if (IsVirtual())
    {
        _header.Chunks[0] = chunk = New<FlaxChunk>();
    }
    else
    {
        chunk = GetOrCreateChunk(0);
    }
    chunk->Data.Copy(ToSpan(stream));

    AssetInitData data;
    data.SerializedVersion = SerializedVersion;
    const bool failed = path.HasChars()
        ? SaveAsset(path, data)
        : SaveAsset(data, true);

    if (IsVirtual())
    {
        _header.Chunks[0] = nullptr;
        Delete(chunk);
    }

    if (failed)
        LOG(Error, "Failed to save motion matching database '{}'.", path);
    return failed;
}

#endif
