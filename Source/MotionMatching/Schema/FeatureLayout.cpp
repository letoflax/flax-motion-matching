#include "FeatureLayout.h"

void FeatureLayout::Clear()
{
    PoseBoneCount = 0;
    ContactCount = 0;
    TrajectoryPointCount = 0;
    PoseFeatureSize = 0;
    TrajectoryFeatureSize = 0;
    ZeroTimePoint = -1;
    PoseBoneNames.Clear();
    PoseBoneGroupLabels.Clear();
    ContactPoseBones.Clear();
    PoseGroups.Clear();
}

void FeatureLayout::DescribePoseChannels(Array<FeatureChannel>& outChannels) const
{
    outChannels.Clear();
    const int32 positionBlockSize = GetPositionBlockSize();
    for (int32 bone = 0; bone < PoseBoneCount; bone++)
    {
        const String& node = PoseBoneNames[bone];
        const String& label = PoseBoneGroupLabels[bone];

        FeatureChannel pos;
        pos.Name = String(TEXT("pose.positions.")) + node;
        pos.Block = 0;
        pos.Offset = bone * 3;
        pos.Dimension = 3;
        pos.Weight = 1.0f;
        pos.NormalizationGroup = GetPoseNormalizationGroup(pos.Offset);
        pos.Kind = FeatureChannelKind::Vector;
        pos.SymmetryGroup = pos.NormalizationGroup;
        pos.Semantic = String(TEXT("position.")) + label;
        pos.ContactIndex = -1;
        outChannels.Add(pos);

        FeatureChannel vel;
        vel.Name = String(TEXT("pose.velocities.")) + node;
        vel.Block = 0;
        vel.Offset = positionBlockSize + bone * 3;
        vel.Dimension = 3;
        vel.Weight = 1.0f;
        vel.NormalizationGroup = GetPoseNormalizationGroup(vel.Offset);
        vel.Kind = FeatureChannelKind::Vector;
        vel.SymmetryGroup = vel.NormalizationGroup;
        vel.Semantic = String(TEXT("velocity.")) + label;
        vel.ContactIndex = -1;
        outChannels.Add(vel);
    }

    for (int32 contact = 0; contact < ContactCount; contact++)
    {
        FeatureChannel ch;
        ch.Name = String(TEXT("pose.contacts.")) + PoseBoneNames[ContactPoseBones[contact]];
        ch.Block = 0;
        ch.Offset = GetContactFeatureOffset(contact);
        ch.Dimension = 1;
        ch.Weight = 1.0f;
        ch.NormalizationGroup = GetPoseNormalizationGroup(ch.Offset);
        ch.Kind = FeatureChannelKind::Scalar;
        ch.SymmetryGroup = ch.NormalizationGroup;
        ch.Semantic = String(TEXT("contact"));
        ch.ContactIndex = contact;
        outChannels.Add(ch);
    }
}

void FeatureLayout::DescribeTrajectoryChannels(Array<FeatureChannel>& outChannels) const
{
    outChannels.Clear();
    // Trajectory dims stay scalar with identity groups. A future
    // vectorization (pos_xy/facing_xy sharing one deviation) would change
    // group ids, the layout hash and the normalization revision.
    const Char* fieldNames[TrajectoryFloatsPerPoint] = {
        TEXT("position_x"), TEXT("position_y"),
        TEXT("facing_x"), TEXT("facing_y"), TEXT("yaw_rate")
    };
    const Char* fieldSemantics[TrajectoryFloatsPerPoint] = {
        TEXT("trajectory.position"), TEXT("trajectory.position"),
        TEXT("trajectory.facing"), TEXT("trajectory.facing"), TEXT("trajectory.yaw_rate")
    };
    for (int32 point = 0; point < TrajectoryPointCount; point++)
    {
        for (int32 field = 0; field < TrajectoryFloatsPerPoint; field++)
        {
            const int32 local = point * TrajectoryFloatsPerPoint + field;
            FeatureChannel ch;
            ch.Name = String(TEXT("trajectory.")) + String::Format(TEXT("{}"), point) +
                String(TEXT(".")) + String(fieldNames[field]);
            ch.Block = 1;
            ch.Offset = local;
            ch.Dimension = 1;
            ch.Weight = 1.0f;
            ch.NormalizationGroup = GetTrajectoryNormalizationGroup(local);
            ch.Kind = FeatureChannelKind::Scalar;
            ch.SymmetryGroup = ch.NormalizationGroup;
            ch.Semantic = String(fieldSemantics[field]);
            ch.ContactIndex = -1;
            outChannels.Add(ch);
        }
    }
}

uint32 FeatureLayout::ComputeHash() const
{
    // FNV-1a over the full channel metadata. 0 is reserved for unknown/legacy.
    uint32 hash = 2166136261u;
    auto mixByte = [&](uint32 b)
    {
        hash ^= (b & 0xFFu);
        hash *= 16777619u;
    };
    auto mixInt = [&](int32 v)
    {
        const uint32 u = static_cast<uint32>(v);
        mixByte(u);
        mixByte(u >> 8);
        mixByte(u >> 16);
        mixByte(u >> 24);
    };
    auto mixString = [&](const String& s)
    {
        mixInt(s.Length());
        for (int32 i = 0; i < s.Length(); i++)
        {
            const uint32 c = static_cast<uint32>(s[i]);
            mixByte(c);
            mixByte(c >> 8);
            mixByte(c >> 16);
            mixByte(c >> 24);
        }
    };
    auto mixChannels = [&](const Array<FeatureChannel>& channels)
    {
        for (int32 i = 0; i < channels.Count(); i++)
        {
            const FeatureChannel& ch = channels[i];
            mixString(ch.Name);
            mixInt(ch.Block);
            mixInt(ch.Offset);
            mixInt(ch.Dimension);
            mixInt(ch.NormalizationGroup);
            mixInt(static_cast<int32>(ch.Kind));
            mixInt(ch.SymmetryGroup);
            mixString(ch.Semantic);
            mixInt(ch.ContactIndex);
        }
    };
    Array<FeatureChannel> pose;
    Array<FeatureChannel> trajectory;
    DescribePoseChannels(pose);
    DescribeTrajectoryChannels(trajectory);
    mixInt(PoseFeatureSize);
    mixInt(TrajectoryFeatureSize);
    mixChannels(pose);
    mixChannels(trajectory);
    if (hash == 0)
        hash = 1;
    return hash;
}

bool FeatureLayout::Validate(String& error) const
{
    error = String::Empty;
    if (PoseBoneCount <= 0 || PoseBoneCount > MaxPoseBones ||
        ContactCount < 0 || ContactCount > MaxContacts ||
        TrajectoryPointCount <= 0 || TrajectoryPointCount > MaxTrajectoryPoints ||
        PoseFeatureSize != PoseBoneCount * PoseFloatsPerBone + ContactCount ||
        TrajectoryFeatureSize != TrajectoryPointCount * TrajectoryFloatsPerPoint ||
        PoseBoneNames.Count() != PoseBoneCount ||
        PoseBoneGroupLabels.Count() != PoseBoneCount ||
        ContactPoseBones.Count() != ContactCount ||
        PoseGroups.Count() != PoseFeatureSize ||
        ZeroTimePoint < 0 || ZeroTimePoint >= TrajectoryPointCount)
    {
        error = TEXT("Feature layout sizes are inconsistent.");
        return false;
    }
    for (int32 contact = 0; contact < ContactCount; contact++)
    {
        const int32 bone = ContactPoseBones[contact];
        if (bone < 0 || bone >= PoseBoneCount)
        {
            error = String::Format(TEXT("Contact {} does not reference a pose bone."), contact);
            return false;
        }
    }

    Array<FeatureChannel> pose;
    Array<FeatureChannel> trajectory;
    DescribePoseChannels(pose);
    DescribeTrajectoryChannels(trajectory);

    Array<bool> used;
    used.Resize(PoseFeatureSize);
    for (int32 i = 0; i < PoseFeatureSize; i++)
        used[i] = false;
    for (const FeatureChannel& ch : pose)
    {
        if (ch.Block != 0 || ch.Dimension <= 0 || ch.Offset < 0 || ch.Offset + ch.Dimension > PoseFeatureSize)
        {
            error = String::Format(TEXT("Feature channel '{}' is outside the pose block."), ch.Name);
            return false;
        }
        for (int32 k = 0; k < ch.Dimension; k++)
        {
            if (GetPoseNormalizationGroup(ch.Offset + k) != ch.NormalizationGroup)
            {
                error = String::Format(TEXT("Feature channel '{}' spans more than one normalization group."), ch.Name);
                return false;
            }
            if (used[ch.Offset + k])
            {
                error = String::Format(TEXT("Feature channel '{}' overlaps another pose channel."), ch.Name);
                return false;
            }
            used[ch.Offset + k] = true;
        }
        if (ch.ContactIndex >= 0 &&
            (ch.Dimension != 1 || ch.Offset != GetContactFeatureOffset(ch.ContactIndex) || ch.Kind != FeatureChannelKind::Scalar))
        {
            error = String::Format(TEXT("Contact channel '{}' is not at GetContactFeatureOffset."), ch.Name);
            return false;
        }
    }
    for (int32 i = 0; i < PoseFeatureSize; i++)
    {
        if (!used[i])
        {
            error = String::Format(TEXT("Pose dimension {} is not covered by any feature channel."), i);
            return false;
        }
    }

    used.Resize(TrajectoryFeatureSize);
    for (int32 i = 0; i < TrajectoryFeatureSize; i++)
        used[i] = false;
    for (const FeatureChannel& ch : trajectory)
    {
        if (ch.Block != 1 || ch.Dimension <= 0 || ch.Offset < 0 || ch.Offset + ch.Dimension > TrajectoryFeatureSize)
        {
            error = String::Format(TEXT("Feature channel '{}' is outside the trajectory block."), ch.Name);
            return false;
        }
        for (int32 k = 0; k < ch.Dimension; k++)
        {
            if (used[ch.Offset + k])
            {
                error = String::Format(TEXT("Feature channel '{}' overlaps another trajectory channel."), ch.Name);
                return false;
            }
            used[ch.Offset + k] = true;
        }
    }
    for (int32 i = 0; i < TrajectoryFeatureSize; i++)
    {
        if (!used[i])
        {
            error = String::Format(TEXT("Trajectory dimension {} is not covered by any feature channel."), i);
            return false;
        }
    }
    return true;
}

// C++14 (Flax Linux/clang): ODR-used static constexpr members need an out-of-class definition (undefined symbol at load otherwise).
#if __cplusplus < 201703L
constexpr int32 FeatureLayout::PoseFloatsPerBone;
constexpr int32 FeatureLayout::TrajectoryFloatsPerPoint;
constexpr int32 FeatureLayout::MaxPoseBones;
constexpr int32 FeatureLayout::MaxContacts;
constexpr int32 FeatureLayout::MaxTrajectoryPoints;
constexpr int32 FeatureLayout::MaxPoseFeatureSize;
constexpr int32 FeatureLayout::MaxTrajectoryFeatureSize;
constexpr int32 FeatureLayout::ReferencePoseFeatureSize;
constexpr int32 FeatureLayout::ReferenceTrajectoryFeatureSize;
#endif
