#include "PoseSearchSchema.h"

#include "Engine/Core/Math/Math.h"
#include "Engine/Serialization/ReadStream.h"
#include "Engine/Serialization/WriteStream.h"

#include <cmath>

namespace
{
    constexpr float MinimumAxisLengthSquared = 0.000001f;
    constexpr float TimeZeroTolerance = 0.0001f;

    bool IsFinite(const Float3 &value)
    {
        return std::isfinite(value.X) && std::isfinite(value.Y) && std::isfinite(value.Z);
    }
}

bool PoseSearchSchema::TryValidate(String &error) const
{
    error = String::Empty;

    if (SampleRate <= 0)
    {
        error = TEXT("SampleRate must be greater than zero.");
        return false;
    }

    if (!IsFinite(RootForwardAxis) || !IsFinite(RootUpAxis) ||
        RootForwardAxis.LengthSquared() <= MinimumAxisLengthSquared ||
        RootUpAxis.LengthSquared() <= MinimumAxisLengthSquared)
    {
        error = TEXT("RootForwardAxis and RootUpAxis must be finite and non-zero.");
        return false;
    }
    Float3 right;
    if (!TryGetRightAxis(right))
    {
        error = TEXT("RootForwardAxis and RootUpAxis cannot be parallel.");
        return false;
    }

    if (!std::isfinite(PoseWeight) || !std::isfinite(TrajectoryWeight) ||
        PoseWeight < 0.0f || TrajectoryWeight < 0.0f)
    {
        error = TEXT("PoseWeight and TrajectoryWeight must be finite and non-negative.");
        return false;
    }

    if (!std::isfinite(ContactMaxSpeed) || !std::isfinite(ContactMaxHeight) ||
        ContactMaxSpeed <= 0.0f || ContactMaxHeight <= 0.0f)
    {
        error = TEXT("ContactMaxSpeed and ContactMaxHeight must be finite and greater than zero.");
        return false;
    }

    for (int32 i = 0; i < BoneAliases.Count(); i++)
    {
        const BoneAlias& alias = BoneAliases[i];
        if (alias.Channel.IsEmpty() || alias.Node.IsEmpty())
        {
            error = String::Format(TEXT("BoneAliases[{}] needs both a Channel and a Node name."), i);
            return false;
        }
        for (int32 j = 0; j < i; j++)
        {
            if (BoneAliases[j].Channel == alias.Channel)
            {
                error = String::Format(TEXT("BoneAliases maps channel '{}' twice."), alias.Channel);
                return false;
            }
            if (BoneAliases[j].Node == alias.Node)
            {
                error = String::Format(TEXT("BoneAliases maps channels '{}' and '{}' onto the same node '{}'."),
                    BoneAliases[j].Channel, alias.Channel, alias.Node);
                return false;
            }
        }
    }

    FeatureLayout layout;
    if (!TryBuildLayout(layout, error))
        return false;

    return true;
}

bool PoseSearchSchema::TryBuildLayout(FeatureLayout& layout, String& error) const
{
    layout.Clear();
    error = String::Empty;
    const Char* configureHint = TEXT("Declare the feature layout for your rig in Schema (RootBone, PoseBones, ContactBones).");

    if (RootBone.IsEmpty())
    {
        error = String::Format(TEXT("Schema.RootBone is empty. {}"), String(configureHint));
        return false;
    }
    const int32 boneCount = PoseBones.Count();
    if (boneCount <= 0 || boneCount > FeatureLayout::MaxPoseBones)
    {
        error = String::Format(TEXT("Schema.PoseBones must declare 1 to {} bones (has {}). {}"),
            FeatureLayout::MaxPoseBones, boneCount, String(configureHint));
        return false;
    }
    for (int32 i = 0; i < boneCount; i++)
    {
        const String& node = PoseBones[i].Node;
        if (node.IsEmpty())
        {
            error = String::Format(TEXT("Schema.PoseBones[{}] has an empty Node name."), i);
            return false;
        }
        if (node == RootBone)
        {
            error = String::Format(TEXT("Schema.PoseBones[{}] is the root bone '{}'; its root-relative features are always zero."), i, node);
            return false;
        }
        for (int32 j = 0; j < i; j++)
        {
            if (PoseBones[j].Node == node)
            {
                error = String::Format(TEXT("Schema.PoseBones declares node '{}' twice."), node);
                return false;
            }
        }
    }
    const int32 contactCount = ContactBones.Count();
    if (contactCount > FeatureLayout::MaxContacts)
    {
        error = String::Format(TEXT("Schema.ContactBones declares {} contacts; at most {} are supported."),
            contactCount, FeatureLayout::MaxContacts);
        return false;
    }

    const int32 pointCount = TrajectorySampleTimes.Count();
    if (pointCount <= 0 || pointCount > FeatureLayout::MaxTrajectoryPoints)
    {
        error = String::Format(TEXT("TrajectorySampleTimes must contain 1 to {} values (has {})."),
            FeatureLayout::MaxTrajectoryPoints, pointCount);
        return false;
    }
    int32 zeroPoint = -1;
    for (int32 i = 0; i < pointCount; i++)
    {
        const float time = TrajectorySampleTimes[i];
        if (!std::isfinite(time) || (i > 0 && time <= TrajectorySampleTimes[i - 1]))
        {
            error = TEXT("TrajectorySampleTimes must be finite and strictly ascending.");
            return false;
        }
        if (zeroPoint < 0 && std::fabs(time) <= TimeZeroTolerance)
            zeroPoint = i;
    }
    if (TrajectorySampleTimes[0] >= 0.0f ||
        TrajectorySampleTimes[pointCount - 1] <= 0.0f ||
        zeroPoint < 0)
    {
        error = TEXT("TrajectorySampleTimes must span past and future around time zero.");
        return false;
    }

    layout.PoseBoneCount = boneCount;
    layout.ContactCount = contactCount;
    layout.TrajectoryPointCount = pointCount;
    layout.PoseFeatureSize = boneCount * FeatureLayout::PoseFloatsPerBone + contactCount;
    layout.TrajectoryFeatureSize = pointCount * FeatureLayout::TrajectoryFloatsPerPoint;
    layout.ZeroTimePoint = zeroPoint;
    layout.PoseBoneNames.Resize(boneCount);
    layout.PoseBoneGroupLabels.Resize(boneCount);
    for (int32 i = 0; i < boneCount; i++)
    {
        layout.PoseBoneNames[i] = PoseBones[i].Node;
        layout.PoseBoneGroupLabels[i] = PoseBones[i].Group.HasChars() ? PoseBones[i].Group : PoseBones[i].Node;
    }

    layout.ContactPoseBones.Resize(contactCount);
    for (int32 c = 0; c < contactCount; c++)
    {
        const String& node = ContactBones[c].Node;
        int32 bone = -1;
        for (int32 i = 0; i < boneCount; i++)
        {
            if (PoseBones[i].Node == node)
            {
                bone = i;
                break;
            }
        }
        if (node.IsEmpty() || bone < 0)
        {
            error = String::Format(TEXT("Schema.ContactBones[{}] ('{}') must name one of Schema.PoseBones: the planted flag uses that bone's position and velocity."), c, node);
            layout.Clear();
            return false;
        }
        for (int32 j = 0; j < c; j++)
        {
            if (layout.ContactPoseBones[j] == bone)
            {
                error = String::Format(TEXT("Schema.ContactBones declares node '{}' twice."), node);
                layout.Clear();
                return false;
            }
        }
        layout.ContactPoseBones[c] = bone;
    }

    // Normalization groups come from the declared labels, never from bone
    // names or indices: a solo bone (empty label) gets its own id, a label
    // gets one id at its first use. Positions first, then velocities
    // (separate ids: a position and a velocity never share a deviation),
    // then one id shared by every contact flag. The reference biped
    // declaration (pelvis, Foot, Foot, spine, head, Hand, Hand + two foot
    // contacts) yields positions 0,1,1,2,3,4,4, velocities 5,6,6,7,8,9,9 and
    // contacts 10.
    layout.PoseGroups.Resize(layout.PoseFeatureSize);
    int32 nextGroup = 0;
    Array<int32> boneGroup;
    boneGroup.Resize(boneCount);
    for (int32 block = 0; block < 2; block++)
    {
        for (int32 i = 0; i < boneCount; i++)
        {
            const String& label = PoseBones[i].Group;
            int32 group = -1;
            if (label.HasChars())
            {
                for (int32 j = 0; j < i; j++)
                {
                    if (PoseBones[j].Group == label)
                    {
                        group = boneGroup[j];
                        break;
                    }
                }
            }
            if (group < 0)
                group = nextGroup++;
            boneGroup[i] = group;
            const int32 offset = block * boneCount * 3 + i * 3;
            layout.PoseGroups[offset] = group;
            layout.PoseGroups[offset + 1] = group;
            layout.PoseGroups[offset + 2] = group;
        }
    }
    if (contactCount > 0)
    {
        const int32 contactGroup = nextGroup++;
        for (int32 c = 0; c < contactCount; c++)
            layout.PoseGroups[layout.GetContactFeatureOffset(c)] = contactGroup;
    }

    if (!layout.Validate(error))
    {
        layout.Clear();
        return false;
    }
    return true;
}

bool PoseSearchSchema::TryResolveNodes(SkinnedModel* model, NodeIndices& result, String& error) const
{
    result = NodeIndices();
    error = String::Empty;
    if (model == nullptr || model->WaitForLoaded())
    {
        error = TEXT("the skinned model is not loaded.");
        return false;
    }
    if (RootBone.IsEmpty() || PoseBones.IsEmpty())
    {
        error = TEXT("the schema declares no feature layout. "
            "Declare RootBone and PoseBones for your rig (scene -> MotionMatchingController -> Schema).");
        return false;
    }

    result.Root = model->FindNode(RootBone);
    result.PoseBones.Resize(PoseBones.Count());
    String missing;
    auto append = [&missing](const String& name)
    {
        if (missing.HasChars())
            missing += TEXT(", ");
        missing += name.HasChars() ? name : String(TEXT("<empty>"));
    };
    if (result.Root < 0)
        append(RootBone);
    for (int32 i = 0; i < PoseBones.Count(); i++)
    {
        result.PoseBones[i] = model->FindNode(PoseBones[i].Node);
        if (result.PoseBones[i] < 0)
            append(PoseBones[i].Node);
    }
    if (missing.HasChars())
    {
        error = String::Format(
            TEXT("declared skeleton nodes are missing in model '{}': {}. "
                "Fix Schema.RootBone/PoseBones for this rig (a declared bone is never baked as zeros)."),
            model->GetPath(), missing);
        result = NodeIndices();
        return false;
    }
    return true;
}

void PoseSearchSchema::ApplyLegacySkeleton(const LegacySkeletonNames& legacy)
{
    RootBone = legacy.Root;
    PoseBones.Clear();
    ContactBones.Clear();
    auto addBone = [this](const String& node, const Char* group)
    {
        if (node.IsEmpty())
            return;
        PoseFeatureBone bone;
        bone.Node = node;
        bone.Group = String(group);
        PoseBones.Add(bone);
    };
    // The v2-v4 fixed order and groups: pelvis, feet ("Foot"), spine, head,
    // hands ("Hand"); contacts on the two feet.
    addBone(legacy.Pelvis, TEXT(""));
    addBone(legacy.LeftFoot, TEXT("Foot"));
    addBone(legacy.RightFoot, TEXT("Foot"));
    addBone(legacy.Spine, TEXT(""));
    addBone(legacy.Head, TEXT(""));
    addBone(legacy.LeftHand, TEXT("Hand"));
    addBone(legacy.RightHand, TEXT("Hand"));
    auto addContact = [this](const String& node)
    {
        if (node.IsEmpty())
            return;
        ContactFeatureBone contact;
        contact.Node = node;
        ContactBones.Add(contact);
    };
    addContact(legacy.LeftFoot);
    addContact(legacy.RightFoot);
}

bool PoseSearchSchema::TryGetNormalizedUpAxis(Float3 &up) const
{
    up = RootUpAxis;
    if (!IsFinite(up) || up.LengthSquared() <= MinimumAxisLengthSquared)
    {
        up = Float3::Zero;
        return false;
    }
    up.Normalize();
    return true;
}

float PoseSearchSchema::ComputeContactHeight(
    const Float3 &footPosition,
    const Float3 &rootPosition,
    const Float3 &normalizedUpAxis)
{
    const Float3 relativeFoot(
        footPosition.X - rootPosition.X,
        footPosition.Y - rootPosition.Y,
        footPosition.Z - rootPosition.Z);
    return Float3::Dot(relativeFoot, normalizedUpAxis);
}

bool PoseSearchSchema::IsPlantedContact(float speedSquared, float maxSpeed, float height, float maxHeight)
{
    const float maxSpeedSq = maxSpeed * maxSpeed;
    return speedSquared <= maxSpeedSq && height <= maxHeight;
}

bool PoseSearchSchema::TryGetRightAxis(Float3 &right) const
{
    Float3 forward = RootForwardAxis;
    Float3 up = RootUpAxis;
    if (forward.LengthSquared() <= MinimumAxisLengthSquared ||
        up.LengthSquared() <= MinimumAxisLengthSquared)
    {
        right = Float3::Zero;
        return false;
    }
    forward.Normalize();
    up.Normalize();
    right = Float3::Cross(forward, up);
    if (right.LengthSquared() <= MinimumAxisLengthSquared)
    {
        right = Float3::Zero;
        return false;
    }
    right.Normalize();
    return true;
}

bool PoseSearchSchema::operator==(const PoseSearchSchema &other) const
{
    if (TrajectorySampleTimes.Count() != other.TrajectorySampleTimes.Count())
        return false;

    for (int i = 0; i < TrajectorySampleTimes.Count(); i++)
        if (TrajectorySampleTimes[i] != other.TrajectorySampleTimes[i])
            return false;

    if (BoneAliases.Count() != other.BoneAliases.Count())
        return false;
    for (int32 i = 0; i < BoneAliases.Count(); i++)
        if (!(BoneAliases[i] == other.BoneAliases[i]))
            return false;

    if (PoseBones.Count() != other.PoseBones.Count() ||
        ContactBones.Count() != other.ContactBones.Count())
        return false;
    for (int32 i = 0; i < PoseBones.Count(); i++)
        if (!(PoseBones[i] == other.PoseBones[i]))
            return false;
    for (int32 i = 0; i < ContactBones.Count(); i++)
        if (!(ContactBones[i] == other.ContactBones[i]))
            return false;

    return SampleRate == other.SampleRate &&
           Float3::NearEqual(Float3::Normalize(RootForwardAxis), Float3::Normalize(other.RootForwardAxis), AxisTolerance) &&
           Float3::NearEqual(Float3::Normalize(RootUpAxis), Float3::Normalize(other.RootUpAxis), AxisTolerance) &&
           PoseWeight == other.PoseWeight &&
           TrajectoryWeight == other.TrajectoryWeight &&
           ContactMaxSpeed == other.ContactMaxSpeed &&
           ContactMaxHeight == other.ContactMaxHeight &&
           RootBone == other.RootBone;
}

namespace
{
    // Bounded: a corrupt count must not allocate unbounded memory. A count
    // outside [0, limit] clears ok; the caller stops reading and reports the
    // failure (Deserialize returns false, which fails the load or upgrade).
    int32 ReadBoundedCount(ReadStream& stream, int32 limit, bool& ok)
    {
        int32 count = 0;
        stream.ReadInt32(&count);
        if (stream.HasError() || count < 0 || count > limit)
        {
            ok = false;
            return 0;
        }
        return count;
    }

    void WriteAliases(WriteStream& stream, const Array<BoneAlias>& aliases)
    {
        stream.WriteInt32(aliases.Count());
        for (const BoneAlias& alias : aliases)
        {
            stream.Write(alias.Channel);
            stream.Write(alias.Node);
        }
    }

    bool ReadAliases(ReadStream& stream, Array<BoneAlias>& aliases)
    {
        aliases.Clear();
        bool ok = true;
        const int32 count = ReadBoundedCount(stream, 4096, ok);
        for (int32 i = 0; i < count && !stream.HasError(); i++)
        {
            BoneAlias alias;
            stream.Read(alias.Channel);
            stream.Read(alias.Node);
            aliases.Add(alias);
        }
        return ok && !stream.HasError();
    }

    void WriteLegacySkeleton(WriteStream& stream, const LegacySkeletonNames& legacy)
    {
        stream.Write(legacy.Root);
        stream.Write(legacy.Pelvis);
        stream.Write(legacy.Spine);
        stream.Write(legacy.Head);
        stream.Write(legacy.LeftFoot);
        stream.Write(legacy.RightFoot);
        stream.Write(legacy.LeftHand);
        stream.Write(legacy.RightHand);
    }

    void ReadLegacySkeleton(ReadStream& stream, LegacySkeletonNames& legacy)
    {
        stream.Read(legacy.Root);
        stream.Read(legacy.Pelvis);
        stream.Read(legacy.Spine);
        stream.Read(legacy.Head);
        stream.Read(legacy.LeftFoot);
        stream.Read(legacy.RightFoot);
        stream.Read(legacy.LeftHand);
        stream.Read(legacy.RightHand);
    }
}

void PoseSearchSchema::Serialize(WriteStream &stream) const
{
    // v5: the declared layout replaces the eight named skeleton roles of v2-v4.
    stream.WriteInt32(SampleRate);
    stream.Write(RootForwardAxis);
    stream.Write(RootUpAxis);
    stream.WriteFloat(PoseWeight);
    stream.WriteFloat(TrajectoryWeight);
    stream.Write(TrajectorySampleTimes);
    stream.WriteFloat(ContactMaxSpeed);
    stream.WriteFloat(ContactMaxHeight);
    WriteAliases(stream, BoneAliases);
    stream.Write(RootBone);
    stream.WriteInt32(PoseBones.Count());
    for (const PoseFeatureBone& bone : PoseBones)
    {
        stream.Write(bone.Node);
        stream.Write(bone.Group);
    }
    stream.WriteInt32(ContactBones.Count());
    for (const ContactFeatureBone& contact : ContactBones)
        stream.Write(contact.Node);
}

bool PoseSearchSchema::Deserialize(ReadStream &stream)
{
    stream.ReadInt32(&SampleRate);
    stream.Read(RootForwardAxis);
    stream.Read(RootUpAxis);
    stream.ReadFloat(&PoseWeight);
    stream.ReadFloat(&TrajectoryWeight);
    stream.Read(TrajectorySampleTimes);
    stream.ReadFloat(&ContactMaxSpeed);
    stream.ReadFloat(&ContactMaxHeight);
    PoseBones.Clear();
    ContactBones.Clear();
    if (!ReadAliases(stream, BoneAliases))
        return false;
    stream.Read(RootBone);
    bool ok = true;
    const int32 boneCount = ReadBoundedCount(stream, 4096, ok);
    if (!ok)
        return false;
    for (int32 i = 0; i < boneCount && !stream.HasError(); i++)
    {
        PoseFeatureBone bone;
        stream.Read(bone.Node);
        stream.Read(bone.Group);
        PoseBones.Add(bone);
    }
    const int32 contactCount = ReadBoundedCount(stream, 4096, ok);
    if (!ok)
        return false;
    for (int32 i = 0; i < contactCount && !stream.HasError(); i++)
    {
        ContactFeatureBone contact;
        stream.Read(contact.Node);
        ContactBones.Add(contact);
    }
    return !stream.HasError();
}

void PoseSearchSchema::SerializeV4(WriteStream &stream, const LegacySkeletonNames& legacy) const
{
    // v4: the v3 fields in their original order, then the contact gate and aliases.
    SerializeV3(stream, legacy);
    stream.WriteFloat(ContactMaxSpeed);
    stream.WriteFloat(ContactMaxHeight);
    WriteAliases(stream, BoneAliases);
}

bool PoseSearchSchema::DeserializeV4(ReadStream &stream, LegacySkeletonNames& legacy)
{
    DeserializeV3(stream, legacy);
    stream.ReadFloat(&ContactMaxSpeed);
    stream.ReadFloat(&ContactMaxHeight);
    return ReadAliases(stream, BoneAliases);
}

void PoseSearchSchema::SerializeV3(WriteStream &stream, const LegacySkeletonNames& legacy) const
{
    stream.WriteInt32(SampleRate);
    stream.Write(RootForwardAxis);
    stream.Write(RootUpAxis);
    stream.WriteFloat(PoseWeight);
    stream.WriteFloat(TrajectoryWeight);
    WriteLegacySkeleton(stream, legacy);
    stream.Write(TrajectorySampleTimes);
}

void PoseSearchSchema::DeserializeV3(ReadStream &stream, LegacySkeletonNames& legacy)
{
    stream.ReadInt32(&SampleRate);
    stream.Read(RootForwardAxis);
    stream.Read(RootUpAxis);
    stream.ReadFloat(&PoseWeight);
    stream.ReadFloat(&TrajectoryWeight);
    ReadLegacySkeleton(stream, legacy);
    stream.Read(TrajectorySampleTimes);
}
