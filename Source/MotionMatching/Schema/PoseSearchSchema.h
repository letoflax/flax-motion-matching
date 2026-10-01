#pragma once

#include "Engine/Core/Collections/Array.h"
#include "Engine/Core/Math/Vector3.h"
#include "Engine/Core/Types/String.h"
#include "FeatureLayout.h"
#include "MotionMatchingTypes.h"

class ReadStream;
class WriteStream;

/// <summary>
/// Named skeleton roles of database formats v2-v4 (the former
/// SkeletonProfile). Upgrader-only; the live schema declares its layout with
/// RootBone, PoseBones and ContactBones.
/// </summary>
struct LegacySkeletonNames
{
    String Root;
    String Pelvis;
    String Spine;
    String Head;
    String LeftFoot;
    String RightFoot;
    String LeftHand;
    String RightHand;
};

/// <summary>
/// Bake contract shared by the baker and the runtime query builder.
/// A snapshot of the schema used for baking is stored inside MotionMatchingDatabase,
/// so a database never depends on scene-side settings to be interpreted.
/// </summary>
API_STRUCT(Namespace="MotionMatching")
struct MOTIONMATCHING_API PoseSearchSchema
{
    DECLARE_SCRIPTING_TYPE_MINIMAL(PoseSearchSchema);

    /// <summary>Samples per second baked for every clip.</summary>
    API_FIELD(Attributes="Limit(1, 240)") int32 SampleRate = 30;

    /// <summary>Root-node local axis that points forward.</summary>
    API_FIELD() Float3 RootForwardAxis = Float3(0, -1, 0);

    /// <summary>Root-node local axis that points up.</summary>
    API_FIELD() Float3 RootUpAxis = Float3(0, 0, -1);

    /// <summary>
    /// Relative importance of the pose block in the search cost.
    /// Design note: applied at query/cost time only; it is never baked into
    /// features, so tuning it does not require a re-bake.
    /// </summary>
    API_FIELD(Attributes="Limit(0, 100)") float PoseWeight = 1.0f;

    /// <summary>
    /// Relative importance of the trajectory block in the search cost.
    /// Design note: applied at query/cost time only; it is never baked into
    /// features, so tuning it does not require a re-bake.
    /// </summary>
    API_FIELD(Attributes="Limit(0, 100)") float TrajectoryWeight = 1.0f;

    /// <summary>
    /// Root node of the model: origin and frame of every feature (positions,
    /// velocities and trajectory are expressed relative to it). Empty by
    /// default: the host MUST configure the layout for its own rig.
    /// </summary>
    API_FIELD() String RootBone;

    /// <summary>
    /// Pose feature bones, in feature order (1..64). Each adds a root-relative
    /// position and velocity. Bones sharing a Group label share one
    /// normalization deviation. Every declared node must exist in the model.
    /// Changing the list requires a rebake.
    /// </summary>
    API_FIELD() Array<PoseFeatureBone> PoseBones;

    /// <summary>
    /// Foot-contact features (0..16), each the planted flag of one pose bone
    /// (slow and low, see ContactMaxSpeed/ContactMaxHeight). Order is the
    /// feature order. Changing the list requires a rebake.
    /// </summary>
    API_FIELD() Array<ContactFeatureBone> ContactBones;

    /// <summary>
    /// Trajectory sampling times in seconds relative to the current pose. One
    /// trajectory point per value (up to 32), strictly ascending, spanning
    /// past and future and including time zero (for example -0.4 .. 0.6).
    /// The host provides one TrajectoryPoint per value.
    /// </summary>
    API_FIELD() Array<float> TrajectorySampleTimes = { -0.4f, -0.2f, 0.0f, 0.2f, 0.4f, 0.6f };

    /// <summary>
    /// Foot-contact gate, speed part, in world units per second (the rig's
    /// length unit per second; centimeters on the default rigs). A foot counts
    /// as planted while its root-relative velocity is at or below this speed
    /// AND its height is at or below ContactMaxHeight. Baked into the contact
    /// features and reused by the live query, so changing it requires a rebake.
    /// </summary>
    API_FIELD(Attributes="Limit(0)") float ContactMaxSpeed = 300.0f;

    /// <summary>
    /// Foot-contact gate, height part, in world units: the foot height above
    /// the root, measured along RootUpAxis. See ContactMaxSpeed.
    /// </summary>
    API_FIELD(Attributes="Limit(0)") float ContactMaxHeight = 12.0f;

    /// <summary>
    /// Bone aliases: clip channel name to model node name, applied by the one
    /// shared channel resolver at bake time and at runtime sampling. Aliases
    /// only rename. Clips authored for a skeleton with other proportions or
    /// another rest pose must be retargeted offline first; the database
    /// records a skeleton fingerprint and refuses to bind to a model that does
    /// not match it. Changing aliases requires a rebake.
    /// </summary>
    API_FIELD() Array<BoneAlias> BoneAliases;

    static constexpr float AxisTolerance = 0.001f;       // tolerance for axis validation
    // Normalization revision: bumped when the bake-time statistics change
    // shape. Stored per-database (NOT in the schema snapshot); databases
    // baked with an older revision load registry-only with a "rebake
    // required" message instead of silently mismatching.
    // Rev 1 = pooled grouped (mean/std over samples x grouped dims).
    // Rev 2 = scalar per-dim mean/population-variance, baked z-scored
    // features + stored means.
    // Rev 3 = NO means, raw baked/query values, one mean-row-norm
    // deviation per group floored at `dev > 0.1 ? dev : 1.0`; the search
    // applies sqrt(sum-normalized weight)/deviation once at cost time.
    static constexpr int32 NormalizationRevision = 3;

    /// <summary>
    /// Derives the runtime feature layout from the declaration: pose bone
    /// count and order, contacts (each mapped to its pose bone), trajectory
    /// point count (one per TrajectorySampleTimes entry) and normalization
    /// groups. Group ids: positions first (a new id per solo bone or per
    /// first use of a label), then velocities the same way, then one shared
    /// id for all contacts. Fails with an actionable error when the
    /// declaration is invalid.
    /// </summary>
    bool TryBuildLayout(FeatureLayout& layout, String& error) const;

    /// <summary>
    /// Resolves RootBone and every pose bone against a model. A declared node
    /// that the model does not have is an error (never baked as zeros).
    /// </summary>
    bool TryResolveNodes(SkinnedModel* model, NodeIndices& result, String& error) const;

    /// <summary>
    /// Checks sample rate, axes, weights, the declared layout (root, pose
    /// bones, contacts, trajectory times), contact gate and bone aliases.
    /// </summary>
    bool TryValidate(String& error) const;

    /// <summary>
    /// Returns the normalized right axis (forward x up) or false when axes are degenerate.
    /// </summary>
    bool TryGetRightAxis(Float3& right) const;

    /// <summary>
    /// C1 shared contact helper: normalized root-up axis, or false when the
    /// schema up axis is zero/non-finite. Baker, live query and playback all
    /// resolve the axis through this (baker normalizes on snapshot; callers
    /// with a live schema normalize here) so contact height never hardcodes .Y.
    /// </summary>
    bool TryGetNormalizedUpAxis(Float3& up) const;

    /// <summary>
    /// C1 shared contact helper: height = dot(footPosition - rootPosition,
    /// normalizedUpAxis). footPosition and rootPosition must be in the SAME
    /// space (baker/query: root-relative with root at origin; playback:
    /// model/body space from the composed pose).
    /// </summary>
    static float ComputeContactHeight(
        const Float3& footPosition,
        const Float3& rootPosition,
        const Float3& normalizedUpAxis);

    /// <summary>
    /// C1 shared contact gate: planted while slow and low. Speed is passed
    /// squared to avoid a sqrt on the hot path. Bake and query pass the
    /// root-relative pose velocity and the schema's ContactMaxSpeed /
    /// ContactMaxHeight (the query reads them from the baked snapshot).
    /// </summary>
    static bool IsPlantedContact(float speedSquared, float maxSpeed, float height, float maxHeight);

    bool operator==(const PoseSearchSchema& other) const;
    bool operator!=(const PoseSearchSchema& other) const
    {
        return !(*this == other);
    }

    /// <summary>
    /// Database format v5 layout: SampleRate, axes, weights,
    /// TrajectorySampleTimes, contact gate, BoneAliases, RootBone, PoseBones,
    /// ContactBones. Deserialize returns false when the data is corrupt (a
    /// list count out of range or a stream error); the caller must stop.
    /// </summary>
    void Serialize(WriteStream& stream) const;
    bool Deserialize(ReadStream& stream);

    /// <summary>
    /// Legacy layouts used only by the database upgraders. v2/v3: no contact
    /// gate, no aliases, eight named skeleton roles. v4: v3 + contact gate +
    /// aliases. The legacy role names travel in LegacySkeletonNames;
    /// ApplyLegacySkeleton converts them into the v5 declaration.
    /// </summary>
    void SerializeV3(WriteStream& stream, const LegacySkeletonNames& legacy) const;
    void DeserializeV3(ReadStream& stream, LegacySkeletonNames& legacy);
    void SerializeV4(WriteStream& stream, const LegacySkeletonNames& legacy) const;
    bool DeserializeV4(ReadStream& stream, LegacySkeletonNames& legacy);

    /// <summary>
    /// The fixed v2-v4 layout as a declaration: Root; pose bones pelvis,
    /// left foot, right foot, spine, head, left hand, right hand (feet share
    /// group "Foot", hands share group "Hand", empty names skipped); contacts
    /// left foot, right foot. Reproduces the v4 feature layout exactly when
    /// all eight names are set.
    /// </summary>
    void ApplyLegacySkeleton(const LegacySkeletonNames& legacy);
};
