# Authoring and baking

## Schema and rig

Configure `PoseSearchSchema` for the consuming project's rig. Root forward and
up axes must be finite, non-zero, and non-parallel. The schema also declares
the feature layout: which bones the pose features read, which of them carry a
contact flag, and how many trajectory points the query compares.

## Declaring the feature layout

The layout is data, not code. Four schema fields declare it:

| Field | Meaning |
|---|---|
| `RootBone` | Node that carries root motion. Required. Never a pose bone. |
| `PoseBones` | Ordered list of `{Node, Group}`. 1 to 64 entries, unique nodes. Each bone adds position xyz and velocity xyz (6 floats) in root space. |
| `ContactBones` | Ordered list of `{Node}`. 0 to 16 entries; each names a `PoseBones` node and adds one contact flag. |
| `TrajectorySampleTimes` | 1 to 32 strictly ascending times that span past and future around 0. Each time is one trajectory point (planar position xy, planar facing xy, yaw rate: 5 floats). |

From these the plugin derives the runtime layout (`Schema/FeatureLayout.h`):

```text
pose features       = PoseBones * 6 + ContactBones
                      [positions of every bone][velocities of every bone][contact flags]
trajectory features = TrajectorySampleTimes * 5
```

`Group` is an optional normalization label. Bones with the same non-empty
label share one normalization group (one deviation for positions and one for
velocities), so left and right limbs are scaled alike. An empty label gives
the bone its own groups. Group ids are assigned in first-seen order:
positions first, then velocities, then one group for all contact flags.

Example: a biped that matches pelvis, both feet, spine, head and both hands,
with contacts on the feet (this is the layout of format v4 and of the
flax-test scenes, 44 pose floats and 30 trajectory floats):

```text
RootBone      root
PoseBones     pelvis "", foot_l "Foot", foot_r "Foot", spine "", head "",
              hand_l "Hand", hand_r "Hand"
ContactBones  foot_l, foot_r
SampleTimes   -0.4, -0.2, 0, 0.2, 0.4, 0.6
```

A quadruped could declare its hips, the four paws under one `"Paw"` label and
the head, with contacts on all four paws and eight trajectory points; nothing
in the plugin assumes two legs or six points.

Rules, all checked by `TryValidate` before a bake and at bind time:

- Every declared node (root, pose and contact bones, after `BoneAliases`)
  must exist in the bake model and in the bound model. A missing node is an
  error that lists the missing names; nothing is skipped silently.
- Every pose bone and the root must have an animation channel in at least one
  clip (see the coverage report below), or the bake fails. A clip that lacks a
  channel (importers drop constant tracks) keeps the bind pose for that node and
  is only reported.
- The layout is part of the baked snapshot and of `FeatureLayoutHash`.
  Changing any of the four fields needs a rebake; the controller refuses a
  database whose layout differs from the live schema.

The trajectory plane follows the schema axes, not a world axis: positions and
facings are measured along root right (forward x up) and root forward. The
ground plane used for yaw rate and clip turn angles is the bake model's root
up axis (`RootUpAxis` carried through the root's bind pose, snapped to a
cardinal axis when within 0.999), stored in the database as `PlanarUpAxis`.

`ContactMaxSpeed` (world units per second, default 300) and `ContactMaxHeight`
(world units above the root, default 12) gate the foot contact flags: a foot
counts as planted when it moves slower and sits lower than both limits. They
are stored in the database and shape the baked features.

Schema weights are query-time weights. Changing `PoseWeight` or
`TrajectoryWeight` does not require a rebake. Changes to feature layout,
sample rate, axes, contact limits, bone aliases, skeleton extraction, or
trajectory sampling do require a new bake and a compatible database snapshot.
The controller refuses to bind when the live schema differs from the baked
snapshot in any of these (see `MIGRATION.md`, bind-time checks).

## Clip registry

`MotionMatchingEditorService` provides generic registry operations:

- `AddClips` and `AddClipFiles` register authored tags, loop flags and a
  clip group.
- `ExportClipRegistry` writes deterministic, versioned JSON with content-
  relative paths, normalized tags, loop values and groups.
- `ImportClipRegistry` replaces the whole registry, groups entries by sorted
  tag set, loop value and clip group, and reports skipped entries.
- `ClipRegistryImportOptions` is the explicit extension point for a host path
  resolver or tag/loop/group inference (`InferTags`, `InferLoop`,
  `InferGroup`). The generic editor has no folder, filename, humanoid, or
  project-specific defaults.

Registry JSON has this shape:

```json
{
  "version": 1,
  "clips": [
    {"path": "Animations/Walk.flax", "tags": ["walk"], "loop": true, "group": 1}
  ]
}
```

Paths may be Content-relative or absolute on import. Tags are lowercased,
trimmed, and de-duplicated by the database. A missing loop value defaults to
`false` unless `InferLoop` is explicitly supplied, and a missing group is `0`
(any) unless `InferGroup` is supplied. An entry without tags is skipped unless
`InferTags` supplies them. The group is what `MotionMatchingFrameInput.ClipGroup`
filters one-shot clips on; the host defines the ids.

## Bake and validate

Use `BakeDatabase`, or the combined `BakeValidateAndTest`, after the registry
and schema are configured. The editor service reloads the database after edits.
Run `ValidatePersistence` and the editor-only search self-test before shipping
the asset. `GenerateClipAudit` is an explicit diagnostic command and writes
`Cache/MMClipAudit.txt` only when requested. Its ten columns are index, clip
name, tag set, loop/once, clip group, root travel, net displacement, turn
angle, duration and authored speed.

## Skeletons, bone aliases and retargeting

The baker and the runtime sampler resolve animation channels to model nodes
with one shared resolver (`Runtime/MotionMatchingChannelMap.h`): a channel
drives the node with the same name, or the node its `Schema.BoneAliases`
entry names. The first channel that reaches a node wins; nodes without a
channel keep their bind pose.

Bone aliases only rename. Use them when the clips were made for a skeleton
with the same proportions and joint orientations but other bone names (for
example `foot_l` against `CC_Base_L_Foot`). Each alias maps one clip channel
name to one model node name; channel names must be unique.

Before every bake the baker logs a channel coverage report: channels without a
node, nodes without a channel and alias suggestions. The bake fails when the
schema's `RootBone` or any `PoseBones` node has no channel in any clip; a node
that only some clips lack keeps its bind pose there and appears as a NOTE. Suggestions pair an unresolved channel with the one undriven node
that has the same normalized name (case, separators, digits, a dominant rig
prefix such as `CC_Base_` or `mixamorig:`, and left/right markers are
ignored). Review them before adding them to the schema. The same report is
available without baking: `MotionMatchingEditorService.DescribeChannelCoverage`
and `SuggestBoneAliases`, or the "Suggest Bone Aliases" button of the register
clips window.

The bake also stores a skeleton fingerprint: the driven nodes and their
ancestors with their parents and local offsets, taken from the clips where
they are constant and from the bake model otherwise. When the controller
binds, the body's model must have every fingerprint node with the same parent
and offsets within 5% (nodes whose offset is animated are not compared). The
bake logs a warning when the clips' own offsets already differ from the bake
model by more than 5%.

When the proportions or joint orientations differ, aliases are not enough:
the poses would play on bones of another length and axis. Retarget the clips
offline onto the target skeleton (for example with fbx-animation-retargeter),
import them, register the retargeted clips and bake. Flax binds a skinned model
to the pose stored in its FBX file, so measure and retarget against that
stored pose, not the modelling rest pose. `AllowBindingMismatch` on the
controller binds a refused database anyway, for debugging only.
