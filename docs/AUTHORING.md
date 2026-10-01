# Authoring and baking

## Schema and rig

Configure `PoseSearchSchema` for the consuming project's rig. `Root`,
`Pelvis`, `LeftFoot`, and `RightFoot` are required; additional named bones are
stored for the declared feature layout. Root forward and up axes must be finite,
non-zero, and non-parallel. The default trajectory times are six ascending
values from past through future and must include time zero.

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
schema's `Root`, `Pelvis`, `LeftFoot` or `RightFoot` node has no channel in
some clip. Suggestions pair an unresolved channel with the one undriven node
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
