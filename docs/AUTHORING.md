# Authoring and baking

## Schema and rig

Configure `PoseSearchSchema` for the consuming project's rig. `Root`,
`Pelvis`, `LeftFoot`, and `RightFoot` are required; additional named bones are
stored for the declared feature layout. Root forward and up axes must be finite,
non-zero, and non-parallel. The default trajectory times are six ascending
values from past through future and must include time zero.

Schema weights are query-time weights. Changing `PoseWeight` or
`TrajectoryWeight` does not require a rebake. Changes to feature layout,
sample rate, axes, skeleton extraction, or trajectory sampling do require a
new bake and a compatible database snapshot.

## Clip registry

`MotionMatchingEditorService` provides generic registry operations:

- `AddClips` and `AddClipFiles` register authored tags and loop flags.
- `ExportClipRegistry` writes deterministic, versioned JSON with content-
  relative paths, normalized tags, and loop values.
- `ImportClipRegistry` replaces the whole registry, groups entries by sorted
  tag set and loop value, and reports skipped entries.
- `ClipRegistryImportOptions` is the explicit extension point for a host path
  resolver or tag/loop inference. The generic editor has no folder, filename,
  humanoid, or project-specific defaults.

Registry JSON has this shape:

```json
{
  "version": 1,
  "clips": [
    {"path": "Animations/Walk.flax", "tags": ["walk"], "loop": true}
  ]
}
```

Paths may be Content-relative or absolute on import. Tags are lowercased,
trimmed, and de-duplicated by the database. A missing loop value defaults to
`false` unless `InferLoop` is explicitly supplied. An entry without tags is
skipped unless `InferTags` supplies them.

## Bake and validate

Use `BakeDatabase`, or the combined `BakeValidateAndTest`, after the registry
and schema are configured. The editor service reloads the database after edits.
Run `ValidatePersistence` and the editor-only search self-test before shipping
the asset. `GenerateClipAudit` is an explicit diagnostic command and writes
`Cache/MMClipAudit.txt` only when requested.
