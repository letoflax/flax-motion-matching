# Migration and asset identity

## Namespaces and the database TypeName

The public scripting namespace is `MotionMatching`. Since database format v4
the serialized binary identity matches it:

```text
MotionMatching.MotionMatchingDatabase   (format v4, written by every save)
Game.MotionMatchingDatabase             (legacy key of format v1-v3 files)
```

The asset factory registers both keys, so a v1-v3 file still loads. In the
editor the asset upgrader converts it in place (`Upgrade_1_To_2`,
`Upgrade_2_To_3`, `Upgrade_3_To_4`, chained) and `Upgrade_3_To_4` rewrites the
header TypeName to `MotionMatching.MotionMatchingDatabase`. Do not hand-edit
asset headers, database JSON or scene text to change the key; let the
upgrader (or a save from the editor) do it.

The legacy key does not mean the plugin runtime depends on a game assembly. It
is only a binary asset factory key that stays registered for old files.

## Format v4 upgrade (v3 -> v4)

What `Upgrade_3_To_4` keeps and what it drops:

- Kept: the clip registry (source animations, tag sets, loop flags) and the
  schema values of v3 (sample rate, weights, axes, sample times, skeleton).
- Recovered: the per-source clip `group`. v3 derived a speed band from the
  asset file name at bake time; the upgrader copies each baked band onto the
  registry entry with the same animation id, so a plain rebake keeps the same
  one-shot group filter. Sources that were never baked get group 0 (any).
  The log line `upgraded to v4: kept N registered clips (M groups recovered
  ...)` reports both counts.
- Added with defaults: `ContactMaxSpeed` 300, `ContactMaxHeight` 12 (the old
  hardcoded contact gate), empty `BoneAliases`, empty skeleton fingerprint.
- Dropped: all baked data. The schema layout, the clip metadata (turn-like
  flag and file-name bands replaced by authored groups) and the provenance
  contract changed (`SchemaContractRevision` 3, `BakeToolBuildId` 2), so the
  database is registry-only until it is rebaked.

After the upgrade, rebake once. The pose and trajectory features do not
change for the same clips and schema values (`featuresHash` stays equal);
`clipMetadataHash` changes because it now covers the clip groups instead of
the file-name turn-like flag.

The upgrade only runs in the editor. A cooked game needs a v4 asset, so open
and rebake (or save) every database in the editor before cooking.

## Clip groups replace file-name bands

The plugin no longer reads file names. Each registry entry carries an optional
integer `group` (0 or absent = any). The host decides what the ids mean and
sends the wanted group in `MotionMatchingFrameInput.ClipGroup`; one-shot clips
of another non-zero group are skipped, loops and group-0 clips never are. A
registry exported before v4 has no `group`; importing it gives group 0 unless
the host passes `ClipRegistryImportOptions.InferGroup`. To reproduce the old
behaviour exactly, write the groups from the old rule (flax-test does this
with `Tools/mm_clip_groups.py`, the same rule as its game-side
`GaspClipConventions.InferGroup`):

```text
"to_Sprint" 3, "to_Run" 2, "to_Walk" 1, "_Sprint_" 3, "_Run_" 2, "_Walk_" 1,
else 0 (case-sensitive substring of the asset path, first match wins)
```

`MotionMatchingFrameInput.CandidateBand` is now `ClipGroup` (same values and
filter semantics). The trace keeps its JSON key `band` for this field.

## Bind-time checks

A v4 database stores a skeleton fingerprint of the bake model. When the
controller binds, the live schema must equal the baked snapshot (weights
excepted) and the body's model must pass the fingerprint check (same driven
bone names and parents, bone offsets within 5%). A mismatch is an error: the
controller stays not ready and `BindingReport` names the cause and the fix.
`AllowBindingMismatch = true` binds anyway with a warning (debug only).

## Moving a host project

1. Add the plugin project reference and target module dependencies described in
   `INTEGRATION.md`.
2. Copy the database asset into the new project's Content tree or update the
   host's `DatabasePath` to its new Content-relative location.
3. Open the project in the editor once so older databases upgrade to v4, then
   check that the asset loads as `MotionMatchingDatabase`.
4. Recreate the host schema for the destination rig; do not assume a schema
   snapshot from another rig is compatible. Renamed bones go in
   `Schema.BoneAliases`; other proportions need an offline retarget (see
   `AUTHORING.md`).
5. Export/import the clip registry when source paths change. Registry paths are
   authoring data and can be resolved by `ClipRegistryImportOptions`.
6. Rebake when schema contract, feature layout, sampling, rig extraction, or
   source animation metadata changes. A stale database is registry-only until
   it is rebaked; a structurally corrupt database must not be treated as valid.

## Legacy policy

The public migration guarantee is limited to loading and upgrading older
database assets and the documented runtime/editor APIs. Host scene components
and project-specific providers remain host-owned and must be migrated by the
host. The plugin does not infer old game player types, sample folders, or
filename conventions during import.
