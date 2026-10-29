# Changelog

All notable changes to the MotionMatching plugin. The release version number
is not decided yet (`MotionMatching.flaxproj` still says `1.0`); entries below
are grouped by the production milestone that introduced them.

## Version table

Values are read from the sources; update this table whenever one of them
changes.

| Item | Value | Where | Bump when |
|---|---|---|---|
| Plugin version | `1.0` (placeholder, release number pending) | `MotionMatching.flaxproj` `Version` | a release is cut |
| Minimum engine | `1.13.7012` (release 0.1.0: `1.12.6912`) | `MotionMatching.flaxproj` `MinEngineVersion` | the engine API surface used changes |
| Database `SerializedVersion` | `5` | `DECLARE_BINARY_ASSET_HEADER(MotionMatchingDatabase, 5)` in `Database/MotionMatchingDatabase.h`; upgrader chain `Upgrade_1_To_2` .. `Upgrade_4_To_5` | the binary layout changes (add an upgrader) |
| Database `TypeName` | `MotionMatching.MotionMatchingDatabase` (legacy key `Game.MotionMatchingDatabase` still loads) | `Database/MotionMatchingDatabase.cpp` | never, unless the asset identity moves |
| `SchemaContractRevision` | `4` | `MotionMatchingDatabase::SchemaContractRevision` | the schema contract or feature layout changes |
| `PoseSearchSchema::NormalizationRevision` | `3` | `Schema/PoseSearchSchema.h` | the normalization math changes |
| `BakeToolBuildId` | `3` | `MotionMatchingDatabase::BakeToolBuildId` | the baker output changes |
| Trace `SchemaVersion` | `3` (the importer also reads 1 and 2) | `Runtime/MotionMatchingTrace.h` | the trace JSON format changes |

A database whose stored revisions differ from the compiled ones loads as
registry-only (rebake required); see [MIGRATION.md](MIGRATION.md).

## Unreleased: M5 declared feature layout

### Root event window (2026-10-29)

- New per-clip root height events, derived at load / bake from the baked root positions along the planar up axis
  (never serialized, not hashed; no asset format change, no rebake): `MotionMatchingDatabase.GetClipRootTakeoffTime`
  (the last sample at the start height before the root rises more than 15 units) and `GetClipRootLandTime` (the first
  sample from which the root holds its end height, within 2 units for at least 0.2 s, after being more than 15 units
  above it), plus `GetClipRootTurnStartTime` (the last sample before the root yaw leaves its start value by more
  than 3 degrees, clips turning at least 20 degrees) and `GetClipRootStopTime` (the first sample from which the root
  holds its end position on the ground plane, within 2 units for at least 0.2 s, after more than 30 units of travel).
  Loops have none; `-1` = none.
- New search filter `MotionMatchingSearchSettings.RootEvent` / `RootEventTime` / `RootEventTolerance` (enum
  `MotionRootEvent`: `None`, `Takeoff`, `Landing`, `TurnStart`, `Stop`): with a kind and a positive tolerance, only candidates whose clip has
  that event at `RootEventTime` +- `RootEventTolerance` seconds ahead of the sample rank (negative = the event has
  passed). Brute force, indexed and top-candidate passes apply it alike. Default off, so existing hosts and the
  self-tests see no change; `featuresHash` / `clipMetadataHash` are unchanged.
- Host input: `MotionMatchingFrameInput.RootEvent` / `RootEventTime` / `RootEventTolerance`, passed through
  `MotionMatchingRuntimePolicy.SetRootEventWindow` each tick. A window that admits nothing in scope re-runs the same
  query without it (trace `retry` 3); reseed queries ignore it. The fast-path signature includes the kind.
- Trace v3 lines gain `rev` / `revt` / `revtol` (kind in `TraceFlags` bits 7-9, two new entry fields: 63 of the 64
  allowed); older lines import with the window off and replay unchanged.
- `MotionMatchingDatabase.HasClipTag(clip, tag)` (scripting API): tag-set membership of a baked clip, so a host can
  tell which clip of a scope it is playing.
- Search self-test Phase S: an exact query a little before a clip's event wins on that sample under the window, and an
  impossible window returns nothing.
- Restarts: under a window, the event samples of the playing one-shot are admitted (the one-shot progression guard and
  the reselect ban step aside, since the window admits nothing but event samples), and the policy seeks to such a
  same-clip winner (`seek:root-event`) instead of continuing. A host can so ask for the same turn clip twice in a row,
  or re-sync a drifting landing inside its clip.
- Reversal (2026-10-29, round 3): `MotionRootEvent.Reversal` (5) and `MotionMatchingDatabase.GetClipRootReversalTime`:
  in a one-shot whose planar root travel turns against its lead-in direction (mean velocity of samples 1-3 over 50
  units/s, then a sample moving the opposite way at over 50 units/s), the slowest sample between the last one still
  moving the lead-in way (over 25 units/s) and that one (a pivot). Derived at load like the other events (no rebake,
  hashes unchanged); the search filter, the self-test Phase S and the trace kind bits take it like the other kinds.
  Use: a host that drives its capsule by the clip's root through a pivot asks for the reversal a fixed lead ahead,
  so the pivot clip starts where its own braking does.
- Use: the host predicts its own touchdown (time to land) and asks for `Landing` at that time, so a landing clip starts
  where its touchdown meets the capsule's; after touchdown it passes the negative time since then.

### Query yaw-rate sign (2026-10-29)

- The live query's yaw-rate channel (trajectory float 4 of each point) had the opposite sign of the baked one. The baker
  measures yaw in the model ground plane (`atan2(x, z)` of the root forward); the query derived it from directions in the
  feature plane (`right = Cross(forward, up)`, `forward`), which is the mirror image of that plane. Every turn query
  therefore matched turn clips of the other direction (seen live: right turns in place picked `Turn_*_L` clips and the
  reverse, every time; the position and facing channels were consistent, so strafes and arcs were not affected).
  `MotionMatchingQuery` now negates the query yaw rate. No database change, no rebake: `featuresHash` /
  `clipMetadataHash` are unchanged; search 1388/1388 and playback 9/9 self-tests pass. Non-Y-up schemas use the same
  rule (unverified on a non-Y-up rig).

### Flax 1.13

- `MinEngineVersion` is `1.13.7012`. The native module and the C# shell build unchanged on Flax 1.13.7012 (Windows x64); no source change was needed. Release 0.1.0 stays on Flax 1.12.

### Breaking changes

- `PoseSearchSchema.Skeleton` (`SkeletonProfile`: Root, Pelvis, Spine, Head,
  LeftFoot, RightFoot, LeftHand, RightHand) is replaced by a declared layout:
  `RootBone`, `PoseBones` (`PoseFeatureBone {Node, Group}`, 1 to 64) and
  `ContactBones` (`ContactFeatureBone {Node}`, 0 to 16). The trajectory point
  count is the length of `TrajectorySampleTimes` (1 to 32, was exactly 6).
  See [AUTHORING.md](AUTHORING.md), declaring the feature layout.
- Database format v5, `SchemaContractRevision` 4, `BakeToolBuildId` 3. A v4
  file upgrades in place (`Upgrade_4_To_5` converts the roles into the
  equivalent declaration) and its baked data is dropped: rebake once. Host
  scenes that stored `Schema.Skeleton` must write the declaration themselves.
  See [MIGRATION.md](MIGRATION.md).
- A declared node (root, pose or contact bone, after aliases) missing from the
  bake or bound model is an error that names the missing nodes. The bake also
  requires a channel for every pose bone, not only root/pelvis/feet.
- Removed `SkeletonProfile`, `MotionMatchingSkeleton`, `FeatureContactRole`
  and the fixed layout statics of `PoseSearchSchema` (`PoseBoneCount`,
  `PoseExtraCount`, `TrajectoryPointCount`, `GetPoseFeatureSize()`,
  `GetPoseNormalizationGroup()`, `GetContactFeatureOffset()`,
  `GetFeatureLayoutHash()`, `ValidateLayout()`, ...); they live on
  `FeatureLayout` now. `NodeIndices` is now
  `{Root, PoseBones[]}` and is filled by `PoseSearchSchema::TryResolveNodes`.
- `MotionMatchingQuery.SampleLivePose` takes the schema (it replaces
  `SampleLivePoseWithSchema`; the schema-less overload is gone).
- `MotionMatchingPlayback::Setup(database, fallbackModel, rootBone)` takes the
  root node name instead of a skeleton profile.

### Added

- `FeatureLayout` (`Schema/FeatureLayout.h`), the runtime layout derived from
  the declaration: sizes, contact offsets, per-dim normalization groups
  (shared by bones with the same `Group` label, first-seen order), channel
  descriptions and `ComputeHash()`.
- Database properties `PoseBoneCount`, `ContactCount`,
  `TrajectoryPointCount`, `PlanarUpAxis`; `FeatureLayoutHash` is now a
  property.
- `PlanarUpAxis`: the bake model's ground-plane normal (`RootUpAxis` through
  the root bind pose, snapped to a cardinal axis), stored in the database and
  used for yaw rate and clip turn angles instead of a hard XZ plane.
- `fixtures/schema_default.json`: the reference declaration (the v4 biped
  layout) read by the host's offline tests instead of parsing C++ sources.

### Changed

- The search keeps a fixed-size fast path for the reference sizes (44 pose,
  30 trajectory floats) and a generic loop for any other declared layout.
- The default declaration reproduces the v4 features bit for bit: same bone
  order, groups, contact offsets and op order (`featuresHash` unchanged after
  rebake). `FeatureLayoutHash` values change because the hash now covers the
  declared channel names.

## Unreleased: production milestones M0 to M3

### Breaking changes

- `MotionMatchingFrameInput.CallerPlanarSpeed` is now `TargetSpeed`. It is used
  only for speed-matched play rate; 0 disables it.
- New `MotionMatchingFrameInput.HoldReseedFilter`: the host decides when the
  post-teleport LoopOnly reseed filter ends. The core no longer compares a
  speed against a built-in threshold.
- `MotionMatchingFrameInput.CandidateBand` is now `ClipGroup`, and `OneShotBand`
  is now `OneShotGroup`. Same values and filter semantics (one-shot clips of
  another non-zero group are skipped; loops and group 0 never are). The trace
  JSON keeps the key `band`.
- Removed `RootYawDelta` and the turn-in-place yaw machinery, the `grounded`
  tick parameter, `PolicyState` values other than `Disabled`/`Active`,
  `ForcePlayClip`, `CalculateTrajectory`, the up-axis parameter of
  `MotionMatchingPlayback::Setup` and `TravelingDistance`.
- Removed the native trajectory predictor and spring-damper. The plugin no longer
  predicts trajectories: the host supplies `TrajectoryPoint`s at the schema's
  `SampleTimes`, in actor space (X right, Z forward).
- Removed telemetry JSONL, `CacheFolder`, `TraceOverride` and `IndexedVerifyAuto`.
  One `Diagnostics` flag enables the trace ring and search diagnostics (off by
  default, never changes the winner). `GenerateClipAudit(filePath, out error)`
  takes an explicit path.
- Removed the three editor weight presets and the `MotionMatchingBakerEditor`
  tombstone. The editor contract test no longer carries project-specific tags
  or field lists.
- Removed the file-name based speed bands and turn-like flag from the baker and
  the sample module `MotionMatchingAutoSetup`. The plugin never reads file
  names and ships no naming conventions; clips carry an authored `group`.
- Database format v4. A v1 to v3 file is upgraded in place by the editor
  (registry and per-source groups are kept) but its baked data is dropped, so
  every database must be rebaked once. See [MIGRATION.md](MIGRATION.md).
- A v4 database binds only to a model that matches its skeleton fingerprint, and
  only when the live schema equals the baked snapshot (weights excepted).
  Otherwise the controller is not ready and `BindingReport` says why.

### Added

- Operating constants are now `MotionMatchingPolicyTuning` fields with the
  previous values as defaults: play-rate min/max/min-speed, teleport distance,
  warm-up, fast-path age/frames, reselect window and ban, same-segment twin
  ban, non-loop tail exclusion (`LoopTailExclusion`, 0 = off). New
  `MotionMatchingPlayback::SetPlayRateRange`.
- Clip groups: an integer `group` per registry entry (0 = any), set through
  `AddClips`/`AddClipFiles`/`UpdateSourceClip`/`ApplyClipEdits` or the registry
  JSON, baked into a per-clip array. Hosts can pass
  `ClipRegistryImportOptions.InferGroup` when importing an old registry.
- Contact gate fields on the schema, stored in the database: `ContactMaxSpeed`
  (300 units/s) and `ContactMaxHeight` (12 units above the root). The baker and
  the query read them from the baked schema.
- `Schema.BoneAliases` (clip channel name to model node name) and one shared
  channel resolver (`Runtime/MotionMatchingChannelMap.h`) used by the baker, the
  sampler and playback.
- Bake-time channel coverage report with alias suggestions. The bake fails when
  Root, Pelvis or either foot has no channel in any clip (clips that lack one
  keep the bind pose and are reported). Editor:
  `SuggestBoneAliases`, `DescribeChannelCoverage` and a "Suggest Bone Aliases"
  button in the register window.
- Skeleton fingerprint stored in the database and checked at bind time
  (`BindingReport`, `AllowBindingMismatch` as an explicit debug opt-out).
- `MotionMatchingDatabase.ComputeFeaturesHash()` and `ComputeClipMetadataHash()`
  (64-bit FNV-1a) for bit-exact parity checks across refactors.
- Offline gates: `Tools/check_contract.py` (repaired, with real mutation tests in
  `Tools/test_contract_audit.py`) and `Tools/run_gates.py`, which runs every
  engine-free gate.

### Changed

- Playback self-test rewritten around observable behaviour (time advance, play
  rate and range, end of clip, loop wrap, finite pose, stripped root).
- The editor contract test takes tag scopes and trajectory point counts from the
  database and schema.
- Generated files (`*.Gen.*`, `*.csproj`, `Properties/launchSettings.json`) are
  no longer tracked; Flax.Build regenerates them.
- Comment cleanup: historical phase tags and references to removed features
  were dropped from the sources.

### Verification status

- Built offline on Windows x64 with the installed Flax 1.12 engine
  (Development configuration) and checked by the offline gates after every
  milestone. Native self-tests and the live parity checks were run for M0 to M2;
  the M3 live checks (database upgrade, rebake, bind-time scenarios) are
  tracked in the host repository's `docs/archive/MM_PLUGIN_PRODUCTION.md`.
- No other platform or configuration has been built or tested.
