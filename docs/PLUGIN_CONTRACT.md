# Public plugin contract

This document is the contract for a project that consumes the plugin without
depending on this repository's player or sample code.

## Public types and ownership

| Type | Owner | Use |
| --- | --- | --- |
| `MotionMatchingController` | host facade, plugin runtime | Configure assets, call `Initialize`, `Update`, `ResetPlayback`, and `Shutdown`. |
| `IMotionMatchingInputProvider` | host | Convert host state into one `MotionMatchingFrameInput` per tick. |
| `MotionMatchingInputContext` | plugin -> provider | Time, frame id, actor world transform, database tags, and sample times. |
| `MotionMatchingFrameInput` | provider -> plugin | Trajectory, scope, and explicit caller filters for one tick. |
| `PoseSearchSchema` | host authoring | Axes, the declared feature layout (`RootBone`, `PoseBones`, `ContactBones`, sample times), and live search weights. |
| `MotionMatchingPlayback` | plugin, borrowed by host | Read current playback/base-pose state; never destroy the handle. |
| `MotionMatchingEditorService` | host editor tooling | Registry, bake, validation, audit, tuning, and top-candidate operations. |

The provider and controller are separate responsibilities. The host calls the
provider; the controller does not discover or call game code. The native
boundary receives only plain scope/filter data and trajectory points.

## Coordinates and tick order

- `ActorWorld` is the actor's world transform, not parent-local transform.
- `TrajectoryPoint.Position` and `Facing` are character-local samples. Their
  interpretation uses the schema root forward/up axes; do not assume a global
  world axis in a host integration.
- The plugin does not predict trajectories. The host produces one point per
  `SampleTimes` entry (past from its own history, future from its own motion
  model) in the actor frame, X = right and Z = forward. The flax-test game does
  this in `Source/Game/Animation/Trajectory/TrajectoryPredictor.cs`.
- `SampleTimes` must contain the strictly ascending times used by the schema
  (one point each, 1 to 32), spanning negative and positive time around zero.
  The point count comes from the baked database
  (`MotionMatchingDatabase.TrajectoryPointCount`), never from a constant.
- A host tick updates its provider state, calls `BuildInput(context)`, then
  calls `controller.Update(deltaTime, actorWorld, frame)`.

## Feature layout

The layout is declared by the schema and baked into the database (see
`AUTHORING.md`, declaring the feature layout). Pose and trajectory sizes,
contact offsets and normalization groups are derived from it at run time
(`FeatureLayout`); the database exposes `PoseFeatureSize`,
`TrajectoryFeatureSize`, `PoseBoneCount`, `ContactCount`,
`TrajectoryPointCount`, `PlanarUpAxis` and `FeatureLayoutHash`. A declared
node missing from the bake or bound model is an error, never a silent skip.
Hosts must not hardcode feature sizes or bone roles: the plugin has no
pelvis/foot/hand roles of its own.

## Scope and filter semantics

`AllowedTagSet` and `ExcludedTagSet` are normalized `+`-joined sets.

- A non-empty include scope uses **overlap**: a clip is eligible when it has at
  least one included tag.
- An exclusion set is a **veto**: a clip is rejected when it has any excluded
  tag. Include and exclude both apply.
- At the controller/policy boundary, an empty `AllowedTagSet` is an explicit
  **no-match** command. The current clip holds; no retry widens the scope.
- At the low-level `MotionMatchingSearch` API, an empty `allowedTags` array
  means **all tags**, which is the standalone search primitive's historical
  contract. The runtime controller deliberately converts an empty frame scope
  into no-match before calling search.

`LoopFilter` values are `Any = 0`, `LoopOnly = 1`, and `OneShotOnly = 2`.
`ClipGroup` selects a clip group. Groups are authored per clip in the
registry (`group`, `0` or absent = any, any non-negative integer) and baked
into the database; the plugin never derives them from file names. A non-zero
group restricts one-shot candidates to clips of the same group or group `0`;
loops are never cut by it. The host decides what each id means (format v3
called this field `CandidateBand`).
Normal warmed queries use the caller's values verbatim after range validation;
the core does not infer them from velocity, grounded state, gait, or intent.

Two further frame fields are host decisions the core never derives:

- `TargetSpeed` (world units/s, `>= 0`) is the speed the character really
  moves at. Clip time bends to `TargetSpeed / clipRootSpeed`, clamped to
  `PlayRateMin..PlayRateMax`. At or below `PlayRateMinSpeed` (or `0`), the clip
  plays at its authored speed. It has no other effect.
- `HoldReseedFilter` keeps the structural reseed recovery armed (see below) for
  as long as it is true.

## Neutral LoopOnly invariant

The runtime intentionally has two structural recovery paths:

1. After `ResetPlayback` or a detected teleport/discontinuity, the reseed
   query uses `LoopOnly` while the frame's `HoldReseedFilter` is true. The
   first scan that runs with the hold cleared releases it, and the continuation
   fast path does not hold a pre-reset clip while it is armed.
2. If a query produces no result while playback reports `IsAtEnd`, the runtime
   retries once with temporal exclusion removed and `LoopOnly`.

Both paths retain the provider's same tag scope (include and exclude sets) and clip
group. They do not name or invent an idle/rest tag, read gameplay intent, or
widen the tag scope. Therefore a caller's `OneShotOnly` request is overridden
for this structural recovery query; the normal caller filter is restored on
the next ordinary query. This is an explicit neutral invariant, not a
configuration option, and preserves continuity without changing normal search
behavior.

## Outputs, cleanup, and files

`Playback`, `Trace`, and similar native handles are borrowed. `Shutdown` is
public and idempotent; it releases policy/playback/trace state and leaves host
configuration reusable. Call it on host disable/destroy. Export the trace
before shutdown when `Diagnostics` is enabled.

The runtime never writes files. `Trace.ExportJson`/`ExportCsv` and
`GenerateClipAudit(filePath, ...)` are explicit host/tool operations that take
the destination path from the caller; initialization and play do not scan the
database to create an audit.
