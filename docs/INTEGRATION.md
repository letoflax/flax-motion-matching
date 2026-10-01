# Integration guide

This plugin is consumed by a Flax project as a normal project reference. The
host owns the actor, input provider, movement authority, camera, and any pose
post-processing. The plugin owns database loading, pose search, playback, and
the neutral input/output contract.

## Project and module setup

1. Copy or reference `Plugins/MotionMatching/MotionMatching.flaxproj` from the
   host project's `.flaxproj`.
2. Add `MotionMatching` to the game target modules.
3. Add `MotionMatchingEditor` to the editor target modules when editor bake,
   registry, tuning, or contract-test tooling is needed.
4. Implement `IMotionMatchingInputProvider` in the host (or reuse a
   host-owned god-class like `PlayerAnimationController`); the plugin
   ships no locomotion sample module.

The plugin project itself references only the Flax engine. The runtime module
does not reference a game assembly, actor type, player coordinator, or sample
content. The `Samples` directory is an opt-in example for this repository.

## Minimal host wiring

`MotionMatchingController` is a plain C# object; it is not a scene script.
Create and retain one controller in the host facade, then configure:

```csharp
var controller = new MotionMatchingController
{
    Schema = hostSchema,
    PlayerModel = hostModel,
    CharacterBody = hostAnimatedModel,
    DatabasePath = "MotionMatching/MyDatabase.flax",
};
controller.Initialize();
```

On every host tick:

```csharp
MotionMatchingInputContext context = BuildContext();
MotionMatchingFrameInput frame = provider.BuildInput(context);
controller.Update(deltaTime, actorWorldTransform, frame);
```

The provider owns gameplay interpretation and produces trajectory points,
include/exclude tag sets, the loop filter, the candidate band, the query yaw
rate, the speed-matching target speed, and the reseed hold. The controller
validates filter enum ranges, loads the database on first use when necessary,
and forwards the frame to the native policy.

The base search pose is available through `TryGetBasePose` or the reusable
`TryGetBasePoseInto` overload. The host may apply its own layers or IK and write
the final pose. Playback is a borrowed handle: do not destroy it, and reacquire
it after reset, rebind, or shutdown.

The body must not have an AnimGraph that overwrites manually written poses.
Character movement and root motion remain host responsibilities.

## Database path

`DatabasePath` may be a host Content-relative path or an absolute path. The
default is `MotionMatching/MotionMatchingDatabase.flax`. The host's database
asset must be baked with a schema compatible with the configured rig; the
database stores the bake-time schema snapshot.
