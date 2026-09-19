# MotionMatching plugin

Independent Flax plugin project: pose-search motion matching with a single
input-provider contract. References the engine only — never a game.

- `Source/MotionMatching` — runtime module (native search/query/policy/
  playback/database/baker + C# shell and contract DTOs + trajectory math
  helper). No locomotion policy: gait/scope decisions live in the host.
- `Source/MotionMatchingEditor` — editor module (services, baker/tuning/
  registry windows, generic registry import/export, baker/tuning, and contract
  tests). Registry entries carry their own asset paths, tags, and loop flags;
  host-specific inference can be supplied explicitly through
  `ClipRegistryImportOptions`.
- `Source/MotionMatchingEditor/Samples` — opt-in flax-test sample automation
  (`MotionMatchingAutoSetup`), including the sample `Animations/Humanoid`
  convention. Generic editor workflows do not invoke it.

## Host documentation

- [Integration](docs/INTEGRATION.md): reference the project/modules and drive a
  controller from your own Flax host.
- [Public contract](docs/PLUGIN_CONTRACT.md): input/output ownership, coordinates,
  tag/filter semantics, and the explicit LoopOnly recovery invariant.
- [Lifecycle](docs/LIFECYCLE.md): initialization, reset, rebind, and cleanup.
- [Authoring](docs/AUTHORING.md): rig/schema setup, registry import/export, baking.
- [Migration and asset identity](docs/MIGRATION.md): scripting names versus the
  frozen legacy binary database TypeName.

## Offline contract gate

From this plugin directory, with Python 3.10+ (standard library only):

```text
python Tools/check_contract.py
python Tools/test_contract_audit.py
```

The [gate](Tools/check_contract.py) checks documentation links/public declarations,
filter/recovery source invariants, asset identity, and project-coupling boundaries.
The [audit tests](Tools/test_contract_audit.py) inject in-memory regressions to
verify rejection. Neither command needs a host project, assets, generated
bindings, an editor, or a bridge; paths are resolved relative to the script.
These are static checks, not a native build or playback test.

In the flax-test repository, complementary host-side Python gates are
`Tools/mm_module_check.py`, `Tools/mm_editor_decoupling_check.py`, and
`Tools/mm_tests.py`. The separate `Tools/mm_contract_test.py` requires a running
Flax editor and the host's bridge/scene; it is not an offline plugin dependency.
