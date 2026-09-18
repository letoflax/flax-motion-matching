# MotionMatching plugin

Independent Flax plugin project: pose-search motion matching with a single
input-provider contract. References the engine only — never a game.

- `Source/MotionMatching` — runtime module (native search/query/policy/
  playback/database/baker + C# shell and contract DTOs).
- `Source/MotionMatchingEditor` — editor module (services, baker/tuning/
  registry windows, import presets, contract tests).
- `Source/MotionMatchingLocomotion` — optional default-locomotion sample
  (C# only, neutral snapshot in, no player types).

See the project root `docs/` for the contract, migration and hosting
guides, and `Tools/mm_gates.py` for the verification battery.
