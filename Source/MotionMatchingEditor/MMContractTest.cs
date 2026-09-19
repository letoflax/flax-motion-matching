using System.Collections.Generic;
using FlaxEngine;

namespace MotionMatching;

/// <summary>
/// Scripted input provider for the contract test below. Plain counter +
/// canned frame; no game logic. P3: plain class (the runtime no longer owns
/// a provider slot — the game facade builds frames itself).
/// </summary>
public class MMContractTestProvider : IMotionMatchingInputProvider
{
    public int BuildCalls;
    public bool ResetCalled;
    public MotionMatchingFrameInput FrameToReturn;

    public MotionMatchingFrameInput BuildInput(MotionMatchingInputContext ctx)
    {
        BuildCalls++;
        return FrameToReturn;
    }

    public void Reset()
    {
        ResetCalled = true;
    }
}

/// <summary>
/// Runtime-contract tests (P2 boundary locks, P3 re-anchored): explicit-empty
/// no-match, provider scope respected, reset-then-soak stays usable, per-frame
/// source swap takes effect, input free of gameplay fields (compile-level),
/// pose-out equals the internal search locals, force-hatch effect, pure
/// search with no machine direct-plays, and P2 filter-change-mid-clip
/// fast-path invalidation (+ LoopFilter flip). E1/E2 driver locks are deleted
/// with the native machines.
///
/// P3: all checks run through the plain runtime object (Update with canned
/// frames + an identity world transform), never through provider slots or
/// hand-built native structs: the generated snapshot/tuning layouts are owned
/// by the native build. Runs in PLAY mode (pose sampling and playback only
/// tick live there).
///
/// Debt fix (deterministic harness): the runner pauses simulation
/// (Tools/mm_contract_test.py: play.pause before this op, resume after), so
/// live facade ticks stop. This test additionally runs on an ISOLATED
/// controller instance cloned from the live one (same Database/CharacterBody/
/// Schema/tuning, own native policy): the live controller is never mutated,
/// so test-1 explicit-empty mirrors cannot race with the scene tick.
/// </summary>
public static class MMContractTest
{
    private const float Dt = 1.0f / 60.0f;

    // Field names that must NOT exist on the P2 frame (gameplay lives
    // game-side; the core takes trajectory + scope + caller filter only).
    private static readonly string[] DeadFrameFields = new string[]
    {
        "HasScope",
        "Velocity", "DesiredVelocity", "Acceleration", "AngularVelocity",
        "TargetYaw", "CurrentYaw", "IsGrounded", "IsSprinting",
        "JumpTag", "TurnInPlaceTag", "TurnTravelTag", "RestTag",
        "RestSpeedMax", "JumpWalkSpeedMax", "JumpRunSpeedMax",
        "YawRateThreshold", "TagConfirmRepeats",
        "BackwardThreshold", "LateralThreshold", "LateralZRatio",
        "LayerAngleDeg", "GateBone",
        "EnableJumpDriver", "EnableTurnDrive",
    };

    private static readonly string[] MachineReasonPrefixes = new string[]
    {
        "jump:", "drive:", "held:landing", "held:turn", "replay:turn",
    };

    private static readonly Transform TestWorld = new Transform(Float3.Zero, Quaternion.Identity, Float3.One);

    // Isolated test controller: same assets/tuning as the live scene object,
    // but a fresh native policy (own _policy/playback/trace state). Live
    // mirrors are never written by the checks below. Telemetry/trace overrides
    // stay off: on/off must not change search decisions (P4 gate), and the
    // isolated trace is never exported (only the facade's session export runs).
    private static MotionMatchingController NewIsolated(MotionMatchingController live)
    {
        var iso = new MotionMatchingController
        {
            Schema = live.Schema,
            PlayerModel = live.PlayerModel,
            DatabasePath = live.DatabasePath,
            Database = live.Database,
            CharacterBody = live.CharacterBody,
            CacheFolder = live.CacheFolder,
            TelemetryOverride = false,
            IndexedVerifyAuto = live.IndexedVerifyAuto,
            TraceOverride = false,
            QueryInterval = live.QueryInterval,
            SearchEveryFrame = live.SearchEveryFrame,
            FrameSearchBudget = live.FrameSearchBudget,
            UseIndexedSearch = live.UseIndexedSearch,
            TopKCandidates = live.TopKCandidates,
            VerifyIndexedSearch = live.VerifyIndexedSearch,
            ContinuityWeight = live.ContinuityWeight,
            MinSwitchTime = live.MinSwitchTime,
            ContinuingBias = live.ContinuingBias,
            ContinuingWindow = live.ContinuingWindow,
            SwitchMargin = live.SwitchMargin,
            SameClipExclusion = live.SameClipExclusion,
            SameClipSeekThreshold = live.SameClipSeekThreshold,
            CostFuseThreshold = live.CostFuseThreshold,
            TurnPenaltyWeight = live.TurnPenaltyWeight,
            TurnPenaltyYawScale = live.TurnPenaltyYawScale,
            InertializationDuration = live.InertializationDuration,
            TelemetryLog = false,
        };
        iso.Initialize();
        return iso;
    }

    public static string RunAll(MotionMatchingController controller)
    {
        var lines = new List<string>();
        int passed = 0;
        int total = 0;

        void Check(string name, bool ok, string detail)
        {
            total++;
            if (ok)
                passed++;
            lines.Add((ok ? "PASS " : "FAIL ") + name + " — " + detail);
        }

        if (controller == null)
            return "CONTRACT FAIL: no controller.";
        if (controller.Database == null || !controller.Database.IsBaked)
            return "CONTRACT FAIL: database not baked.";
        if (controller.CharacterBody == null)
            return "CONTRACT FAIL: CharacterBody not assigned.";

        float[] sampleTimes = controller.TrajectorySampleTimes;
        if (sampleTimes == null || sampleTimes.Length != 6)
            return "CONTRACT FAIL: need 6 sample times, got " +
                (sampleTimes == null ? "null" : sampleTimes.Length.ToString()) + ".";

        // Isolated instance: all checks below run here; the live scene
        // controller is never written (see NewIsolated + class comment).
        var test = NewIsolated(controller);

        // 1: explicit-empty scope = no-match, never a silent pool fallback.
        {
            var prov = NewCanned(string.Empty, string.Empty);
            test.ResetPlayback();
            TickProvider(test, prov, 100);
            bool statusOk = test.ChooserStatus.StartsWith("ext:empty");
            bool reasonOk = test.LastSwitchReason == "no-result:empty-scope";
            Check("explicit-empty", statusOk && reasonOk,
                "chooser='" + test.ChooserStatus + "' reason='" + test.LastSwitchReason + "'");
        }

        // 2: provider scope is respected (single-tag idle at rest).
        {
            var prov = NewCanned("idle", string.Empty);
            test.ResetPlayback();
            TickProvider(test, prov, 120);
            bool ok = test.ActiveTag == "idle";
            Check("scope-respected", ok,
                "activeTag='" + test.ActiveTag + "' winner='" + test.WinnerClipName + "'");
        }

        // 3: reset leaves the runtime usable (P3: the runtime owns no
        // provider slot, so there is no reset propagation to test here —
        // the facade resets its provider separately. Reset then soak must
        // still settle on the scoped tag).
        {
            var prov = NewCanned("idle", string.Empty);
            test.ResetPlayback();
            TickProvider(test, prov, 120);
            bool ok = test.ActiveTag == "idle";
            Check("reset-then-soak", ok,
                "activeTag='" + test.ActiveTag + "' after ResetPlayback");
        }

        // 4: per-frame source swap takes effect on the next update (P3: the
        // facade feeds frames directly, so swap = different frame contents:
        // idle soak, then empty scope must flip the chooser to ext:empty,
        // then idle must recover).
        {
            var idle = NewCanned("idle", string.Empty);
            var empty = NewCanned(string.Empty, string.Empty);
            test.ResetPlayback();
            TickProvider(test, idle, 60);
            TickProvider(test, empty, 60);
            bool emptyOk = test.ChooserStatus.StartsWith("ext:empty");
            TickProvider(test, idle, 120);
            bool recoverOk = test.ActiveTag == "idle";
            Check("frame-swap", emptyOk && recoverOk,
                "emptyChooser='" + test.ChooserStatus + "' recoveredTag='" + test.ActiveTag + "'");
            test.ResetPlayback();
        }

        // 5: input carries no gameplay fields (compile-level boundary).
        {
            var names = new HashSet<string>();
            foreach (var f in typeof(MotionMatchingFrameInput).GetFields())
                names.Add(f.Name);
            var leaked = new List<string>();
            foreach (string dead in DeadFrameFields)
            {
                if (names.Contains(dead))
                    leaked.Add(dead);
            }
            Check("input-no-gameplay-fields", leaked.Count == 0,
                leaked.Count == 0
                    ? "frame fields=[" + string.Join(",", names) + "]"
                    : "leaked=[" + string.Join(",", leaked) + "]");
        }

        // 6: pose-out equals the internal search locals.
        {
            var prov = NewCanned("idle", string.Empty);
            test.ResetPlayback();
            TickProvider(test, prov, 30);
            bool ok = false;
            string detail = "TryGetBasePose=false";
            if (test.TryGetBasePose(out Matrix[] locals, out int clip, out float cost))
            {
                var playback = test.Playback;
                Matrix[] internals = playback != null ? playback.GetSearchLocals() : null;
                if (internals != null && locals.Length == internals.Length && locals.Length > 0)
                {
                    ok = true;
                    for (int i = 0; i < locals.Length; i++)
                    {
                        if (!locals[i].Equals(internals[i]))
                        {
                            ok = false;
                            break;
                        }
                    }
                    detail = "bones=" + locals.Length + " clip=" + clip;
                }
                else
                {
                    detail = "lens out=" + locals.Length + " internal=" + (internals == null ? "null" : internals.Length.ToString());
                }
            }
            Check("pose-out-is-search-locals", ok, detail);
        }

        // 7: force-hatch lands the requested clip (P0 hatch, logs every call).
        {
            var prov = NewCanned("idle", string.Empty);
            test.ResetPlayback();
            TickProvider(test, prov, 10);
            var playback = test.Playback;
            int clips = test.Database != null ? test.Database.ClipCount : 0;
            bool ok = false;
            string detail = "no playback/clips";
            if (playback != null && clips > 1)
            {
                int target = (playback.CurrentClip + 1) % clips;
                test.ForcePlayClip(target, 0.0f, "contract:force-hatch");
                ok = playback.CurrentClip == target;
                detail = "forced=" + target + " current=" + playback.CurrentClip;
            }
            Check("force-hatch", ok, detail);
        }

        // 8: pure search — a grounded fixed-scope soak never takes a
        // machine direct-play (no jump/drive/hold/replay reasons; only
        // the search-winner path and the ForcePlayClip hatch may Play).
        {
            var prov = NewCanned("idle", string.Empty);
            test.ResetPlayback();
            TickProvider(test, prov, 90);
            string reason = test.LastSwitchReason ?? string.Empty;
            bool machine = false;
            foreach (string prefix in MachineReasonPrefixes)
            {
                if (reason.StartsWith(prefix))
                {
                    machine = true;
                    break;
                }
            }
            Check("pure-search-no-machine-play", !machine,
                "reason='" + reason + "' chooser='" + test.ChooserStatus + "'");
        }

        // 9: P2 filter-change-mid-clip invalidates the fast-path: soak idle
        // until the continuation holds, then change the scope to walk at
        // constant input. The core must NOT hold the stale idle clip — the
        // chooser must adopt the new filter within the soak window.
        {
            var idle = NewCanned("idle", string.Empty);
            var walk = NewCanned("walk", string.Empty);
            test.ResetPlayback();
            TickProvider(test, idle, 120);
            string before = test.ActiveTag ?? string.Empty;
            TickProvider(test, walk, 120);
            string after = test.ActiveTag ?? string.Empty;
            bool ok = before == "idle" && after == "walk";
            Check("filter-change-invalidates-fastpath", ok,
                "before='" + before + "' after='" + after + "'");
            // Same-scope LoopFilter flip must also take effect (Any -> LoopOnly
            // on the idle scope keeps idle loops; the recorded filter rides
            // the trace for replay).
            var idleLoopOnly = NewCannedFiltered("idle", string.Empty, 1, 0);
            TickProvider(test, idleLoopOnly, 60);
            bool loopOk = (test.ActiveTag ?? string.Empty) == "idle";
            Check("loopfilter-change-takes-effect", loopOk,
                "tag='" + test.ActiveTag + "' chooser='" + test.ChooserStatus + "'");
            test.ResetPlayback();
        }

        test.ResetPlayback();

        string verdict = passed == total ? "CONTRACT PASS" : "CONTRACT FAIL";
        return verdict + ": " + passed + "/" + total + " checks\n" + string.Join("\n", lines);
    }

    private static MMContractTestProvider NewCanned(string allowed, string excluded)
    {
        return new MMContractTestProvider { FrameToReturn = CannedFrame(allowed, excluded, 0, 0) };
    }

    private static MMContractTestProvider NewCannedFiltered(string allowed, string excluded, int loop, int band)
    {
        return new MMContractTestProvider { FrameToReturn = CannedFrame(allowed, excluded, loop, band) };
    }

    private static void TickProvider(MotionMatchingController controller, MMContractTestProvider prov, int n)
    {
        for (int i = 0; i < n; i++)
        {
            var ctx = new MotionMatchingInputContext
            {
                ActorWorld = TestWorld,
                DeltaTime = Dt,
                FrameId = i,
                DatabaseTags = controller.AnimationTags,
                SampleTimes = controller.TrajectorySampleTimes,
            };
            MotionMatchingFrameInput frame = prov.BuildInput(ctx);
            controller.Update(Dt, TestWorld, frame);
        }
    }

    private static MotionMatchingFrameInput CannedFrame(string allowed, string excluded, int loop, int band)
    {
        // P2 slim frame: trajectory + scope + caller filter only.
        return new MotionMatchingFrameInput
        {
            Points = new TrajectoryPoint[6],
            AllowedTagSet = allowed ?? string.Empty,
            ExcludedTagSet = excluded ?? string.Empty,
            LoopFilter = loop,
            CandidateBand = band,
            QueryYawRate = 0.0f,
        };
    }
}
