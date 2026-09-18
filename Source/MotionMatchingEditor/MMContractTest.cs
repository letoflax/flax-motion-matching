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
/// pose-out equals the internal search locals, force-hatch effect, and pure
/// search with no machine direct-plays. E1/E2 driver locks are deleted with
/// the native machines.
///
/// P3: all checks run through the plain runtime object (Update with canned
/// frames + an identity world transform), never through provider slots or
/// hand-built native structs: the generated snapshot/tuning layouts are owned
/// by the native build. Runs in PLAY mode (pose sampling and playback only
/// tick live there).
/// </summary>
public static class MMContractTest
{
    private const float Dt = 1.0f / 60.0f;

    // Field names that must NOT exist on the P2 frame (gameplay lives
    // game-side; the core takes movement + trajectory + scope only).
    private static readonly string[] DeadFrameFields = new string[]
    {
        "HasScope",
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

        // 1: explicit-empty scope = no-match, never a silent pool fallback.
        {
            var prov = NewCanned(string.Empty, string.Empty);
            controller.ResetPlayback();
            TickProvider(controller, prov, 100);
            bool statusOk = controller.ChooserStatus.StartsWith("ext:empty");
            bool reasonOk = controller.LastSwitchReason == "no-result:empty-scope";
            Check("explicit-empty", statusOk && reasonOk,
                "chooser='" + controller.ChooserStatus + "' reason='" + controller.LastSwitchReason + "'");
        }

        // 2: provider scope is respected (single-tag idle at rest).
        {
            var prov = NewCanned("idle", string.Empty);
            controller.ResetPlayback();
            TickProvider(controller, prov, 120);
            bool ok = controller.ActiveTag == "idle";
            Check("scope-respected", ok,
                "activeTag='" + controller.ActiveTag + "' winner='" + controller.WinnerClipName + "'");
        }

        // 3: reset leaves the runtime usable (P3: the runtime owns no
        // provider slot, so there is no reset propagation to test here —
        // the facade resets its provider separately. Reset then soak must
        // still settle on the scoped tag).
        {
            var prov = NewCanned("idle", string.Empty);
            controller.ResetPlayback();
            TickProvider(controller, prov, 120);
            bool ok = controller.ActiveTag == "idle";
            Check("reset-then-soak", ok,
                "activeTag='" + controller.ActiveTag + "' after ResetPlayback");
        }

        // 4: per-frame source swap takes effect on the next update (P3: the
        // facade feeds frames directly, so swap = different frame contents:
        // idle soak, then empty scope must flip the chooser to ext:empty,
        // then idle must recover).
        {
            var idle = NewCanned("idle", string.Empty);
            var empty = NewCanned(string.Empty, string.Empty);
            controller.ResetPlayback();
            TickProvider(controller, idle, 60);
            TickProvider(controller, empty, 60);
            bool emptyOk = controller.ChooserStatus.StartsWith("ext:empty");
            TickProvider(controller, idle, 120);
            bool recoverOk = controller.ActiveTag == "idle";
            Check("frame-swap", emptyOk && recoverOk,
                "emptyChooser='" + controller.ChooserStatus + "' recoveredTag='" + controller.ActiveTag + "'");
            controller.ResetPlayback();
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
            controller.ResetPlayback();
            TickProvider(controller, prov, 30);
            bool ok = false;
            string detail = "TryGetBasePose=false";
            if (controller.TryGetBasePose(out Matrix[] locals, out int clip, out float cost))
            {
                var playback = controller.Playback;
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
            controller.ResetPlayback();
            TickProvider(controller, prov, 10);
            var playback = controller.Playback;
            int clips = controller.Database != null ? controller.Database.ClipCount : 0;
            bool ok = false;
            string detail = "no playback/clips";
            if (playback != null && clips > 1)
            {
                int target = (playback.CurrentClip + 1) % clips;
                controller.ForcePlayClip(target, 0.0f, "contract:force-hatch");
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
            controller.ResetPlayback();
            TickProvider(controller, prov, 90);
            string reason = controller.LastSwitchReason ?? string.Empty;
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
                "reason='" + reason + "' chooser='" + controller.ChooserStatus + "'");
        }

        controller.ResetPlayback();

        string verdict = passed == total ? "CONTRACT PASS" : "CONTRACT FAIL";
        return verdict + ": " + passed + "/" + total + " checks\n" + string.Join("\n", lines);
    }

    private static MMContractTestProvider NewCanned(string allowed, string excluded)
    {
        return new MMContractTestProvider { FrameToReturn = CannedFrame(allowed, excluded) };
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

    private static MotionMatchingFrameInput CannedFrame(string allowed, string excluded)
    {
        // P2 slim frame: movement + trajectory + scope only.
        return new MotionMatchingFrameInput
        {
            Velocity = Float3.Zero,
            DesiredVelocity = Float3.Zero,
            Acceleration = Float3.Zero,
            AngularVelocity = Float3.Zero,
            TargetYaw = 0.0f,
            CurrentYaw = 0.0f,
            IsGrounded = true,
            IsSprinting = false,
            Points = new TrajectoryPoint[6],
            AllowedTagSet = allowed ?? string.Empty,
            ExcludedTagSet = excluded ?? string.Empty,
        };
    }
}
