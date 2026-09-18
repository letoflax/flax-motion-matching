using System.Collections.Generic;
using FlaxEngine;

namespace MotionMatching;

/// <summary>
/// Scripted input provider for the contract test below. Plain counter +
/// canned frame; no game logic.
/// </summary>
public class MMContractTestProvider : Script, IMotionMatchingInputProvider
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
/// Provider-contract tests (MM plugin §14, review R11): explicit-empty,
/// unconstrained, reset propagation, mid-play swap, driver opt-outs.
/// Runs in PLAY mode on the loaded PlayerTestScene through a private policy
/// instance (never touches the scene's live policy) plus the shell slot
/// with save/restore. Surfaced via the mm.tuning bridge op "contract".
/// Play mode is required: pose sampling and playback only tick live there
/// (edit-mode ticks run no queries, so A/B cannot assert).
/// </summary>
public static class MMContractTest
{
    private const float Dt = 1.0f / 60.0f;

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

        // A: explicit-empty scope = no-match, never a silent pool fallback.
        {
            var policy = NewPolicy(controller);
            var tuning = BaseTuning();
            var snapshot = BaseSnapshot();
            snapshot.ScopeConstrained = true;
            snapshot.AllowedTagSet = string.Empty;
            snapshot.ExcludedTagSet = string.Empty;
            TickN(policy, snapshot, tuning, controller, 100);
            bool statusOk = policy.ChooserStatusText.StartsWith("ext:empty");
            bool reasonOk = policy.LastSwitchReason == "no-result:empty-scope";
            Check("explicit-empty", statusOk && reasonOk,
                "chooser='" + policy.ChooserStatusText + "' reason='" + policy.LastSwitchReason + "'");
            policy.Shutdown();
        }

        // B: unconstrained = legacy native path (no "ext:" status).
        {
            var policy = NewPolicy(controller);
            var tuning = BaseTuning();
            var snapshot = BaseSnapshot();
            snapshot.ScopeConstrained = false;
            snapshot.AllowedTagSet = string.Empty;
            TickN(policy, snapshot, tuning, controller, 100);
            bool ok = !policy.ChooserStatusText.StartsWith("ext:") &&
                policy.ChooserStatusText.Length > 0;
            Check("unconstrained-legacy", ok,
                "chooser='" + policy.ChooserStatusText + "'");
            policy.Shutdown();
        }

        // C: shell ResetPlayback propagates to the provider.
        {
            var prov = controller.Actor.AddScript<MMContractTestProvider>();
            Script saved = controller.InputProvider;
            controller.InputProvider = prov;
            try
            {
                controller.ResetPlayback();
                Check("reset-propagates", prov.ResetCalled,
                    "provider.ResetCalled=" + prov.ResetCalled);
            }
            finally
            {
                controller.InputProvider = saved;
                Destroy(prov);
            }
        }

        // D: mid-play provider swap takes effect on the next tick.
        {
            var provA = controller.Actor.AddScript<MMContractTestProvider>();
            var provB = controller.Actor.AddScript<MMContractTestProvider>();
            provA.FrameToReturn = CannedFrame(sampleTimes);
            provB.FrameToReturn = CannedFrame(sampleTimes);
            Script saved = controller.InputProvider;
            try
            {
                controller.InputProvider = provA;
                controller.TickPolicy(Dt);
                controller.TickPolicy(Dt);
                int aCalls = provA.BuildCalls;
                controller.InputProvider = provB;
                controller.TickPolicy(Dt);
                controller.TickPolicy(Dt);
                bool ok = aCalls >= 2 && provA.BuildCalls == aCalls && provB.BuildCalls >= 2;
                Check("provider-swap", ok,
                    "A=" + provA.BuildCalls + " (was " + aCalls + ") B=" + provB.BuildCalls);
                controller.ResetPlayback();
            }
            finally
            {
                controller.InputProvider = saved;
                Destroy(provA);
                Destroy(provB);
            }
        }

        // E1: jump driver off = airborne never takes off.
        {
            var policy = NewPolicy(controller);
            var tuning = BaseTuning();
            tuning.EnableJumpDriver = false;
            var snapshot = BaseSnapshot();
            snapshot.IsGrounded = false;
            snapshot.CurrentVelocity = new Float3(0.0f, -100.0f, 300.0f);
            TickN(policy, snapshot, tuning, controller, 30);
            bool ok = policy.JumpCount == 0;
            Check("jump-driver-off", ok, "JumpCount=" + policy.JumpCount);

            var policyOn = NewPolicy(controller);
            var tuningOn = BaseTuning();
            tuningOn.EnableJumpDriver = true;
            TickN(policyOn, snapshot, tuningOn, controller, 30);
            bool okOn = policyOn.JumpCount >= 1;
            Check("jump-driver-on-positive", okOn, "JumpCount=" + policyOn.JumpCount);
            policy.Shutdown();
            policyOn.Shutdown();
        }

        // E2: turn drive off = huge facing error never commits a turn.
        // 45 ticks: the drive would start right after the 0.5s warmup
        // (~tick 31) and still be mid-clip at the assert; 90 ticks could
        // outlast the turn clip and exit normally.
        {
            var policy = NewPolicy(controller);
            var tuning = BaseTuning();
            tuning.EnableTurnDrive = false;
            var snapshot = BaseSnapshot();
            snapshot.TargetYaw = 120.0f;
            snapshot.CurrentYaw = 0.0f;
            TickN(policy, snapshot, tuning, controller, 45);
            bool ok = !policy.TurnDriveActive;
            Check("turn-drive-off", ok, "TurnDriveActive=" + policy.TurnDriveActive);

            var policyOn = NewPolicy(controller);
            var tuningOn = BaseTuning();
            tuningOn.EnableTurnDrive = true;
            TickN(policyOn, snapshot, tuningOn, controller, 45);
            bool okOn = policyOn.TurnDriveActive;
            Check("turn-drive-on-positive", okOn, "TurnDriveActive=" + policyOn.TurnDriveActive);
            policy.Shutdown();
            policyOn.Shutdown();
        }

        string verdict = passed == total ? "CONTRACT PASS" : "CONTRACT FAIL";
        return verdict + ": " + passed + "/" + total + " checks\n" + string.Join("\n", lines);
    }

    private static MotionMatchingRuntimePolicy NewPolicy(MotionMatchingController controller)
    {
        var policy = new MotionMatchingRuntimePolicy();
        policy.Initialize(controller.Database, controller.CharacterBody,
            controller.Schema, string.Empty, 0.25f);
        return policy;
    }

    private static MotionMatchingPolicyTuning BaseTuning()
    {
        // Explicit every field: C# `new` on a bound struct does NOT run the
        // C++ member initializers, so unset fields read zero (a zeroed
        // YawRateThreshold once let every hold gate pass and swallowed all
        // queries). Values mirror the shell/provider defaults; search is
        // deterministic: every warmed tick scans unstrided, brute force,
        // no telemetry, no IK sweeps.
        return new MotionMatchingPolicyTuning
        {
            PoseWeight = 1.0f,
            TrajectoryWeight = 2.0f,
            SampleRate = 30.0f,
            IdleSpeedMax = 5.0f,
            WalkSpeedMax = 200.0f,
            RunSpeedMax = 490.0f,
            YawRateThreshold = 0.5f,
            FairFightAccum = 40.0f,
            FairFightWindow = 0.25f,
            TagConfirmRepeats = 2,
            SettleToLoopDelay = 0.3f,
            TurnMinSwitchTime = 0.3f,
            MinSwitchTime = 0.12f,
            SameClipExclusion = 0.25f,
            SameClipSeekThreshold = 0.25f,
            ContinuingBias = 0.012f,
            ContinuingWindow = 0.15f,
            SwitchMargin = 0.008f,
            RestLoopPreference = 0.02f,
            FrameSearchBudget = 0,
            SearchEveryFrame = true,
            QueryInterval = 0.1f,
            UseIndexedSearch = false,
            TopKCandidates = 16,
            VerifyIndexedSearch = false,
            IndexedVerifyAuto = false,
            EffectiveTelemetry = false,
            EnableFootIK = false,
            EnableJumpDriver = true,
            EnableTurnDrive = true,
            ContinuityWeight = 0.0025f,
            CostFuseThreshold = 2.0f,
            TurnDriveAngle = 25.0f,
            TurnLookahead = 0.2f,
            TurnPatientAngle = 15.0f,
            TurnPatientTime = 1.0f,
            TurnSettleRate = 2.0f,
            BackwardThreshold = 30.0f,
            LateralThreshold = 30.0f,
            LateralZRatio = 0.58f,
            JumpTag = "jump",
            TurnInPlaceTag = "turn_inplace",
            TurnTravelTag = "turn_travel",
            RestTag = "idle",
        };
    }

    private static MotionMatchingPlayerSnapshot BaseSnapshot()
    {
        return new MotionMatchingPlayerSnapshot
        {
            CurrentVelocity = Float3.Zero,
            DesiredVelocity = Float3.Zero,
            AngularVelocity = Float3.Zero,
            TargetYaw = 0.0f,
            CurrentYaw = 0.0f,
            AimPitchDeg = 0.0f,
            HideHead = false,
            IsGrounded = true,
            IsSprinting = false,
            ScopeConstrained = false,
            AllowedTagSet = string.Empty,
            ExcludedTagSet = string.Empty,
        };
    }

    private static MotionMatchingFrameInput CannedFrame(float[] sampleTimes)
    {
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
            HasScope = false,
            AllowedTagSet = string.Empty,
            ExcludedTagSet = string.Empty,
            JumpTag = "jump",
            TurnInPlaceTag = "turn_inplace",
            TurnTravelTag = "turn_travel",
            RestTag = "idle",
            RestSpeedMax = 5.0f,
            JumpWalkSpeedMax = 200.0f,
            JumpRunSpeedMax = 490.0f,
            YawRateThreshold = 0.5f,
            TagConfirmRepeats = 2,
            BackwardThreshold = 30.0f,
            LateralThreshold = 30.0f,
            LateralZRatio = 0.58f,
            AimPitchDeg = 0.0f,
            HideHead = false,
            EnableJumpDriver = true,
            EnableTurnDrive = true,
        };
    }

    private static void TickN(MotionMatchingRuntimePolicy policy,
        MotionMatchingPlayerSnapshot snapshot, MotionMatchingPolicyTuning tuning,
        MotionMatchingController controller, int n)
    {
        var points = new TrajectoryPoint[6];
        for (int i = 0; i < n; i++)
        {
            policy.Tick(Dt, snapshot, points,
                controller.Actor.Transform.GetWorld(),
                controller.TrajectorySampleTimes, tuning);
        }
    }

    private static void Destroy(Script script)
    {
        if (script)
            Object.Destroy(script);
    }
}
