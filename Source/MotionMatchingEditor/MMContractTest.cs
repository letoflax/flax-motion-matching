using System.Collections.Generic;
using FlaxEngine;

namespace MotionMatching;

/// <summary>
/// Scripted input provider for the contract test below. Plain counter +
/// canned frame; no game logic.
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
/// Runtime-contract tests: explicit-empty scope is a no-match, scope is
/// respected, reset-then-soak stays usable, per-frame input swaps take effect,
/// pose-out equals the internal search locals, filter changes invalidate the
/// continuation fast path, one-shots hold their progression, a schema rebind
/// releases the session and Shutdown is idempotent.
///
/// Everything runs through the plain runtime object (Update with canned frames
/// and an identity world transform), never through hand-built native structs.
/// Scope tags come from the baked database, so the test works on any database
/// with at least one looping clip. Runs in PLAY mode (pose sampling and
/// playback only tick live there). The runner pauses simulation
/// (Tools/mm_contract_test.py: play.pause before this op, resume after) and the
/// test runs on an ISOLATED controller cloned from the live one (same
/// Database/CharacterBody/Schema/tuning, own native policy), so the live
/// controller is never mutated.
/// </summary>
public static class MMContractTest
{
    private const float Dt = 1.0f / 60.0f;

    private static readonly Transform TestWorld = new Transform(Float3.Zero, Quaternion.Identity, Float3.One);

    // Isolated test controller: same assets/tuning as the live scene object,
    // but a fresh native policy (own _policy/playback/trace state). Live
    // mirrors are never written by the checks below. Diagnostics stay off:
    // on/off must not change search decisions, and the isolated trace is never
    // exported.
    private static MotionMatchingController NewIsolated(MotionMatchingController live)
    {
        var iso = new MotionMatchingController
        {
            Schema = live.Schema,
            PlayerModel = live.PlayerModel,
            DatabasePath = live.DatabasePath,
            Database = live.Database,
            CharacterBody = live.CharacterBody,
            AllowBindingMismatch = live.AllowBindingMismatch,
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
            PlayRateMin = live.PlayRateMin,
            PlayRateMax = live.PlayRateMax,
            PlayRateMinSpeed = live.PlayRateMinSpeed,
            TeleportDistance = live.TeleportDistance,
            WarmupTime = live.WarmupTime,
            FastPathMaxAge = live.FastPathMaxAge,
            FastPathMaxFrames = live.FastPathMaxFrames,
            ReselectBanHistory = live.ReselectBanHistory,
            ReselectBanWindow = live.ReselectBanWindow,
            SameSegmentTwinBan = live.SameSegmentTwinBan,
            LoopTailExclusion = live.LoopTailExclusion,
            Diagnostics = false,
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
        if (sampleTimes == null || sampleTimes.Length == 0)
            return "CONTRACT FAIL: no trajectory sample times in the schema/database.";
        _pointCount = sampleTimes.Length;

        // Scope tags come from the baked database: the first two distinct tags
        // that own a looping clip. A single-tag database skips the two-tag checks.
        FindLoopTags(controller.Database, out string tagA, out string tagB);
        if (tagA == null)
            return "CONTRACT FAIL: database has no looping clip with a tag.";

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

        // 2: provider scope is respected (single-tag scope).
        {
            var prov = NewCanned(tagA, string.Empty);
            test.ResetPlayback();
            TickProvider(test, prov, 120);
            bool ok = test.ActiveTag == tagA;
            Check("scope-respected", ok,
                "activeTag='" + test.ActiveTag + "' winner='" + test.WinnerClipName + "'");
        }

        // 3: reset leaves the runtime usable: reset then soak must still
        // settle on the scoped tag.
        {
            var prov = NewCanned(tagA, string.Empty);
            test.ResetPlayback();
            TickProvider(test, prov, 120);
            bool ok = test.ActiveTag == tagA;
            Check("reset-then-soak", ok,
                "activeTag='" + test.ActiveTag + "' after ResetPlayback");
        }

        // 4: a per-frame input swap takes effect on the next update: scoped
        // soak, then an empty scope must flip the chooser to ext:empty, then
        // the scope must recover.
        {
            var idle = NewCanned(tagA, string.Empty);
            var empty = NewCanned(string.Empty, string.Empty);
            test.ResetPlayback();
            TickProvider(test, idle, 60);
            TickProvider(test, empty, 60);
            bool emptyOk = test.ChooserStatus.StartsWith("ext:empty");
            TickProvider(test, idle, 120);
            bool recoverOk = test.ActiveTag == tagA;
            Check("frame-swap", emptyOk && recoverOk,
                "emptyChooser='" + test.ChooserStatus + "' recoveredTag='" + test.ActiveTag + "'");
            test.ResetPlayback();
        }

        // 6: pose-out equals the internal search locals.
        {
            var prov = NewCanned(tagA, string.Empty);
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

        // 9: a scope change mid-clip invalidates the fast path: soak tag A
        // until the continuation holds, then switch the scope to tag B at
        // constant input. The core must NOT hold the stale clip; the chooser
        // must adopt the new filter within the soak window.
        if (tagB != null)
        {
            var scopeA = NewCanned(tagA, string.Empty);
            var scopeB = NewCanned(tagB, string.Empty);
            test.ResetPlayback();
            TickProvider(test, scopeA, 120);
            string before = test.ActiveTag ?? string.Empty;
            TickProvider(test, scopeB, 120);
            string after = test.ActiveTag ?? string.Empty;
            bool ok = before == tagA && after == tagB;
            Check("filter-change-invalidates-fastpath", ok,
                "before='" + before + "' after='" + after + "'");
        }
        // Same-scope LoopFilter flip must also take effect (Any -> LoopOnly
        // keeps the scope's loops; the recorded filter rides the trace for replay).
        {
            var loopOnly = NewCannedFiltered(tagA, string.Empty, 1, 0);
            TickProvider(test, loopOnly, 60);
            bool loopOk = (test.ActiveTag ?? string.Empty) == tagA;
            Check("loopfilter-change-takes-effect", loopOk,
                "tag='" + test.ActiveTag + "' chooser='" + test.ChooserStatus + "'");
            test.ResetPlayback();
        }
        // 10: one-shot progression hold. Pick a long (>1s) non-loop clip at mid-length under its own tag scope; while the clip identity is unchanged, CurrentTime must never decrease and SeekCount must not grow (far self-seeks barred). A clip change at any tick is a legal exit and passes.
        {
            test.ResetPlayback();
            TickProvider(test, NewCanned(tagA, string.Empty), 10);
            var playback = test.Playback;
            var db = test.Database;
            bool ok = false;
            string detail = "no playback/database";
            if (playback != null && db != null)
            {
                int target = -1;
                float targetLen = 0f;
                int clips = db.ClipCount;
                for (int c = 0; c < clips; c++)
                {
                    bool isLoop = false;
                    try { isLoop = db.GetClipLoop(c); } catch { continue; }
                    if (isLoop) continue;
                    float len = 0f;
                    try { var lens = db.ClipLengths; if (lens != null && c < lens.Length) len = lens[c]; } catch { len = 0f; }
                    if (len > 1.0f) { target = c; targetLen = len; break; }
                }
                if (target >= 0)
                {
                    string tag = tagA;
                    try { string t = db.GetClipTag(target); if (!string.IsNullOrEmpty(t)) tag = t; } catch { }
                    // One-shot-only scope: soak until the search itself has put a
                    // one-shot on screen, then watch its progression.
                    var prov = NewCannedFiltered(tag, string.Empty, 2, 0);
                    test.ResetPlayback();
                    TickProvider(test, prov, 60);
                    int playing = playback.CurrentClip;
                    bool playingOneShot = playing >= 0 && playing < clips && !db.GetClipLoop(playing);
                    if (!playingOneShot)
                    {
                        detail = "fixture: one-shot-only search did not put a one-shot on screen (clip " + playing + ")";
                    }
                    else
                    {
                        target = playing;
                        int seeksBefore = test.SeekCount;
                        float lastTime = playback.CurrentTime;
                        bool regressed = false;
                        bool changed = false;
                        detail = "held";
                        for (int i = 0; i < 30; i++)
                        {
                            TickProvider(test, prov, 1);
                            if (playback.CurrentClip != target) { changed = true; break; }
                            float now = playback.CurrentTime;
                            if (now < lastTime - 0.0001f) { regressed = true; detail = "time regressed " + lastTime + " -> " + now; break; }
                            lastTime = now;
                        }
                        if (changed) { ok = true; detail = "clip changed (legal exit) target=" + target; }
                        else if (!regressed && test.SeekCount == seeksBefore) { ok = true; detail = "held progression target=" + target; }
                        else if (!regressed) { detail = "seeks grew " + seeksBefore + " -> " + test.SeekCount; }
                    }
                }
                else { detail = "no non-loop clip > 1s (fixture missing)"; }
            }
            Check("oneshot-progression-hold", ok, detail);
        }


        test.ResetPlayback();

        // 11: changing the structural schema binding releases the native
        // session before the next host tick, so no playback/trace handle from
        // the old rig can survive a host rebind.
        {
            PoseSearchSchema original = test.Schema;
            PoseSearchSchema rebound = original;
            rebound.SampleRate = original.SampleRate + 1;
            test.Schema = rebound;
            bool released = !test.IsReady && test.Playback == null && test.Trace == null;
            Check("schema-rebind-releases-session", released,
                "ready=" + test.IsReady + " playback=" + (test.Playback != null)
                + " trace=" + (test.Trace != null));
            test.Schema = original;
            test.Update(Dt, TestWorld, CannedFrame(tagA, string.Empty, 0, 0));
        }

        // 12: host cleanup is public and idempotent; borrowed native handles
        // are gone immediately after the first call and remain gone after the
        // second call.
        test.Shutdown();
        test.Shutdown();
        Check("shutdown-idempotent", !test.IsReady && test.Playback == null && test.Trace == null,
            "ready=" + test.IsReady);

        string verdict = passed == total ? "CONTRACT PASS" : "CONTRACT FAIL";
        return verdict + ": " + passed + "/" + total + " checks\n" + string.Join("\n", lines);
    }

    private static int _pointCount = 1;

    private static void FindLoopTags(MotionMatchingDatabase db, out string tagA, out string tagB)
    {
        tagA = null;
        tagB = null;
        int clips = db.ClipCount;
        for (int c = 0; c < clips; c++)
        {
            string tag;
            try
            {
                if (!db.GetClipLoop(c))
                    continue;
                tag = db.GetClipTag(c);
            }
            catch
            {
                continue;
            }
            if (string.IsNullOrEmpty(tag))
                continue;
            if (tagA == null)
                tagA = tag;
            else if (tag != tagA)
            {
                tagB = tag;
                return;
            }
        }
    }

    private static MMContractTestProvider NewCanned(string allowed, string excluded)
    {
        return new MMContractTestProvider { FrameToReturn = CannedFrame(allowed, excluded, 0, 0) };
    }

    private static MMContractTestProvider NewCannedFiltered(string allowed, string excluded, int loop, int group)
    {
        return new MMContractTestProvider { FrameToReturn = CannedFrame(allowed, excluded, loop, group) };
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

    private static MotionMatchingFrameInput CannedFrame(string allowed, string excluded, int loop, int group)
    {
        return new MotionMatchingFrameInput
        {
            Points = new TrajectoryPoint[_pointCount],
            AllowedTagSet = allowed ?? string.Empty,
            ExcludedTagSet = excluded ?? string.Empty,
            LoopFilter = loop,
            ClipGroup = group,
            QueryYawRate = 0.0f,
        };
    }
}
