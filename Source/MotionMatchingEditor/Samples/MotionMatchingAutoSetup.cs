using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using FlaxEngine;

namespace MotionMatching;

/// <summary>
/// Optional flax-test sample setup: registers every animation under
/// Content/Animations/Humanoid with auto-inferred tags, then bakes.
/// Tag rule (lowercase, deterministic, whole-token matching):
/// - filename containing the token "turn" overrides to "turn";
/// - else first gait token by POSITION ("Run_to_Walk" -> "run": the clip is
///   still A until it ends, and the chooser needs it while running);
/// - else parent folder name; else "misc".
/// Position-first (not priority-first): "Walk_to_Run" must be walk, not run.
/// Folder-as-gait would tag transitions by destination; tokens tag by source.
/// </summary>
public static class MotionMatchingAutoSetup
{
    private static readonly HashSet<string> SkipFolders = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
    {
        "crouch",
        "aimoffset",
        "traversal",
    };
    /// <summary>
    /// True when any folder segment of the file (relative to the Humanoid
    /// root) is excluded. Segment-wise so nested sets like
    /// Traversal/Catch/Hurdle are caught, not just top-level folders.
    /// </summary>
    public static bool IsSkipped(string filePath, string root)
    {
        string relative = filePath;
        if (relative.StartsWith(root, StringComparison.OrdinalIgnoreCase))
            relative = relative.Substring(root.Length);
        foreach (string segment in relative.Split(
            new char[] { Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar },
            StringSplitOptions.RemoveEmptyEntries))
        {
            if (SkipFolders.Contains(segment))
                return true;
        }
        return false;
    }

    // Gait vocabulary for first-token tagging (checked by position in name).
    private static readonly HashSet<string> GaitTokens = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
    {
        "jump", "sprint", "run", "walk", "idle",
    };

    // Direction tokens (whole-token match): single letters are compass
    // directions, two-letter tokens are their diagonal combos (bl/br lean
    // backward, fl/fr lean forward, ll/rr/lr/rl are lateral). Foot markers
    // ("lfoot"/"rfoot") are single tokens and never collide with these.
    private static readonly HashSet<string> BackwardDirs = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
    {
        "b", "bl", "br",
    };
    private static readonly HashSet<string> StrafeDirs = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
    {
        "l", "r", "ll", "rr", "lr", "rl",
    };
    // Turn-like keywords shared with the baked DB flag (C++ IsTurnLikePath in
    // Baker/MotionMatchingBakeHeuristics.h) — keep the two lists identical.
    // This whole file is baker-config (editor import path), not runtime.
    public static readonly HashSet<string> TurnLikeKeywords = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
    {
        "turn", "arc", "reface", "pivot", "spin",
    };

    // Splits a filename into whole tokens for exact matching (avoids "stop"
    // matching a "to" rule or partial hits like "return" matching "turn").
    private static string[] Tokenize(string filePath)
    {
        return Path.GetFileNameWithoutExtension(filePath).ToLowerInvariant()
            .Split(new char[] { '_', '-', ' ' }, StringSplitOptions.RemoveEmptyEntries);
    }

    // Multi-tag inference: every applicable label is emitted at once
    // ("Run_Turn_90" -> [run, turn_travel]) instead of forcing one bucket.
    // Compat invariant: the old single-tag InferTag output is ALWAYS element
    // of the new set, so existing queries only gain candidates, never lose.
    // Domain rules (verified against baked root travel: 10 stand/idle turns
    // travel exactly 0.0u, 40 walk/run/sprint turns travel 780-2170u, zero
    // borderline cases): gait-carrying turns travel, the rest turn in place.
    // Unknown-gait turns default in-place (rest-safe: a misclassified
    // traveling turn merely loses, a misclassified in-place turn would
    // teleport into rest queries).
    public static string[] InferTags(string filePath)
    {
        var set = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        string[] tokens = Tokenize(filePath);
        // Crouch wins outright: a crouching skeleton must never contest
        // stand/walk/run queries (observed: crouch idle break winning rest
        // with zero yaw). Rebake picks this up; registry edit below covers
        // the live DB path without a rebake.
        if (tokens.Contains("crouch", StringComparer.OrdinalIgnoreCase))
        {
            set.Add("crouch");
            return set.ToArray();
        }
        bool jump = tokens.Contains("jump", StringComparer.OrdinalIgnoreCase);
        bool hasGait = tokens.Any(t => GaitTokens.Contains(t)) && !jump;
        string gait = tokens.FirstOrDefault(t => GaitTokens.Contains(t))?.ToLowerInvariant();
        if (tokens.Contains("turn", StringComparer.OrdinalIgnoreCase))
        {
            if (tokens.Contains("walk", StringComparer.OrdinalIgnoreCase) ||
                tokens.Contains("run", StringComparer.OrdinalIgnoreCase) ||
                tokens.Contains("sprint", StringComparer.OrdinalIgnoreCase))
            {
                set.Add("turn_travel");
                set.Add(gait ?? "run");
            }
            else
            {
                set.Add("turn_inplace");
            }
            if (jump)
                set.Add("jump");
        }
        else
        {
            // Backward set (b/bl/br): backpedaling goes "back" plus its gait
            // when known; jumps keep gait-first (airborne chooser needs them
            // in "jump"). Backward wins direction ties (bl leans back first).
            if (tokens.Any(t => BackwardDirs.Contains(t)) && !jump)
            {
                set.Add("back");
                if (hasGait)
                    set.Add(gait);
            }
            // Lateral set (l/r/ll/rr/lr/rl): "strafe" plus gait when known.
            // Turn-like names (arcs, refaces, pivots) ALSO get turn_travel —
            // with multi-tags an arc no longer has to choose between its gait
            // and its turn identity.
            else if (tokens.Any(t => StrafeDirs.Contains(t)) && !jump)
            {
                set.Add("strafe");
                if (hasGait)
                    set.Add(gait);
                if (tokens.Any(t => TurnLikeKeywords.Contains(t)))
                    set.Add(hasGait ? "turn_travel" : "turn_inplace");
            }
            // Turn-like keyword without a "turn" token (pivot_180, arc_45):
            // same domain split by gait presence; without a gait token the
            // folder fallback is ALSO kept so the old single tag survives
            // in-set (compat invariant).
            else if (tokens.Any(t => TurnLikeKeywords.Contains(t)))
            {
                set.Add(hasGait ? "turn_travel" : "turn_inplace");
                if (hasGait)
                    set.Add(gait);
                else
                {
                    string folder = Path.GetFileName(Path.GetDirectoryName(filePath) ?? string.Empty).ToLowerInvariant();
                    set.Add(string.IsNullOrEmpty(folder) ? "misc" : folder);
                }
                if (jump)
                    set.Add("jump");
            }
            else if (!string.IsNullOrEmpty(gait))
            {
                set.Add(gait);
            }
        }

        if (set.Count == 0)
        {
            // Folder is last-resort fallback (old behavior, kept in-set).
            string folder = Path.GetFileName(Path.GetDirectoryName(filePath) ?? string.Empty).ToLowerInvariant();
            set.Add(string.IsNullOrEmpty(folder) ? "misc" : folder);
        }
        var tags = set.ToArray();
        Array.Sort(tags, StringComparer.Ordinal);
        return tags;
    }

    // Loop flag = whole "loop" token in the filename.
    public static bool InferLoop(string filePath)
    {
        return Tokenize(filePath).Contains("loop", StringComparer.OrdinalIgnoreCase);
    }

    /// <summary>
    /// Clears the registry, registers everything grouped by (tag, loop) so
    /// each batch shares one loop flag, then bakes. Logs per-tag counts.
    /// Writes Cache/MMRebuild.json markers (started/finished) so automation
    /// can poll a long rebuild from disk instead of holding a bridge call.
    /// </summary>
    public static void RebuildAll(MotionMatchingController controller)
    {
        WriteRebuildMarker("started", string.Empty);
        try
        {
            RebuildAllInner(controller);
            WriteRebuildMarker("finished", "ok");
        }
        catch (System.Exception ex)
        {
            WriteRebuildMarker("finished", "error: " + ex.Message);
            throw;
        }
    }

    private static void WriteRebuildMarker(string phase, string result)
    {
        try
        {
            // Plugin boundary: the editor resolves the Cache folder itself
            // instead of calling game-side automation helpers.
            string folder = MotionMatchingEditorService.TraceDefaultFolder();
            if (folder == null)
                return;
            string json = "{\"phase\":\"" + phase + "\",\"time\":\"" +
                System.DateTime.UtcNow.ToString("u") + "\",\"result\":\"" +
                result.Replace("\"", "'") + "\"}";
            File.WriteAllText(Path.Combine(folder, "MMRebuild.json"), json);
        }
        catch
        {
        }
    }

    private static void RebuildAllInner(MotionMatchingController controller)
    {
        if (controller == null)
            return;
        if (!controller.PlayerModel)
        {
            Debug.LogWarning("Motion matching auto-setup stopped: PlayerModel is not set.");
            return;
        }

        string root = Path.Combine(Globals.ProjectContentFolder, "Animations", "Humanoid");
        if (!Directory.Exists(root))
        {
            Debug.LogWarning($"Motion matching auto-setup stopped: folder not found: '{root}'.");
            return;
        }

        string[] files = Directory.GetFiles(root, "*.flax", SearchOption.AllDirectories)
            .OrderBy(f => f, StringComparer.OrdinalIgnoreCase)
            .ToArray();
        if (files.Length == 0)
        {
            Debug.LogWarning($"Motion matching auto-setup stopped: no animations in '{root}'.");
            return;
        }

        var groups = new Dictionary<string, List<string>>();
        int skipped = 0;
        foreach (string file in files)
        {
            if (IsSkipped(file, root))
            {
                skipped++;
                continue;
            }
            string key = string.Join("+", InferTags(file)) + "|" + (InferLoop(file) ? "loop" : "once");
            if (!groups.TryGetValue(key, out var list))
            {
                list = new List<string>();
                groups[key] = list;
            }
            list.Add(file);
        }

        MotionMatchingEditorService.ClearAllAnimationClips(controller);
        foreach (var pair in groups.OrderBy(p => p.Key, StringComparer.Ordinal))
        {
            string[] parts = pair.Key.Split('|');
            MotionMatchingEditorService.AddClipFiles(controller, parts[0].Split('+'), pair.Value.ToArray(), parts[1] == "loop");
        }

        Debug.Log($"Motion matching auto-setup registered {files.Length - skipped} clips in {groups.Count} groups " +
            $"(skipped {skipped} in [{string.Join(", ", SkipFolders)}]): " +
            string.Join(", ", groups.OrderBy(p => p.Key).Select(p => $"{p.Key}={p.Value.Count}")));
        MotionMatchingEditorService.BakeDatabase(controller);
    }
}
