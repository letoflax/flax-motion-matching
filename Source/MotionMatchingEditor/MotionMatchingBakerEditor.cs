using FlaxEngine;

namespace MotionMatching;

// P3: the runtime (MotionMatchingController) is a plain C# class with zero
// scene presence, so there is no inspector CustomEditor for it anymore. The
// game-side facade (MotionMatchingPlayer, Game assembly) carries the editor
// button through its own CustomEditor (Source/Game/Player/
// MotionMatchingPlayerEditor.cs), which opens ManageMotionMatchingWindow on
// the facade's controller object. This file stays as the documented removal
// point so the old [CustomEditor(typeof(MotionMatchingController))] binding
// has an explicit tombstone instead of a silent disappearance.
public static class MotionMatchingBakerEditorTombstone
{
    public static string Note => "P3 removed: use the MotionMatchingPlayer inspector button (game facade).";
}
