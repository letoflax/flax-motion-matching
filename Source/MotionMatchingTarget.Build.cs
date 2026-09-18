using Flax.Build;

public class MotionMatchingTarget : GameProjectTarget
{
    /// <inheritdoc />
    public override void Init()
    {
        base.Init();

        // Reference the modules for game: core runtime + sample integration.
        // The editor module is editor-only (see MotionMatchingEditorTarget).
        Modules.Add("MotionMatching");
        Modules.Add("MotionMatchingLocomotion");
    }
}
