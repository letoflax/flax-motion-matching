using Flax.Build;

public class MotionMatchingEditorTarget : GameProjectEditorTarget
{
    /// <inheritdoc />
    public override void Init()
    {
        base.Init();

        // Reference the modules for editor: core + editor services + sample.
        Modules.Add("MotionMatching");
        Modules.Add("MotionMatchingEditor");
        Modules.Add("MotionMatchingLocomotion");
    }
}
