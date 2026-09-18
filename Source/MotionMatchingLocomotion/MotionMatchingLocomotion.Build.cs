using Flax.Build;
using Flax.Build.NativeCpp;

public class MotionMatchingLocomotion : GameModule
{
    /// <inheritdoc />
    public override void Setup(BuildOptions options)
    {
        base.Setup(options);

        options.ScriptingAPI.IgnoreMissingDocumentationWarnings = true;

        // Default locomotion integration sample: C# only, no native code.
        // Implements the plugin input contract without any player code, so
        // any host can reuse it. Public dependency keeps native link order
        // against the runtime module's .lib.
        options.PublicDependencies.Add("MotionMatching");
    }
}
