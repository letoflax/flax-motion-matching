using System.IO;
using FlaxEditor;
using FlaxEditor.CustomEditors;
using FlaxEditor.CustomEditors.Elements;
using FlaxEngine;

namespace MotionMatching;

/// <summary>
/// Single management UI for the motion-matching database. Replaces the old
/// row of panel buttons: register / clear / import / export / bake pipeline
/// / tuning live here. Import REPLACES the whole registry (no merge);
/// the shipped Content/MotionMatching/DefaultClips.json restores the full
/// set (it supersedes the old Rebuild All button).
/// </summary>
public class ManageMotionMatchingWindow : CustomEditorWindow
{
    private readonly MotionMatchingController _controller;
    private LabelElement _status;

    public ManageMotionMatchingWindow(MotionMatchingController controller)
    {
        _controller = controller;
    }

    public override void Initialize(LayoutElementsContainer layout)
    {
        layout.Label("Manage Motion Matching Data", TextAlignment.Center);
        layout.Space(4);
        layout.Button("Register Animation Clips").Button.Clicked += () =>
        {
            if (_controller != null)
                new RegisterClipsWindow(_controller).Show();
        };
        layout.Button("Import Registry JSON (replaces all)").Button.Clicked += ImportRegistry;
        layout.Button("Export Registry JSON").Button.Clicked += ExportRegistry;
        layout.Button("Clear Animation Clips").Button.Clicked += () =>
        {
            if (_controller == null)
                return;
            MotionMatchingEditorService.ClearAllAnimationClips(_controller);
            SetStatus("Registry cleared.");
        };
        layout.Space(8);
        layout.Button("Bake + Validate + Self-Test").Button.Clicked += () =>
        {
            if (_controller == null)
                return;
            SetStatus(MotionMatchingEditorService.BakeValidateAndTest(_controller));
        };
        layout.Button("Open Tuning (M6)").Button.Clicked += () =>
        {
            if (_controller != null)
                new MotionMatchingTuningWindow(_controller).Show();
        };
        layout.Space(8);
        _status = layout.Label("Ready.");
        RefreshStatus();
    }

    private void ImportRegistry()
    {
        if (_controller == null)
            return;
        const string filter = "Clip Registry JSON\0*.json\0";
        if (!FileSystem.ShowOpenFileDialog(
                Editor.Instance.Windows.MainWindow,
                Globals.ProjectContentFolder,
                filter,
                false,
                "Import Clip Registry (replaces all clips)",
                out string[] files) || files == null || files.Length == 0)
        {
            return;
        }
        SetStatus(MotionMatchingEditorService.ImportClipRegistry(_controller, files[0]));
        RefreshStatus();
    }

    private void ExportRegistry()
    {
        if (_controller == null)
            return;
        string path = Path.Combine(
            Globals.ProjectContentFolder, "MotionMatching", "RegistryExport.json");
        SetStatus(MotionMatchingEditorService.ExportClipRegistry(_controller, path));
    }

    private void RefreshStatus()
    {
        if (_controller == null || _controller.Database == null)
        {
            SetStatus("No database loaded.");
            return;
        }
        SetStatus($"Registry: {_controller.Database.AnimationClipsCount} clips, " +
            $"{_controller.Database.SampleCount} samples, baked={_controller.Database.IsBaked}.");
    }

    private void SetStatus(string text)
    {
        if (_status != null)
            _status.Label.Text = text;
    }
}
