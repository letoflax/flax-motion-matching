using System;
using System.IO;
using FlaxEngine;
using FlaxEditor;
using FlaxEditor.CustomEditors;
using FlaxEditor.CustomEditors.Editors;
using FlaxEditor.CustomEditors.Elements;

namespace MotionMatching;

/// <summary>
/// MotionMatchingBakerEditor class.
/// </summary>
[CustomEditor(typeof(MotionMatchingController))]
public class MotionMatchingBakerEditor : GenericEditor
{
    public override void Initialize(LayoutElementsContainer layout)
    {
        base.Initialize(layout);
        var controller = IsSingleObject ? Values[0] as MotionMatchingController : null;

        // if (controller != null)
        // {
        //     int count = Math.Min(
        //         controller.AnimationTags.Length,
        //         controller.AnimationClipCountsByTag.Length);
        //     for (int i = 0; i < count; i++)
        //         layout.Label(controller.AnimationTags[i], controller.AnimationClipCountsByTag[i].ToString());
        // }
        // layout.Space(12);

        layout.Button("Manage Motion Matching Data").Button.Clicked += () =>
        {
            if (controller != null)
                new ManageMotionMatchingWindow(controller).Show();
        };
    }
}
