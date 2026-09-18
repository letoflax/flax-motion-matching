namespace MotionMatching;

/// <summary>
/// One gait band: planar speeds up to MaxSpeed resolve to Tag. Bands are
/// sorted ascending at use; the first entry is the rest band. Plain data,
/// owned by the locomotion integration (not the core contract): a custom
/// provider with its own vocabulary never needs this type.
/// </summary>
[System.Serializable]
public struct GaitBand
{
    public string Tag;
    public float MaxSpeed;
}
