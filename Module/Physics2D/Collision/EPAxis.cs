namespace StdbModule.Physics2D.Collision;

/// <summary>
/// This structure is used to keep track of the best separating axis.
/// </summary>
public struct EPAxis
{
    public int Index;
    public double Separation;
    public EPAxisType Type;
}