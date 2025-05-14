using StdbModule.Common;

namespace StdbModule.Physics2D.Collision;

/// <summary>
/// Used for computing contact manifolds.
/// </summary>
public struct ClipVertex
{
    public ContactID ID;
    public Vector2 V;
}
