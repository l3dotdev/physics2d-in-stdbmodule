using StdbModule.Physics2D.Collision;

namespace StdbModule.Physics2D.Dynamics;

/// <summary>
/// This proxy is used internally to connect fixtures to the broad-phase.
/// </summary>
public struct FixtureProxy
{
    public AABB AABB;
    public int ChildIndex;
    public Fixture Fixture;
    public int ProxyId;
}