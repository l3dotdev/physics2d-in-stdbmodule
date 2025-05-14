using StdbModule.Common;
using StdbModule.Physics2D.Dynamics;

namespace StdbModule.Physics2D.Collision;

public interface IBroadPhase : IBroadPhase<FixtureProxy> { }

public interface IBroadPhase<TNode>
    where TNode : struct
{
    int ProxyCount { get; }
    void UpdatePairs(BroadphaseDelegate callback);

    bool TestOverlap(int proxyIdA, int proxyIdB);

    int AddProxy(ref AABB aabb);

    void RemoveProxy(int proxyId);

    void MoveProxy(int proxyId, ref AABB aabb, Vector2 displacement);

    void SetProxy(int proxyId, ref TNode proxy);

    TNode GetProxy(int proxyId);

    void TouchProxy(int proxyId);

    void GetFatAABB(int proxyId, out AABB aabb);

    void Query(BroadPhaseQueryCallback callback, ref AABB aabb);

    void RayCast(BroadPhaseRayCastCallback callback, ref RayCastInput input);

    void ShiftOrigin(Vector2 newOrigin);
}

public delegate void BroadphaseDelegate(int proxyIdA, int proxyIdB);
public delegate bool BroadPhaseQueryCallback(int proxyId);
public delegate double BroadPhaseRayCastCallback(ref RayCastInput input, int proxyId);