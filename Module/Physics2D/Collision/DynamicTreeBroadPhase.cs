using StdbModule.Common;
using StdbModule.Physics2D.Dynamics;

namespace StdbModule.Physics2D.Collision;

internal struct Pair : IComparable<Pair>
{
    public int ProxyIdA;
    public int ProxyIdB;

    #region IComparable<Pair> Members

    public readonly int CompareTo(Pair other)
    {
        if (ProxyIdB < other.ProxyIdB)
        {
            return -1;
        }
        if (ProxyIdB == other.ProxyIdB)
        {
            if (ProxyIdA < other.ProxyIdA)
            {
                return -1;
            }
            if (ProxyIdA == other.ProxyIdA)
            {
                return 0;
            }
        }

        return 1;
    }

    #endregion
}


/// <summary>
/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
/// </summary>
public class DynamicTreeBroadPhase : DynamicTreeBroadPhase<FixtureProxy>, IBroadPhase { }

/// <summary>
/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
/// </summary>
public class DynamicTreeBroadPhase<TNode> : IBroadPhase<TNode>
    where TNode : struct
{
    private const int NullProxy = -1;
    private int[] _moveBuffer;
    private int _moveCapacity;
    private int _moveCount;

    private Pair[] _pairBuffer;
    private int _pairCapacity;
    private int _pairCount;
    private int _proxyCount;
    private BroadPhaseQueryCallback _queryCallbackCache;
    private int _queryProxyId;
    private DynamicTree<TNode> _tree = new DynamicTree<TNode>();

    /// <summary>
    /// Constructs a new broad phase based on the dynamic tree implementation
    /// </summary>
    public DynamicTreeBroadPhase()
    {
        _queryCallbackCache = new BroadPhaseQueryCallback(QueryCallback);
        _proxyCount = 0;

        _pairCapacity = 16;
        _pairCount = 0;
        _pairBuffer = new Pair[_pairCapacity];

        _moveCapacity = 16;
        _moveCount = 0;
        _moveBuffer = new int[_moveCapacity];
    }

    /// <summary>
    /// Get the number of proxies.
    /// </summary>
    /// <value>The proxy count.</value>
    public int ProxyCount
    {
        get { return _proxyCount; }
    }

    /// <summary>
    /// Create a proxy with an initial AABB. Pairs are not reported until
    /// UpdatePairs is called.
    /// </summary>
    /// <param name="proxy">The user data.</param>
    /// <returns></returns>
    public int AddProxy(ref AABB aabb)
    {
        int proxyId = _tree.AddProxy(ref aabb);
        ++_proxyCount;
        BufferMove(proxyId);

        return proxyId;
    }

    /// <summary>
    /// Destroy a proxy. It is up to the client to remove any pairs.
    /// </summary>
    /// <param name="proxyId">The proxy id.</param>
    public void RemoveProxy(int proxyId)
    {
        UnBufferMove(proxyId);
        --_proxyCount;
        _tree.RemoveProxy(proxyId);
    }

    public void MoveProxy(int proxyId, ref AABB aabb, Vector2 displacement)
    {
        bool buffer = _tree.MoveProxy(proxyId, ref aabb, displacement);
        if (buffer)
        {
            BufferMove(proxyId);
        }
    }

    public void TouchProxy(int proxyId)
    {
        BufferMove(proxyId);
    }

    private void BufferMove(int proxyId)
    {
        if (_moveCount == _moveCapacity)
        {
            int[] oldBuffer = _moveBuffer;
            _moveCapacity *= 2;
            _moveBuffer = new int[_moveCapacity];
            Array.Copy(oldBuffer, _moveBuffer, _moveCount);
        }

        _moveBuffer[_moveCount] = proxyId;
        ++_moveCount;
    }

    private void UnBufferMove(int proxyId)
    {
        for (int i = 0; i < _moveCount; ++i)
        {
            if (_moveBuffer[i] == proxyId)
            {
                _moveBuffer[i] = NullProxy;
            }
        }
    }

    /// <summary>
    /// This is called from DynamicTree.Query when we are gathering pairs.
    /// </summary>
    /// <param name="proxyId"></param>
    /// <returns></returns>
    private bool QueryCallback(int proxyId)
    {
        // A proxy cannot form a pair with itself.
        if (proxyId == _queryProxyId)
        {
            return true;
        }

        // Grow the pair buffer as needed.
        if (_pairCount == _pairCapacity)
        {
            Pair[] oldBuffer = _pairBuffer;
            _pairCapacity *= 2;
            _pairBuffer = new Pair[_pairCapacity];
            Array.Copy(oldBuffer, _pairBuffer, _pairCount);
        }

        _pairBuffer[_pairCount].ProxyIdA = Math.Min(proxyId, _queryProxyId);
        _pairBuffer[_pairCount].ProxyIdB = Math.Max(proxyId, _queryProxyId);
        ++_pairCount;

        return true;
    }

    /// <summary>
    /// Get the AABB for a proxy.
    /// </summary>
    /// <param name="proxyId">The proxy id.</param>
    /// <param name="aabb">The aabb.</param>
    public void GetFatAABB(int proxyId, out AABB aabb)
    {
        _tree.GetFatAABB(proxyId, out aabb);
    }

    public void SetProxy(int proxyId, ref TNode proxy)
    {
        _tree.SetUserData(proxyId, proxy);
    }

    /// <summary>
    /// Get user data from a proxy. Returns null if the id is invalid.
    /// </summary>
    /// <param name="proxyId">The proxy id.</param>
    /// <returns></returns>
    public TNode GetProxy(int proxyId)
    {
        return _tree.GetUserData(proxyId);
    }

    /// <summary>
    /// Test overlap of fat AABBs.
    /// </summary>
    /// <param name="proxyIdA">The proxy id A.</param>
    /// <param name="proxyIdB">The proxy id B.</param>
    /// <returns></returns>
    public bool TestOverlap(int proxyIdA, int proxyIdB)
    {
        return _tree.TestFatAABBOverlap(proxyIdA, proxyIdB);
    }

    /// <summary>
    /// Update the pairs. This results in pair callbacks. This can only add pairs.
    /// </summary>
    /// <param name="callback">The callback.</param>
    public void UpdatePairs(BroadphaseDelegate callback)
    {
        // Reset pair buffer
        _pairCount = 0;

        // Perform tree queries for all moving proxies.
        for (int j = 0; j < _moveCount; ++j)
        {
            _queryProxyId = _moveBuffer[j];
            if (_queryProxyId == NullProxy)
            {
                continue;
            }

            // We have to query the tree with the fat AABB so that
            // we don't fail to create a pair that may touch later.
            AABB fatAABB = _tree.GetFatAABB(_queryProxyId);

            // Query tree, create pairs and add them pair buffer.
            _tree.Query(_queryCallbackCache, ref fatAABB);
        }

        // Reset move buffer
        _moveCount = 0;

        // Sort the pair buffer to expose duplicates.
        Array.Sort(_pairBuffer, 0, _pairCount);

        // Send the pairs back to the client.
        int i = 0;
        while (i < _pairCount)
        {
            Pair primaryPair = _pairBuffer[i];

            callback(primaryPair.ProxyIdA, primaryPair.ProxyIdB);
            ++i;

            // Skip any duplicate pairs.
            while (i < _pairCount)
            {
                Pair pair = _pairBuffer[i];
                if (pair.ProxyIdA != primaryPair.ProxyIdA || pair.ProxyIdB != primaryPair.ProxyIdB)
                {
                    break;
                }
                ++i;
            }
        }

        // Try to keep the tree balanced.
        //_tree.Rebalance(4);
    }

    /// <summary>
    /// Query an AABB for overlapping proxies. The callback class
    /// is called for each proxy that overlaps the supplied AABB.
    /// </summary>
    /// <param name="callback">The callback.</param>
    /// <param name="aabb">The aabb.</param>
    public void Query(BroadPhaseQueryCallback callback, ref AABB aabb)
    {
        _tree.Query(callback, ref aabb);
    }

    /// <summary>
    /// Ray-cast against the proxies in the tree. This relies on the callback
    /// to perform a exact ray-cast in the case were the proxy contains a shape.
    /// The callback also performs the any collision filtering. This has performance
    /// roughly equal to k * log(n), where k is the number of collisions and n is the
    /// number of proxies in the tree.
    /// </summary>
    /// <param name="callback">A callback class that is called for each proxy that is hit by the ray.</param>
    /// <param name="input">The ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).</param>
    public void RayCast(BroadPhaseRayCastCallback callback, ref RayCastInput input)
    {
        _tree.RayCast(callback, ref input);
    }

    public void ShiftOrigin(Vector2 newOrigin)
    {
        _tree.ShiftOrigin(newOrigin);
    }

    /// <summary>
    /// Get the tree quality based on the area of the tree.
    /// </summary>
    public double TreeQuality
    {
        get { return _tree.AreaRatio; }
    }

    /// <summary>
    /// Gets the balance of the tree.
    /// </summary>
    public int TreeBalance
    {
        get { return _tree.MaxBalance; }
    }

    /// <summary>
    /// Gets the height of the tree.
    /// </summary>
    public int TreeHeight
    {
        get { return _tree.Height; }
    }
}
