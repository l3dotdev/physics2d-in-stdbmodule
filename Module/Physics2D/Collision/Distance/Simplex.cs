using System.Diagnostics;
using StdbModule.Common;

namespace StdbModule.Physics2D.Collision;

/// <summary>
/// Used to warm start ComputeDistance.
/// Set count to zero on first call.
/// </summary>
public struct SimplexCache
{
    /// <summary>
    /// Length or area
    /// </summary>
    public ushort Count;

    /// <summary>
    /// Vertices on shape A
    /// </summary>
    public FixedArray3<byte> IndexA;

    /// <summary>
    /// Vertices on shape B
    /// </summary>
    public FixedArray3<byte> IndexB;

    public double Metric;
}

internal struct SimplexVertex
{
    /// <summary>
    /// Barycentric coordinate for closest point 
    /// </summary>
    public double A;

    /// <summary>
    /// wA index
    /// </summary>
    public int IndexA;

    /// <summary>
    /// wB index
    /// </summary>
    public int IndexB;

    /// <summary>
    /// wB - wA
    /// </summary>
    public Vector2 W;

    /// <summary>
    /// Support point in proxyA
    /// </summary>
    public Vector2 WA;

    /// <summary>
    /// Support point in proxyB
    /// </summary>
    public Vector2 WB;
}

internal struct Simplex
{
    internal int Count;
    internal FixedArray3<SimplexVertex> V;

    internal void ReadCache(ref SimplexCache cache, ref DistanceProxy proxyA, ref Transform transformA, ref DistanceProxy proxyB, ref Transform transformB)
    {
        Debug.Assert(cache.Count <= 3);

        // Copy data from cache.
        Count = cache.Count;
        for (int i = 0; i < Count; ++i)
        {
            SimplexVertex v = V[i];
            v.IndexA = cache.IndexA[i];
            v.IndexB = cache.IndexB[i];
            Vector2 wALocal = proxyA.Vertices[v.IndexA];
            Vector2 wBLocal = proxyB.Vertices[v.IndexB];
            v.WA = Transform.Multiply(ref wALocal, ref transformA);
            v.WB = Transform.Multiply(ref wBLocal, ref transformB);
            v.W = v.WB - v.WA;
            v.A = 0.0;
            V[i] = v;
        }

        // Compute the new simplex metric, if it is substantially different than
        // old metric then flush the simplex.
        if (Count > 1)
        {
            double metric1 = cache.Metric;
            double metric2 = GetMetric();
            if (metric2 < 0.5 * metric1 || 2.0 * metric1 < metric2 || metric2 < MathUtils.Epsilon)
            {
                // Reset the simplex.
                Count = 0;
            }
        }

        // If the cache is empty or invalid ...
        if (Count == 0)
        {
            SimplexVertex v = V[0];
            v.IndexA = 0;
            v.IndexB = 0;
            Vector2 wALocal = proxyA.Vertices[0];
            Vector2 wBLocal = proxyB.Vertices[0];
            v.WA = Transform.Multiply(ref wALocal, ref transformA);
            v.WB = Transform.Multiply(ref wBLocal, ref transformB);
            v.W = v.WB - v.WA;
            v.A = 1.0;
            V[0] = v;
            Count = 1;
        }
    }

    internal void WriteCache(ref SimplexCache cache)
    {
        cache.Metric = GetMetric();
        cache.Count = (ushort)Count;
        for (int i = 0; i < Count; ++i)
        {
            cache.IndexA[i] = (byte)V[i].IndexA;
            cache.IndexB[i] = (byte)V[i].IndexB;
        }
    }

    internal readonly Vector2 GetSearchDirection()
    {
        switch (Count)
        {
            case 1:
                return -V[0].W;

            case 2:
                {
                    Vector2 e12 = V[1].W - V[0].W;
                    double sgn = MathUtils.Cross(e12, -V[0].W);
                    if (sgn > 0.0)
                    {
                        // Origin is left of e12.
                        return new Vector2(-e12.Y, e12.X);
                    }
                    else
                    {
                        // Origin is right of e12.
                        return new Vector2(e12.Y, -e12.X);
                    }
                }

            default:
                Debug.Assert(false);
                return Vector2.Zero;
        }
    }

    internal readonly Vector2 GetClosestPoint()
    {
        switch (Count)
        {
            case 0:
                Debug.Assert(false);
                return Vector2.Zero;

            case 1:
                return V[0].W;

            case 2:
                return V[0].A * V[0].W + V[1].A * V[1].W;

            case 3:
                return Vector2.Zero;

            default:
                Debug.Assert(false);
                return Vector2.Zero;
        }
    }

    internal readonly void GetWitnessPoints(out Vector2 pA, out Vector2 pB)
    {
        switch (Count)
        {
            case 0:
                pA = Vector2.Zero;
                pB = Vector2.Zero;
                Debug.Assert(false);
                break;

            case 1:
                pA = V[0].WA;
                pB = V[0].WB;
                break;

            case 2:
                pA = V[0].A * V[0].WA + V[1].A * V[1].WA;
                pB = V[0].A * V[0].WB + V[1].A * V[1].WB;
                break;

            case 3:
                pA = V[0].A * V[0].WA + V[1].A * V[1].WA + V[2].A * V[2].WA;
                pB = pA;
                break;

            default:
                throw new Exception();
        }
    }

    internal double GetMetric()
    {
        switch (Count)
        {
            case 0:
                Debug.Assert(false);
                return 0.0;
            case 1:
                return 0.0;

            case 2:
                return (V[0].W - V[1].W).Length;

            case 3:
                return MathUtils.Cross(V[1].W - V[0].W, V[2].W - V[0].W);

            default:
                Debug.Assert(false);
                return 0.0;
        }
    }

    // Solve a line segment using barycentric coordinates.
    //
    // p = a1 * w1 + a2 * w2
    // a1 + a2 = 1
    //
    // The vector from the origin to the closest point on the line is
    // perpendicular to the line.
    // e12 = w2 - w1
    // dot(p, e) = 0
    // a1 * dot(w1, e) + a2 * dot(w2, e) = 0
    //
    // 2-by-2 linear system
    // [1      1     ][a1] = [1]
    // [w1.e12 w2.e12][a2] = [0]
    //
    // Define
    // d12_1 =  dot(w2, e12)
    // d12_2 = -dot(w1, e12)
    // d12 = d12_1 + d12_2
    //
    // Solution
    // a1 = d12_1 / d12
    // a2 = d12_2 / d12

    internal void Solve2()
    {
        Vector2 w1 = V[0].W;
        Vector2 w2 = V[1].W;
        Vector2 e12 = w2 - w1;

        // w1 region
        double d12_2 = -Vector2.Dot(w1, e12);
        if (d12_2 <= 0.0)
        {
            // a2 <= 0, so we clamp it to 0
            SimplexVertex v0 = V[0];
            v0.A = 1.0;
            V[0] = v0;
            Count = 1;
            return;
        }

        // w2 region
        double d12_1 = Vector2.Dot(w2, e12);
        if (d12_1 <= 0.0)
        {
            // a1 <= 0, so we clamp it to 0
            SimplexVertex v1 = V[1];
            v1.A = 1.0;
            V[1] = v1;
            Count = 1;
            V[0] = V[1];
            return;
        }

        // Must be in e12 region.
        double inv_d12 = 1.0 / (d12_1 + d12_2);
        SimplexVertex v0_2 = V[0];
        SimplexVertex v1_2 = V[1];
        v0_2.A = d12_1 * inv_d12;
        v1_2.A = d12_2 * inv_d12;
        V[0] = v0_2;
        V[1] = v1_2;
        Count = 2;
    }

    // Possible regions:
    // - points[2]
    // - edge points[0]-points[2]
    // - edge points[1]-points[2]
    // - inside the triangle
    internal void Solve3()
    {
        Vector2 w1 = V[0].W;
        Vector2 w2 = V[1].W;
        Vector2 w3 = V[2].W;

        // Edge12
        // [1      1     ][a1] = [1]
        // [w1.e12 w2.e12][a2] = [0]
        // a3 = 0
        Vector2 e12 = w2 - w1;
        double w1e12 = Vector2.Dot(w1, e12);
        double w2e12 = Vector2.Dot(w2, e12);
        double d12_1 = w2e12;
        double d12_2 = -w1e12;

        // Edge13
        // [1      1     ][a1] = [1]
        // [w1.e13 w3.e13][a3] = [0]
        // a2 = 0
        Vector2 e13 = w3 - w1;
        double w1e13 = Vector2.Dot(w1, e13);
        double w3e13 = Vector2.Dot(w3, e13);
        double d13_1 = w3e13;
        double d13_2 = -w1e13;

        // Edge23
        // [1      1     ][a2] = [1]
        // [w2.e23 w3.e23][a3] = [0]
        // a1 = 0
        Vector2 e23 = w3 - w2;
        double w2e23 = Vector2.Dot(w2, e23);
        double w3e23 = Vector2.Dot(w3, e23);
        double d23_1 = w3e23;
        double d23_2 = -w2e23;

        // Triangle123
        double n123 = MathUtils.Cross(ref e12, ref e13);

        double d123_1 = n123 * MathUtils.Cross(ref w2, ref w3);
        double d123_2 = n123 * MathUtils.Cross(ref w3, ref w1);
        double d123_3 = n123 * MathUtils.Cross(ref w1, ref w2);

        // w1 region
        if (d12_2 <= 0.0 && d13_2 <= 0.0)
        {
            SimplexVertex v0_1 = V[0];
            v0_1.A = 1.0;
            V[0] = v0_1;
            Count = 1;
            return;
        }

        // e12
        if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0)
        {
            double inv_d12 = 1.0 / (d12_1 + d12_2);
            SimplexVertex v0_2 = V[0];
            SimplexVertex v1_2 = V[1];
            v0_2.A = d12_1 * inv_d12;
            v1_2.A = d12_2 * inv_d12;
            V[0] = v0_2;
            V[1] = v1_2;
            Count = 2;
            return;
        }

        // e13
        if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0)
        {
            double inv_d13 = 1.0 / (d13_1 + d13_2);
            SimplexVertex v0_3 = V[0];
            SimplexVertex v2_3 = V[2];
            v0_3.A = d13_1 * inv_d13;
            v2_3.A = d13_2 * inv_d13;
            V[0] = v0_3;
            V[2] = v2_3;
            Count = 2;
            V[1] = V[2];
            return;
        }

        // w2 region
        if (d12_1 <= 0.0 && d23_2 <= 0.0)
        {
            SimplexVertex v1_4 = V[1];
            v1_4.A = 1.0;
            V[1] = v1_4;
            Count = 1;
            V[0] = V[1];
            return;
        }

        // w3 region
        if (d13_1 <= 0.0 && d23_1 <= 0.0)
        {
            SimplexVertex v2_5 = V[2];
            v2_5.A = 1.0;
            V[2] = v2_5;
            Count = 1;
            V[0] = V[2];
            return;
        }

        // e23
        if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0)
        {
            double inv_d23 = 1.0 / (d23_1 + d23_2);
            SimplexVertex v1_6 = V[1];
            SimplexVertex v2_6 = V[2];
            v1_6.A = d23_1 * inv_d23;
            v2_6.A = d23_2 * inv_d23;
            V[1] = v1_6;
            V[2] = v2_6;
            Count = 2;
            V[0] = V[2];
            return;
        }

        // Must be in triangle123
        double inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3);
        SimplexVertex v0_7 = V[0];
        SimplexVertex v1_7 = V[1];
        SimplexVertex v2_7 = V[2];
        v0_7.A = d123_1 * inv_d123;
        v1_7.A = d123_2 * inv_d123;
        v2_7.A = d123_3 * inv_d123;
        V[0] = v0_7;
        V[1] = v1_7;
        V[2] = v2_7;
        Count = 3;
    }
}
