using System.Diagnostics;
using StdbModule.Common;

namespace StdbModule.Physics2D.Collision;

/// <summary>
/// Input for Distance.ComputeDistance().
/// You have to option to use the shape radii in the computation. 
/// </summary>
public struct DistanceInput
{
    public DistanceProxy ProxyA;
    public DistanceProxy ProxyB;
    public Transform TransformA;
    public Transform TransformB;
    public bool UseRadii;
}

/// <summary>
/// Output for Distance.ComputeDistance().
/// </summary>
public struct DistanceOutput
{
    public double Distance;

    /// <summary>
    /// Number of GJK iterations used
    /// </summary>
    public int Iterations;

    /// <summary>
    /// Closest point on shapeA
    /// </summary>
    public Vector2 PointA;

    /// <summary>
    /// Closest point on shapeB
    /// </summary>
    public Vector2 PointB;
}

/// <summary>
/// The Gilbert–Johnson–Keerthi distance algorithm that provides the distance between shapes.
/// </summary>
public static class Distance
{
    /// <summary>
    /// The number of calls made to the ComputeDistance() function.
    /// Note: This is only activated when Settings.EnableDiagnostics = true
    /// </summary>
    [ThreadStatic]
    public static int GJKCalls;

    /// <summary>
    /// The number of iterations that was made on the last call to ComputeDistance().
    /// Note: This is only activated when Settings.EnableDiagnostics = true
    /// </summary>
    [ThreadStatic]
    public static int GJKIters;

    /// <summary>
    /// The maximum numer of iterations ever mae with calls to the CompteDistance() funtion.
    /// Note: This is only activated when Settings.EnableDiagnostics = true
    /// </summary>
    [ThreadStatic]
    public static int GJKMaxIters;

    public static void ComputeDistance(out DistanceOutput output, out SimplexCache cache, DistanceInput input)
    {
        cache = new SimplexCache();

        if (Settings.EnableDiagnostics) //FPE: We only gather diagnostics when enabled
            ++GJKCalls;

        // Initialize the simplex.
        Simplex simplex = new();
        simplex.ReadCache(ref cache, ref input.ProxyA, ref input.TransformA, ref input.ProxyB, ref input.TransformB);

        // These store the vertices of the last simplex so that we
        // can check for duplicates and prevent cycling.
        FixedArray3<int> saveA = new();
        FixedArray3<int> saveB = new();

        //double distanceSqr1 = Settings.MaxDouble;

        // Main iteration loop.
        int iter = 0;
        while (iter < Settings.MaxGJKIterations)
        {
            // Copy simplex so we can identify duplicates.
            int saveCount = simplex.Count;
            for (int i = 0; i < saveCount; ++i)
            {
                saveA[i] = simplex.V[i].IndexA;
                saveB[i] = simplex.V[i].IndexB;
            }

            switch (simplex.Count)
            {
                case 1:
                    break;
                case 2:
                    simplex.Solve2();
                    break;
                case 3:
                    simplex.Solve3();
                    break;
                default:
                    Debug.Assert(false);
                    break;
            }

            // If we have 3 points, then the origin is in the corresponding triangle.
            if (simplex.Count == 3)
            {
                break;
            }

            //FPE: This code was not used anyway.
            // Compute closest point.
            //Vector2 p = simplex.GetClosestPoint();
            //double distanceSqr2 = p.LengthSquared;

            // Ensure progress
            //if (distanceSqr2 >= distanceSqr1)
            //{
            //break;
            //}
            //distanceSqr1 = distanceSqr2;

            // Get search direction.
            Vector2 d = simplex.GetSearchDirection();

            // Ensure the search direction is numerically fit.
            if (d.LengthSquared < MathUtils.Epsilon * MathUtils.Epsilon)
            {
                // The origin is probably contained by a line segment
                // or triangle. Thus the shapes are overlapped.

                // We can't return zero here even though there may be overlap.
                // In case the simplex is a point, segment, or triangle it is difficult
                // to determine if the origin is contained in the CSO or very close to it.
                break;
            }

            // Compute a tentative new simplex vertex using support points.
            SimplexVertex vertex = simplex.V[simplex.Count];
            vertex.IndexA = input.ProxyA.GetSupport(-Complex.Divide(ref d, ref input.TransformA.Rotation));
            vertex.WA = Transform.Multiply(input.ProxyA.Vertices[vertex.IndexA], ref input.TransformA);

            vertex.IndexB = input.ProxyB.GetSupport(Complex.Divide(ref d, ref input.TransformB.Rotation));
            vertex.WB = Transform.Multiply(input.ProxyB.Vertices[vertex.IndexB], ref input.TransformB);
            vertex.W = vertex.WB - vertex.WA;
            simplex.V[simplex.Count] = vertex;

            // Iteration count is equated to the number of support point calls.
            ++iter;

            if (Settings.EnableDiagnostics) //FPE: We only gather diagnostics when enabled
                ++GJKIters;

            // Check for duplicate support points. This is the main termination criteria.
            bool duplicate = false;
            for (int i = 0; i < saveCount; ++i)
            {
                if (vertex.IndexA == saveA[i] && vertex.IndexB == saveB[i])
                {
                    duplicate = true;
                    break;
                }
            }

            // If we found a duplicate support point we must exit to avoid cycling.
            if (duplicate)
            {
                break;
            }

            // New vertex is ok and needed.
            ++simplex.Count;
        }

        if (Settings.EnableDiagnostics) //FPE: We only gather diagnostics when enabled
            GJKMaxIters = Math.Max(GJKMaxIters, iter);

        // Prepare output.
        simplex.GetWitnessPoints(out output.PointA, out output.PointB);
        output.Distance = (output.PointA - output.PointB).Length;
        output.Iterations = iter;

        // Cache the simplex.
        simplex.WriteCache(ref cache);

        // Apply radii if requested.
        if (input.UseRadii)
        {
            double rA = input.ProxyA.Radius;
            double rB = input.ProxyB.Radius;

            if (output.Distance > rA + rB && output.Distance > MathUtils.Epsilon)
            {
                // Shapes are still no overlapped.
                // Move the witness points to the outer surface.
                output.Distance -= rA + rB;
                Vector2 normal = (output.PointB - output.PointA).Normalized;
                output.PointA += rA * normal;
                output.PointB -= rB * normal;
            }
            else
            {
                // Shapes are overlapped when radii are considered.
                // Move the witness points to the middle.
                Vector2 p = 0.5 * (output.PointA + output.PointB);
                output.PointA = p;
                output.PointB = p;
                output.Distance = 0.0;
            }
        }
    }
}
