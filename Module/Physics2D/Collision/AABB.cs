using StdbModule.Common;
using StdbModule.Physics2D.Common;

namespace StdbModule.Physics2D.Collision;

/// <summary>
/// An axis aligned bounding box.
/// </summary>
public struct AABB
{
    /// <summary>
    /// The lower vertex
    /// </summary>
    public Vector2 LowerBound;

    /// <summary>
    /// The upper vertex
    /// </summary>
    public Vector2 UpperBound;

    public AABB(Vector2 min, Vector2 max)
        : this(ref min, ref max)
    {
    }

    public AABB(ref Vector2 min, ref Vector2 max)
    {
        LowerBound = min;
        UpperBound = max;
    }

    public AABB(Vector2 center, double width, double height)
    {
        LowerBound = center - new Vector2(width / 2, height / 2);
        UpperBound = center + new Vector2(width / 2, height / 2);
    }

    public readonly double Width => UpperBound.X - LowerBound.X;
    public readonly double Height => UpperBound.Y - LowerBound.Y;

    /// <summary>
    /// Get the center of the AABB.
    /// </summary>
    public readonly Vector2 Center => 0.5 * (LowerBound + UpperBound);

    /// <summary>
    /// Get the extents of the AABB (half-widths).
    /// </summary>
    public readonly Vector2 Extents => 0.5 * (UpperBound - LowerBound);

    /// <summary>
    /// Get the perimeter length
    /// </summary>
    public readonly double Perimeter
    {
        get
        {
            double wx = UpperBound.X - LowerBound.X;
            double wy = UpperBound.Y - LowerBound.Y;
            return 2.0 * (wx + wy);
        }
    }

    /// <summary>
    /// Gets the vertices of the AABB.
    /// </summary>
    /// <value>The corners of the AABB</value>
    public readonly Vertices Vertices => [
        UpperBound,
        new Vector2(UpperBound.X, LowerBound.Y),
        LowerBound,
        new Vector2(LowerBound.X, UpperBound.Y),
    ];

    /// <summary>
    /// First quadrant
    /// </summary>
    public readonly AABB Q1 => new(Center, UpperBound);

    /// <summary>
    /// Second quadrant
    /// </summary>
    public readonly AABB Q2 => new(new Vector2(LowerBound.X, Center.Y), new Vector2(Center.X, UpperBound.Y));

    /// <summary>
    /// Third quadrant
    /// </summary>
    public readonly AABB Q3 => new(LowerBound, Center);

    /// <summary>
    /// Forth quadrant
    /// </summary>
    public readonly AABB Q4 => new(new Vector2(Center.X, LowerBound.Y), new Vector2(UpperBound.X, Center.Y));

    /// <summary>
    /// Verify that the bounds are sorted. And the bounds are valid numbers (not NaN).
    /// </summary>
    /// <returns>
    /// 	<c>true</c> if this instance is valid; otherwise, <c>false</c>.
    /// </returns>
    public readonly bool IsValid()
    {
        Vector2 d = UpperBound - LowerBound;
        bool valid = d.X >= 0.0 && d.Y >= 0.0;
        valid = valid && LowerBound.IsValid() && UpperBound.IsValid();
        return valid;
    }

    /// <summary>
    /// Combine an AABB into this one.
    /// </summary>
    /// <param name="aabb">The aabb.</param>
    public void Combine(ref AABB aabb)
    {
        Vector2.Min(ref LowerBound, ref aabb.LowerBound, out LowerBound);
        Vector2.Max(ref UpperBound, ref aabb.UpperBound, out UpperBound);
    }

    /// <summary>
    /// Combine two AABBs into this one.
    /// </summary>
    /// <param name="aabb1">The aabb1.</param>
    /// <param name="aabb2">The aabb2.</param>
    public void Combine(ref AABB aabb1, ref AABB aabb2)
    {
        Vector2.Min(ref aabb1.LowerBound, ref aabb2.LowerBound, out LowerBound);
        Vector2.Max(ref aabb1.UpperBound, ref aabb2.UpperBound, out UpperBound);
    }

    /// <summary>
    /// Does this aabb contain the provided AABB.
    /// </summary>
    /// <param name="aabb">The aabb.</param>
    /// <returns>
    /// 	<c>true</c> if it contains the specified aabb; otherwise, <c>false</c>.
    /// </returns>
    public readonly bool Contains(ref AABB aabb)
    {
        bool result = true;
        result = result && LowerBound.X <= aabb.LowerBound.X;
        result = result && LowerBound.Y <= aabb.LowerBound.Y;
        result = result && aabb.UpperBound.X <= UpperBound.X;
        result = result && aabb.UpperBound.Y <= UpperBound.Y;
        return result;
    }

    /// <summary>
    /// Determines whether the AAABB contains the specified point.
    /// </summary>
    /// <param name="point">The point.</param>
    /// <returns>
    /// 	<c>true</c> if it contains the specified point; otherwise, <c>false</c>.
    /// </returns>
    public readonly bool Contains(ref Vector2 point)
    {
        //using epsilon to try and gaurd against float rounding errors.
        return point.X > (LowerBound.X + MathUtils.Epsilon) && point.X < (UpperBound.X - MathUtils.Epsilon) &&
                point.Y > (LowerBound.Y + MathUtils.Epsilon) && point.Y < (UpperBound.Y - MathUtils.Epsilon);
    }

    /// <summary>
    /// Test if the two AABBs overlap.
    /// </summary>
    /// <param name="a">The first AABB.</param>
    /// <param name="b">The second AABB.</param>
    /// <returns>True if they are overlapping.</returns>
    public static bool TestOverlap(ref AABB a, ref AABB b)
    {
        if (b.LowerBound.X > a.UpperBound.X || b.LowerBound.Y > a.UpperBound.Y)
            return false;

        if (a.LowerBound.X > b.UpperBound.X || a.LowerBound.Y > b.UpperBound.Y)
            return false;

        return true;
    }

    /// <summary>
    /// Raycast against this AABB using the specificed points and maxfraction (found in input)
    /// </summary>
    /// <param name="output">The results of the raycast.</param>
    /// <param name="input">The parameters for the raycast.</param>
    /// <returns>True if the ray intersects the AABB</returns>
    public bool RayCast(out RayCastOutput output, ref RayCastInput input, bool doInteriorCheck = true)
    {
        // From Real-time Collision Detection, p179.

        output = new RayCastOutput();

        double tmin = -MathUtils.MaxDouble;
        double tmax = MathUtils.MaxDouble;

        Vector2 p = input.Point1;
        Vector2 d = input.Point2 - input.Point1;
        Vector2 absD = MathUtils.Abs(d);

        Vector2 normal = Vector2.Zero;

        for (int i = 0; i < 2; ++i)
        {
            double absD_i = i == 0 ? absD.X : absD.Y;
            double lowerBound_i = i == 0 ? LowerBound.X : LowerBound.Y;
            double upperBound_i = i == 0 ? UpperBound.X : UpperBound.Y;
            double p_i = i == 0 ? p.X : p.Y;

            if (absD_i < MathUtils.Epsilon)
            {
                // Parallel.
                if (p_i < lowerBound_i || upperBound_i < p_i)
                {
                    return false;
                }
            }
            else
            {
                double d_i = i == 0 ? d.X : d.Y;

                double inv_d = 1.0 / d_i;
                double t1 = (lowerBound_i - p_i) * inv_d;
                double t2 = (upperBound_i - p_i) * inv_d;

                // Sign of the normal vector.
                double s = -1.0;

                if (t1 > t2)
                {
                    MathUtils.Swap(ref t1, ref t2);
                    s = 1.0;
                }

                // Push the min up
                if (t1 > tmin)
                {
                    if (i == 0)
                    {
                        normal.X = s;
                    }
                    else
                    {
                        normal.Y = s;
                    }

                    tmin = t1;
                }

                // Pull the max down
                tmax = Math.Min(tmax, t2);

                if (tmin > tmax)
                {
                    return false;
                }
            }
        }

        // Does the ray start inside the box?
        // Does the ray intersect beyond the max fraction?
        if (doInteriorCheck && (tmin < 0.0 || input.MaxFraction < tmin))
        {
            return false;
        }

        // Intersection.
        output.Fraction = tmin;
        output.Normal = normal;
        return true;
    }
}
