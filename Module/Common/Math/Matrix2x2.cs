using SpacetimeDB;

namespace StdbModule.Common;

[Type]
public partial struct Matrix2x2(Vector2 c1, Vector2 c2) : IEquatable<Matrix2x2>
{
    public static Matrix2x2 Identity => new(
        1, 0,
        0, 1
    );

    public Vector2 Ex = c1;
    public Vector2 Ey = c2;

    public Matrix2x2(double a11, double a12, double a21, double a22) : this(new(a11, a12), new(a21, a22)) { }

    public readonly Matrix2x2 Inverse
    {
        get
        {
            double a = Ex.X, b = Ey.X, c = Ex.Y, d = Ey.Y;
            double det = a * d - b * c;
            if (det != 0.0)
            {
                det = 1.0 / det;
            }

            return new(
                new(det * d, -det * c),
                new(-det * b, det * a)
            );
        }
    }

    public void Set(Vector2 c1, Vector2 c2)
    {
        Ex = c1;
        Ey = c2;
    }

    /// <summary>
    /// Set this to the identity matrix.
    /// </summary>
    public void SetIdentity()
    {
        Ex.X = 1.0;
        Ey.X = 0.0;
        Ex.Y = 0.0;
        Ey.Y = 1.0;
    }

    /// <summary>
    /// Set this matrix to all zeros.
    /// </summary>
    public void SetZero()
    {
        Ex.X = 0.0;
        Ey.X = 0.0;
        Ex.Y = 0.0;
        Ey.Y = 0.0;
    }

    /// <summary>
    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases.
    /// </summary>
    /// <param name="other">The b.</param>
    /// <returns></returns>
    public readonly Vector2 Solve(Vector2 other)
    {
        double a11 = Ex.X, a12 = Ey.X, a21 = Ex.Y, a22 = Ey.Y;
        double det = a11 * a22 - a12 * a21;
        if (det != 0.0)
        {
            det = 1.0 / det;
        }

        return new Vector2(det * (a22 * other.X - a12 * other.Y), det * (a11 * other.Y - a21 * other.X));
    }

    public static void Add(ref Matrix2x2 m1, ref Matrix2x2 m2, out Matrix2x2 result)
    {
        result.Ex = m1.Ex + m2.Ex;
        result.Ey = m1.Ey + m2.Ey;
    }
}