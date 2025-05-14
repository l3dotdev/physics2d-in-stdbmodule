using SpacetimeDB;

namespace StdbModule.Common;

[Type]
public partial struct Matrix3x3(Vector3 c1, Vector3 c2, Vector3 c3) : IEquatable<Matrix3x3>
{
    public static Matrix3x3 Identity => new(
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    );

    public Vector3 Ex = c1;
    public Vector3 Ey = c2;
    public Vector3 Ez = c3;

    public Matrix3x3(double x1, double x2, double x3, double y1, double y2, double y3, double z1, double z2, double z3) : this(new(x1, x2, x3), new(y1, y2, y3), new(z1, z2, z3)) { }

    public void Set(Vector3 c1, Vector3 c2, Vector3 c3)
    {
        Ex = c1;
        Ey = c2;
        Ez = c3;
    }

    /// <summary>
    /// Set this to the identity matrix.
    /// </summary>
    public void SetIdentity()
    {
        Ex.X = 1.0;
        Ey.X = 0.0;
        Ez.X = 0.0;
        Ex.Y = 0.0;
        Ey.Y = 1.0;
        Ez.Y = 0.0;
        Ex.Z = 0.0;
        Ey.Z = 0.0;
        Ez.Z = 1.0;
    }

    /// <summary>
    /// Set this matrix to all zeros.
    /// </summary>
    public void SetZero()
    {
        Ex.X = 0.0;
        Ey.X = 0.0;
        Ez.X = 0.0;
        Ex.Y = 0.0;
        Ey.Y = 0.0;
        Ey.Y = 0.0;
        Ex.Z = 0.0;
        Ey.Z = 0.0;
        Ez.Z = 0.0;
    }

    /// <summary>
    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases.
    /// </summary>
    /// <param name="b">The b.</param>
    /// <returns></returns>
    public readonly Vector3 Solve3x3(Vector3 b)
    {
        double det = Vector3.Dot(Ex, Vector3.Cross(Ey, Ez));
        if (det != 0.0)
        {
            det = 1.0 / det;
        }

        return new Vector3(det * Vector3.Dot(b, Vector3.Cross(Ey, Ez)), det * Vector3.Dot(Ex, Vector3.Cross(b, Ez)), det * Vector3.Dot(Ex, Vector3.Cross(Ey, b)));
    }

    /// <summary>
    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases. Solve only the upper
    /// 2-by-2 matrix equation.
    /// </summary>
    /// <param name="b">The b.</param>
    /// <returns></returns>
    public readonly Vector2 Solve2x2(Vector2 b)
    {
        double a11 = Ex.X, a12 = Ey.X, a21 = Ex.Y, a22 = Ey.Y;
        double det = a11 * a22 - a12 * a21;

        if (det != 0.0)
        {
            det = 1.0 / det;
        }

        return new Vector2(det * (a22 * b.X - a12 * b.Y), det * (a11 * b.Y - a21 * b.X));
    }

    /// Get the inverse of this matrix as a 2-by-2.
    /// Returns the zero matrix if singular.
    public readonly void GetInverse2x2(ref Matrix3x3 M)
    {
        double a = Ex.X, b = Ey.X, c = Ex.Y, d = Ey.Y;
        double det = a * d - b * c;
        if (det != 0.0)
        {
            det = 1.0 / det;
        }

        M.Ex.X = det * d; M.Ey.X = -det * b; M.Ex.Z = 0.0;
        M.Ex.Y = -det * c; M.Ey.Y = det * a; M.Ey.Z = 0.0;
        M.Ez.X = 0.0; M.Ez.Y = 0.0; M.Ez.Z = 0.0;
    }

    /// Get the symmetric inverse of this matrix as a 3-by-3.
    /// Returns the zero matrix if singular.
    public void GetSymInverse3x3(ref Matrix3x3 M)
    {
        double det = MathUtils.Dot(Ex, MathUtils.Cross(ref Ey, ref Ez));
        if (det != 0.0)
        {
            det = 1.0 / det;
        }

        double a11 = Ex.X, a12 = Ey.X, a13 = Ez.X;
        double a22 = Ey.Y, a23 = Ez.Y;
        double a33 = Ez.Z;

        M.Ex.X = det * (a22 * a33 - a23 * a23);
        M.Ex.Y = det * (a13 * a23 - a12 * a33);
        M.Ex.Z = det * (a12 * a23 - a13 * a22);

        M.Ey.X = M.Ex.Y;
        M.Ey.Y = det * (a11 * a33 - a13 * a13);
        M.Ey.Z = det * (a13 * a12 - a11 * a23);

        M.Ez.X = M.Ex.Z;
        M.Ez.Y = M.Ey.Z;
        M.Ez.Z = det * (a11 * a22 - a12 * a12);
    }
}