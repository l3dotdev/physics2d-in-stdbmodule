namespace StdbModule.Common;

public static class MathUtils
{
    public const double Pi = Math.PI;
    public const double Tau = Math.PI * 2.0;
    public const double Epsilon = 1.192092896e-07;
    public const double MaxDouble = 3.402823466e+38;

    public static double Cross(ref Vector2 a, ref Vector2 b)
    {
        return a.X * b.Y - a.Y * b.X;
    }

    public static double Cross(Vector2 a, Vector2 b)
    {
        return Cross(ref a, ref b);
    }

    /// Perform the cross product on two vectors.
    public static Vector3 Cross(ref Vector3 a, ref Vector3 b)
    {
        return new Vector3(a.Y * b.Z - a.Z * b.Y,
                            a.Z * b.X - a.X * b.Z,
                            a.X * b.Y - a.Y * b.X);
    }

    public static Vector2 Cross(Vector2 a, double s)
    {
        return new Vector2(s * a.Y, -s * a.X);
    }

    public static Vector2 Rot270(ref Vector2 a)
    {
        return new Vector2(a.Y, -a.X);
    }

    public static Vector2 Cross(double s, ref Vector2 a)
    {
        return new Vector2(-s * a.Y, s * a.X);
    }

    public static Vector2 Rot90(ref Vector2 a)
    {
        return new Vector2(-a.Y, a.X);
    }

    public static Vector2 Abs(Vector2 v)
    {
        return new Vector2(Math.Abs(v.X), Math.Abs(v.Y));
    }

    public static Vector2 Mul(ref Matrix2x2 A, Vector2 v)
    {
        return Mul(ref A, ref v);
    }

    public static Vector2 Mul(ref Matrix2x2 A, ref Vector2 v)
    {
        return new Vector2(A.Ex.X * v.X + A.Ey.X * v.Y, A.Ex.Y * v.X + A.Ey.Y * v.Y);
    }

    public static Vector2 MulT(ref Matrix2x2 A, Vector2 v)
    {
        return MulT(ref A, ref v);
    }

    public static Vector2 MulT(ref Matrix2x2 A, ref Vector2 v)
    {
        return new Vector2(v.X * A.Ex.X + v.Y * A.Ex.Y, v.X * A.Ey.X + v.Y * A.Ey.Y);
    }


    // A^T * B
    public static void MulT(ref Matrix2x2 A, ref Matrix2x2 B, out Matrix2x2 C)
    {
        C.Ex.X = A.Ex.X * B.Ex.X + A.Ex.Y * B.Ex.Y;
        C.Ex.Y = A.Ey.X * B.Ex.X + A.Ey.Y * B.Ex.Y;
        C.Ey.X = A.Ex.X * B.Ey.X + A.Ex.Y * B.Ey.Y;
        C.Ey.Y = A.Ey.X * B.Ey.X + A.Ey.Y * B.Ey.Y;
    }

    /// Multiply a matrix times a vector.
    public static Vector3 Mul(Matrix3x3 A, Vector3 v)
    {
        return v.X * A.Ex + v.Y * A.Ey + v.Z * A.Ez;
    }

    public static void Swap<T>(ref T a, ref T b)
    {
        T tmp = a;
        a = b;
        b = tmp;
    }

    /// Multiply a matrix times a vector.
    public static Vector2 Mul22(Matrix3x3 A, Vector2 v)
    {
        return new Vector2(A.Ex.X * v.X + A.Ey.X * v.Y, A.Ex.Y * v.X + A.Ey.Y * v.Y);
    }

    /// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
    public static Vector2 Skew(Vector2 input)
    {
        return new Vector2(-input.Y, input.X);
    }

    /// <summary>
    /// This function is used to ensure that a floating point number is
    /// not a NaN or infinity.
    /// </summary>
    /// <param name="x">The x.</param>
    /// <returns>
    /// 	<c>true</c> if the specified x is valid; otherwise, <c>false</c>.
    /// </returns>
    public static bool IsValid(double x)
    {
        if (double.IsNaN(x))
        {
            // NaN.
            return false;
        }

        return !double.IsInfinity(x);
    }

    public static bool IsValid(this Vector2 x)
    {
        return IsValid(x.X) && IsValid(x.Y);
    }

    public static int Clamp(int a, int low, int high)
    {
        return Math.Max(low, Math.Min(a, high));
    }

    public static double Clamp(double a, double low, double high)
    {
        return Math.Max(low, Math.Min(a, high));
    }

    public static Vector2 Clamp(Vector2 a, Vector2 low, Vector2 high)
    {
        a.X = Math.Max(low.X, Math.Min(a.X, high.X));
        a.Y = Math.Max(low.Y, Math.Min(a.Y, high.Y));
        return a;
    }

    public static void Cross(ref Vector2 a, ref Vector2 b, out double c)
    {
        c = a.X * b.Y - a.Y * b.X;
    }

    /// <summary>
    /// Return the angle between two vectors on a plane
    /// The angle is from vector 1 to vector 2, positive anticlockwise
    /// The result is between -pi -> pi
    /// </summary>
    public static double VectorAngle(ref Vector2 p1, ref Vector2 p2)
    {
        double theta1 = Math.Atan2(p1.Y, p1.X);
        double theta2 = Math.Atan2(p2.Y, p2.X);
        double dtheta = theta2 - theta1;
        while (dtheta > Math.PI)
            dtheta -= (2 * Math.PI);
        while (dtheta < -Math.PI)
            dtheta += (2 * Math.PI);

        return (dtheta);
    }

    /// Perform the dot product on two vectors.
    public static double Dot(Vector3 a, Vector3 b)
    {
        return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
    }

    /// Perform the dot product on two vectors.
    public static double Dot(Vector2 a, ref Vector2 b)
    {
        return a.X * b.X + a.Y * b.Y;
    }

    public static double VectorAngle(Vector2 p1, Vector2 p2)
    {
        return VectorAngle(ref p1, ref p2);
    }

    /// <summary>
    /// Returns a positive number if c is to the left of the line going from a to b.
    /// </summary>
    /// <returns>Positive number if point is left, negative if point is right, 
    /// and 0 if points are collinear.</returns>
    public static double Area(Vector2 a, Vector2 b, Vector2 c)
    {
        return Area(ref a, ref b, ref c);
    }

    /// <summary>
    /// Returns a positive number if c is to the left of the line going from a to b.
    /// </summary>
    /// <returns>Positive number if point is left, negative if point is right, 
    /// and 0 if points are collinear.</returns>
    public static double Area(ref Vector2 a, ref Vector2 b, ref Vector2 c)
    {
        return a.X * (b.Y - c.Y) + b.X * (c.Y - a.Y) + c.X * (a.Y - b.Y);
    }

    /// <summary>
    /// Determines if three vertices are collinear (ie. on a straight line)
    /// </summary>
    /// <param name="a">First vertex</param>
    /// <param name="b">Second vertex</param>
    /// <param name="c">Third vertex</param>
    /// <param name="tolerance">The tolerance</param>
    /// <returns></returns>
    public static bool IsCollinear(ref Vector2 a, ref Vector2 b, ref Vector2 c, double tolerance = 0)
    {
        return DoubleInRange(Area(ref a, ref b, ref c), -tolerance, tolerance);
    }

    public static void Cross(double s, ref Vector2 a, out Vector2 b)
    {
        b.X = -s * a.Y;
        b.Y = s * a.X;
    }

    public static bool DoubleEquals(double value1, double value2)
    {
        return Math.Abs(value1 - value2) <= Epsilon;
    }

    /// <summary>
    /// Checks if a floating point Value is equal to another,
    /// within a certain tolerance.
    /// </summary>
    /// <param name="value1">The first floating point Value.</param>
    /// <param name="value2">The second floating point Value.</param>
    /// <param name="delta">The floating point tolerance.</param>
    /// <returns>True if the values are "equal", false otherwise.</returns>
    public static bool DoubleEquals(double value1, double value2, double delta)
    {
        return DoubleInRange(value1, value2 - delta, value2 + delta);
    }

    /// <summary>
    /// Checks if a floating point Value is within a specified
    /// range of values (inclusive).
    /// </summary>
    /// <param name="value">The Value to check.</param>
    /// <param name="min">The minimum Value.</param>
    /// <param name="max">The maximum Value.</param>
    /// <returns>True if the Value is within the range specified,
    /// false otherwise.</returns>
    public static bool DoubleInRange(double value, double min, double max)
    {
        return value >= min && value <= max;
    }

}