using SpacetimeDB;

namespace StdbModule.Common;

[Type]
public partial struct Vector3(double x, double y, double z) : IEquatable<Vector3>
{
    public static Vector3 Zero => new(0, 0, 0);
    public static Vector3 One => new(1, 1, 1);
    public static Vector3 Up => new(0, 1, 0);
    public static Vector3 Down => new(0, -1, 0);
    public static Vector3 Left => new(-1, 0, 0);
    public static Vector3 Right => new(1, 0, 0);
    public static Vector3 Forward => new(0, 0, -1);
    public static Vector3 Back => new(0, 0, 1);

    public static Vector3 Cross(Vector3 left, Vector3 right)
    {
        return new(
            left.Y * right.Z - left.Z * right.Y,
            left.Z * right.X - left.X * right.Z,
            left.X * right.Y - left.Y * right.X
        );

    }

    public static double Dot(Vector3 left, Vector3 right)
    {
        return left.X * right.X + left.Y * right.Y + left.Z * right.Z;
    }

    public static double DistanceSquared(Vector3 left, Vector3 right)
    {
        double dX = left.X - right.X;
        double dY = left.Y - right.Y;
        double dZ = left.Z - right.Z;
        return dX * dX + dY * dY + dZ * dZ;
    }

    public static double Distance(Vector3 left, Vector3 right)
    {
        return Math.Sqrt(DistanceSquared(left, right));
    }

    public double X = x;
    public double Y = y;
    public double Z = z;

    public Vector3(double xyz) : this(xyz, xyz, xyz) { }

    public readonly double LengthSquared => X * X + Y * Y + Z * Z;
    public readonly double Length => Math.Sqrt(LengthSquared);

    public readonly Vector3 Normalized => Length == 0 ? Zero : this / Length;

    public readonly double Dot(Vector3 other) => Dot(this, other);
    public readonly double DistanceTo(Vector3 other) => Distance(this, other);
    public readonly double DistanceSquaredTo(Vector3 other) => DistanceSquared(this, other);

    public static Vector3 operator +(Vector3 left, Vector3 right) => new(left.X + right.X, left.Y + right.Y, left.Z + right.Z);
    public static Vector3 operator -(Vector3 left, Vector3 right) => new(left.X - right.X, left.Y - right.Y, left.Z - right.Z);
    public static Vector3 operator -(Vector3 right) => new(-right.X, -right.Y, -right.Z);
    public static Vector3 operator *(Vector3 left, Vector3 right) => new(left.X * right.X, left.Y * right.Y, left.Z * right.Z);
    public static Vector3 operator *(Vector3 left, double right) => new(left.X * right, left.Y * right, left.Z * right);
    public static Vector3 operator /(Vector3 left, double right) => new(left.X / right, left.Y / right, left.Z / right);
    public static Vector3 operator *(double left, Vector3 right) => new(right.X * left, right.Y * left, right.Z * left);
    public static Vector3 operator /(double left, Vector3 right) => new(right.X / left, right.Y / left, right.Z / left);

    #region Fast ref methods
    public static void Cross(ref Vector3 left, ref Vector3 right, out Vector3 result)
    {
        result.X = left.Y * right.Z - left.Z * right.Y;
        result.Y = left.Z * right.X - left.X * right.Z;
        result.Z = left.X * right.Y - left.Y * right.X;
    }

    public static void Dot(ref Vector3 left, ref Vector3 right, out double result)
    {
        result = left.X * right.X + left.Y * right.Y + left.Z * right.Z;
    }

    public static void Min(ref Vector3 v1, ref Vector3 v2, out Vector3 result)
    {
        result.X = (v1.X < v2.X) ? v1.X : v2.X;
        result.Y = (v1.Y < v2.Y) ? v1.Y : v2.Y;
        result.Z = (v1.Z < v2.Z) ? v1.Z : v2.Z;
    }

    public static void Max(ref Vector3 v1, ref Vector3 v2, out Vector3 result)
    {
        result.X = (v1.X > v2.X) ? v1.X : v2.X;
        result.Y = (v1.Y > v2.Y) ? v1.Y : v2.Y;
        result.Z = (v1.Z > v2.Z) ? v1.Z : v2.Z;
    }

    public static void Distance(ref Vector3 v1, ref Vector3 v2, out double result)
    {
        double dx = v1.X - v2.X;
        double dy = v1.Y - v2.Y;
        double dz = v1.Z - v2.Z;
        result = Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    public static void DistanceSquared(ref Vector3 v1, ref Vector3 v2, out double result)
    {
        double dx = v1.X - v2.X;
        double dy = v1.Y - v2.Y;
        double dz = v1.Z - v2.Z;
        result = (dx * dx) + (dy * dy) + (dz * dz);
    }

    public static void Add(ref Vector3 left, ref Vector3 right, out Vector3 result)
    {
        result.X = left.X + right.X;
        result.Y = left.Y + right.Y;
        result.Z = left.Z + right.Z;
    }
    public static void Subtract(ref Vector3 left, ref Vector3 right, out Vector3 result)
    {
        result.X = left.X - right.X;
        result.Y = left.Y - right.Y;
        result.Z = left.Z - right.Z;
    }

    public static void Multiply(ref Vector3 left, ref Vector3 right, out Vector3 result)
    {
        result.X = left.X * right.X;
        result.Y = left.Y * right.Y;
        result.Z = left.Z * right.Z;
    }

    public static void Multiply(ref Vector3 left, double right, out Vector3 result)
    {
        result.X = left.X * right;
        result.Y = left.Y * right;
        result.Z = left.Z * right;
    }

    public static void Divide(ref Vector3 left, double right, out Vector3 result)
    {
        double invRight = 1 / right;
        result.X = left.X * invRight;
        result.Y = left.Y * invRight;
        result.Z = left.Z * invRight;
    }
    #endregion Fast ref methods
}