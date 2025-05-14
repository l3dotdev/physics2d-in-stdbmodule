using SpacetimeDB;
using Tomlet.Attributes;

namespace StdbModule.Common;

[Type]
public partial struct Vector2(double x, double y) : IEquatable<Vector2>
{
    public static Vector2 Zero => new(0, 0);
    public static Vector2 One => new(1, 1);
    public static Vector2 Up => new(0, -1);
    public static Vector2 Down => new(0, 1);
    public static Vector2 Left => new(-1, 0);
    public static Vector2 Right => new(1, 0);

    public static double Dot(Vector2 left, Vector2 right)
    {
        return left.X * right.X + left.Y * right.Y;
    }

    public static double DistanceSquared(Vector2 left, Vector2 right)
    {
        double dX = left.X - right.X;
        double dY = left.Y - right.Y;
        return dX * dX + dY * dY;
    }

    public static double Distance(Vector2 left, Vector2 right)
    {
        return Math.Sqrt(DistanceSquared(left, right));
    }

    public double X = x;
    public double Y = y;

    public Vector2(double xy) : this(xy, xy) { }

    public readonly double LengthSquared => X * X + Y * Y;
    public readonly double Length => Math.Sqrt(LengthSquared);

    public readonly Vector2 Normalized => Length == 0 ? Zero : this / Length;

    public readonly double Dot(Vector2 other) => Dot(this, other);
    public readonly double DistanceTo(Vector2 other) => Distance(this, other);
    public readonly double DistanceSquaredTo(Vector2 other) => DistanceSquared(this, other);

    public static Vector2 operator +(Vector2 left, Vector2 right) => new(left.X + right.X, left.Y + right.Y);
    public static Vector2 operator -(Vector2 left, Vector2 right) => new(left.X - right.X, left.Y - right.Y);
    public static Vector2 operator -(Vector2 right) => new(-right.X, -right.Y);
    public static Vector2 operator *(Vector2 left, Vector2 right) => new(left.X * right.X, left.Y * right.Y);
    public static Vector2 operator *(Vector2 left, double right) => new(left.X * right, left.Y * right);
    public static Vector2 operator /(Vector2 left, double right) => new(left.X / right, left.Y / right);
    public static Vector2 operator *(double left, Vector2 right) => new(right.X * left, right.Y * left);
    public static Vector2 operator /(double left, Vector2 right) => new(right.X / left, right.Y / left);

    #region Fast ref methods
    public static void Dot(ref Vector2 left, ref Vector2 right, out double result)
    {
        result = left.X * right.X + left.Y * right.Y;
    }

    public static void Min(ref Vector2 v1, ref Vector2 v2, out Vector2 result)
    {
        result.X = (v1.X < v2.X) ? v1.X : v2.X;
        result.Y = (v1.Y < v2.Y) ? v1.Y : v2.Y;
    }

    public static void Max(ref Vector2 v1, ref Vector2 v2, out Vector2 result)
    {
        result.X = (v1.X > v2.X) ? v1.X : v2.X;
        result.Y = (v1.Y > v2.Y) ? v1.Y : v2.Y;
    }

    public static void Distance(ref Vector2 v1, ref Vector2 v2, out double result)
    {
        double dx = v1.X - v2.X;
        double dy = v1.Y - v2.Y;
        result = Math.Sqrt(dx * dx + dy * dy);
    }

    public static void DistanceSquared(ref Vector2 v1, ref Vector2 v2, out double result)
    {
        double dx = v1.X - v2.X;
        double dy = v1.Y - v2.Y;
        result = (dx * dx) + (dy * dy);
    }

    public static void Add(ref Vector2 left, ref Vector2 right, out Vector2 result)
    {
        result.X = left.X + right.X;
        result.Y = left.Y + right.Y;
    }
    public static void Subtract(ref Vector2 left, ref Vector2 right, out Vector2 result)
    {
        result.X = left.X - right.X;
        result.Y = left.Y - right.Y;
    }

    public static void Multiply(ref Vector2 left, ref Vector2 right, out Vector2 result)
    {
        result.X = left.X * right.X;
        result.Y = left.Y * right.Y;
    }

    public static void Multiply(ref Vector2 left, double right, out Vector2 result)
    {
        result.X = left.X * right;
        result.Y = left.Y * right;
    }

    public static void Divide(ref Vector2 left, double right, out Vector2 result)
    {
        double invRight = 1 / right;
        result.X = left.X * invRight;
        result.Y = left.Y * invRight;
    }
    #endregion Fast ref methods
}

public class TomlVector2
{
    [TomlProperty("x")]
    public double X { get; set; }

    [TomlProperty("y")]
    public double Y { get; set; }
}
