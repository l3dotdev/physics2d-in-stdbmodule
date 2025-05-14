using SpacetimeDB;

namespace StdbModule.Common;

[Type]
public partial struct Transform(Vector2 position, Complex rotation) : IEquatable<Transform>
{
    public static Transform Identity => new(Vector2.Zero, Complex.One);

    public Vector2 Position = position;
    public Complex Rotation = rotation;

    /// <summary>
    /// Initialize using a position vector and a rotation.
    /// </summary>
    /// <param name="position">The position.</param>
    /// <param name="angle">The rotation angle</param>
    public Transform(Vector2 position, double angle) : this(position, Complex.FromAngle(angle)) { }

    public static Vector2 Multiply(Vector2 left, ref Transform right)
    {
        return Multiply(ref left, ref right);
    }

    public static Vector2 Multiply(ref Vector2 left, ref Transform right)
    {
        // Opt: var result = Complex.Multiply(left, right.q) + right.p;
        return new Vector2(
            left.X * right.Rotation.Real - left.Y * right.Rotation.i + right.Position.X,
            left.Y * right.Rotation.Real + left.X * right.Rotation.i + right.Position.Y
        );
    }

    public static Vector2 Divide(Vector2 left, ref Transform right)
    {
        return Divide(ref left, ref right);
    }

    public static Vector2 Divide(ref Vector2 left, ref Transform right)
    {
        // Opt: var result = Complex.Divide(left - right.p, right);
        double px = left.X - right.Position.X;
        double py = left.Y - right.Position.Y;
        return new Vector2(
            px * right.Rotation.Real + py * right.Rotation.i,
            py * right.Rotation.Real - px * right.Rotation.i
        );
    }

    public static void Divide(Vector2 left, ref Transform right, out Vector2 result)
    {
        // Opt: var result = Complex.Divide(left - right.p, right);
        double px = left.X - right.Position.X;
        double py = left.Y - right.Position.Y;
        result.X = px * right.Rotation.Real + py * right.Rotation.i;
        result.Y = py * right.Rotation.Real - px * right.Rotation.i;
    }

    public static Transform Multiply(ref Transform left, ref Transform right)
    {
        return new Transform(
                Complex.Multiply(ref left.Position, ref right.Rotation) + right.Position,
                Complex.Multiply(ref left.Rotation, ref right.Rotation));
    }

    public static Transform Divide(ref Transform left, ref Transform right)
    {
        return new Transform(
            Complex.Divide(left.Position - right.Position, ref right.Rotation),
            Complex.Divide(ref left.Rotation, ref right.Rotation));
    }

    public static void Divide(ref Transform left, ref Transform right, out Transform result)
    {
        Complex.Divide(left.Position - right.Position, ref right.Rotation, out result.Position);
        Complex.Divide(ref left.Rotation, ref right.Rotation, out result.Rotation);
    }

    public static void Multiply(ref Transform left, Complex right, out Transform result)
    {
        result.Position = Complex.Multiply(ref left.Position, ref right);
        result.Rotation = Complex.Multiply(ref left.Rotation, ref right);
    }

    public static void Divide(ref Transform left, Complex right, out Transform result)
    {
        result.Position = Complex.Divide(ref left.Position, ref right);
        result.Rotation = Complex.Divide(ref left.Rotation, ref right);
    }
}
