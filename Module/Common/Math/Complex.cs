using SpacetimeDB;

namespace StdbModule.Common;

[Type]
public partial struct Complex(double real, double imaginary) : IEquatable<Complex>
{
    public static Complex One => new(1, 0);
    public static Complex ImaginaryOne => new(0, 1);

    public double Real = real;
    public double i = imaginary;

    public double Phase
    {
        readonly get { return Math.Atan2(i, Real); }
        set
        {
            if (value == 0)
            {
                this = One;
                return;
            }
            Real = Math.Cos(value);
            i = Math.Sin(value);
        }
    }

    public readonly double Magnitude => Math.Sqrt(MagnitudeSquared());

    public static Complex FromAngle(double angle)
    {
        if (angle == 0)
            return One;

        return new Complex(
            Math.Cos(angle),
            Math.Sin(angle)
        );
    }

    public void Conjugate()
    {
        i = -i;
    }

    public void Negate()
    {
        Real = -Real;
        i = -i;
    }

    public readonly double MagnitudeSquared()
    {
        return (Real * Real) + (i * i);
    }

    public void Normalize()
    {
        var mag = Magnitude;
        Real /= mag;
        i /= mag;
    }

    public readonly Vector2 ToVector2()
    {
        return new Vector2(Real, i);
    }

    #region Fast ref methods
    public static Complex Multiply(ref Complex left, ref Complex right)
    {
        return new Complex(left.Real * right.Real - left.i * right.i,
                            left.i * right.Real + left.Real * right.i);
    }

    public static Complex Divide(ref Complex left, ref Complex right)
    {
        return new Complex(right.Real * left.Real + right.i * left.i,
                            right.Real * left.i - right.i * left.Real);
    }
    public static void Divide(ref Complex left, ref Complex right, out Complex result)
    {
        result = new Complex(right.Real * left.Real + right.i * left.i,
                                right.Real * left.i - right.i * left.Real);
    }

    public static Vector2 Multiply(ref Vector2 left, ref Complex right)
    {
        return new Vector2(left.X * right.Real - left.Y * right.i,
                            left.Y * right.Real + left.X * right.i);
    }
    public static void Multiply(ref Vector2 left, ref Complex right, out Vector2 result)
    {
        result = new Vector2(left.X * right.Real - left.Y * right.i,
                                left.Y * right.Real + left.X * right.i);
    }
    public static Vector2 Multiply(Vector2 left, ref Complex right)
    {
        return new Vector2(left.X * right.Real - left.Y * right.i,
                            left.Y * right.Real + left.X * right.i);
    }

    public static Vector2 Divide(ref Vector2 left, ref Complex right)
    {
        return new Vector2(left.X * right.Real + left.Y * right.i,
                            left.Y * right.Real - left.X * right.i);
    }

    public static Vector2 Divide(Vector2 left, ref Complex right)
    {
        return new Vector2(left.X * right.Real + left.Y * right.i,
                            left.Y * right.Real - left.X * right.i);
    }
    public static void Divide(Vector2 left, ref Complex right, out Vector2 result)
    {
        result = new Vector2(left.X * right.Real + left.Y * right.i,
                                left.Y * right.Real - left.X * right.i);
    }

    public static Complex Conjugate(ref Complex value)
    {
        return new Complex(value.Real, -value.i);
    }

    public static Complex Negate(ref Complex value)
    {
        return new Complex(-value.Real, -value.i);
    }

    public static Complex Normalize(ref Complex value)
    {
        var mag = value.Magnitude;
        return new Complex(value.Real / mag, -value.i / mag);
    }
    #endregion Fast ref methods
}
