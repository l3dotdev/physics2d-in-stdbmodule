using System.Diagnostics;

namespace StdbModule.Common;

public struct Sweep
{
    /// <summary>
    /// World angles
    /// </summary>
    public double A;
    public double A0;

    /// <summary>
    /// Fraction of the current time step in the range [0,1]
    /// c0 and a0 are the positions at alpha0.
    /// </summary>
    public double Alpha0;

    /// <summary>
    /// Center world positions
    /// </summary>
    public Vector2 C;
    public Vector2 C0;

    /// <summary>
    /// Local center of mass position
    /// </summary>
    public Vector2 LocalCenter;

    /// <summary>
    /// Get the interpolated transform at a specific time.
    /// </summary>
    /// <param name="xfb">The transform.</param>
    /// <param name="beta">beta is a factor in [0,1], where 0 indicates alpha0.</param>
    public void GetTransform(out Transform xfb, double beta)
    {
        xfb.Position.X = (1.0 - beta) * C0.X + beta * C.X;
        xfb.Position.Y = (1.0 - beta) * C0.Y + beta * C.Y;
        double angle = (1.0 - beta) * A0 + beta * A;
        xfb.Rotation = Complex.FromAngle(angle);

        // Shift to origin
        xfb.Position -= Complex.Multiply(ref LocalCenter, ref xfb.Rotation);
    }

    /// <summary>
    /// Advance the sweep forward, yielding a new initial state.
    /// </summary>
    /// <param name="alpha">new initial time..</param>
    public void Advance(double alpha)
    {
        Debug.Assert(Alpha0 < 1.0);
        double beta = (alpha - Alpha0) / (1.0 - Alpha0);
        C0 += beta * (C - C0);
        A0 += beta * (A - A0);
        Alpha0 = alpha;
    }

    /// <summary>
    /// Normalize the angles.
    /// </summary>
    public void Normalize()
    {
        double d = MathUtils.Tau * Math.Floor(A0 / MathUtils.Tau);
        A0 -= d;
        A -= d;
    }
}
