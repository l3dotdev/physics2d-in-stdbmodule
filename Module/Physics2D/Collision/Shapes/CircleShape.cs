using System.Diagnostics;
using StdbModule.Common;

namespace StdbModule.Physics2D.Collision.Shapes;

/// <summary>
/// A circle shape.
/// </summary>
public class CircleShape : Shape
{
    internal Vector2 _position;

    /// <summary>
    /// Create a new circle with the desired radius and density.
    /// </summary>
    /// <param name="radius">The radius of the circle.</param>
    /// <param name="density">The density of the circle.</param>
    public CircleShape(double radius, double density) : base(density)
    {
        Debug.Assert(radius >= 0);
        Debug.Assert(density >= 0);

        ShapeType = ShapeType.Circle;
        _position = Vector2.Zero;
        Radius = radius; // The Radius property cache 2radius and calls ComputeProperties(). So no need to call ComputeProperties() here.
    }

    internal CircleShape() : base(0)
    {
        ShapeType = ShapeType.Circle;
        _radius = 0.0;
        _position = Vector2.Zero;
    }

    public override int ChildCount => 1;

    /// <summary>
    /// Get or set the position of the circle
    /// </summary>
    public Vector2 Position
    {
        get { return _position; }
        set
        {
            _position = value;
            ComputeProperties(); //TODO: Optimize here
        }
    }

    public override bool TestPoint(ref Transform transform, ref Vector2 point)
    {
        Vector2 center = transform.Position + Complex.Multiply(ref _position, ref transform.Rotation);
        Vector2 d = point - center;
        return Vector2.Dot(d, d) <= _2radius;
    }

    public override bool RayCast(out RayCastOutput output, ref RayCastInput input, ref Transform transform, int childIndex)
    {
        // Collision Detection in Interactive 3D Environments by Gino van den Bergen
        // From Section 3.1.2
        // x = s + a * r
        // norm(x) = radius

        output = new RayCastOutput();

        Vector2 position = transform.Position + Complex.Multiply(ref _position, ref transform.Rotation);
        Vector2 s = input.Point1 - position;
        double b = Vector2.Dot(s, s) - _2radius;

        // Solve quadratic equation.
        Vector2 r = input.Point2 - input.Point1;
        double c = Vector2.Dot(s, r);
        double rr = Vector2.Dot(r, r);
        double sigma = c * c - rr * b;

        // Check for negative discriminant and short segment.
        if (sigma < 0.0 || rr < MathUtils.Epsilon)
        {
            return false;
        }

        // Find the point of intersection of the line with the circle.
        double a = -(c + Math.Sqrt(sigma));

        // Is the intersection point on the segment?
        if (0.0 <= a && a <= input.MaxFraction * rr)
        {
            a /= rr;
            output.Fraction = a;

            //TODO: Check results here
            output.Normal = (s + a * r).Normalized;
            return true;
        }

        return false;
    }

    public override void ComputeAABB(out AABB aabb, ref Transform transform, int childIndex)
    {
        // OPT: Vector2 p = transform.p + Complex.Multiply(ref _position, ref transform.q);
        var pX = _position.X * transform.Rotation.Real - _position.Y * transform.Rotation.i + transform.Position.X;
        var pY = _position.Y * transform.Rotation.Real + _position.X * transform.Rotation.i + transform.Position.Y;

        // OPT: aabb.LowerBound = new Vector2(p.X - Radius, p.Y - Radius);
        // OPT: aabb.UpperBound = new Vector2(p.X + Radius, p.Y + Radius);
        aabb.LowerBound.X = pX - Radius;
        aabb.LowerBound.Y = pY - Radius;
        aabb.UpperBound.X = pX + Radius;
        aabb.UpperBound.Y = pY + Radius;
    }

    protected override sealed void ComputeProperties()
    {
        double area = MathUtils.Pi * _2radius;
        MassData.Area = area;
        MassData.Mass = Density * area;
        MassData.Centroid = Position;

        // inertia about the local origin
        MassData.Inertia = MassData.Mass * (0.5 * _2radius + Vector2.Dot(Position, Position));
    }

    public override double ComputeSubmergedArea(ref Vector2 normal, double offset, ref Transform xf, out Vector2 sc)
    {
        sc = Vector2.Zero;

        Vector2 p = Transform.Multiply(ref _position, ref xf);
        double l = -(Vector2.Dot(normal, p) - offset);
        if (l < -Radius + MathUtils.Epsilon)
        {
            //Completely dry
            return 0;
        }
        if (l > Radius)
        {
            //Completely wet
            sc = p;
            return MathUtils.Pi * _2radius;
        }

        //Magic
        double l2 = l * l;
        double area = _2radius * (Math.Asin(l / Radius) + MathUtils.Pi / 2 + l * Math.Sqrt(_2radius - l2));
        double com = -2.0 / 3.0 * Math.Pow(_2radius - l2, 1.5) / area;

        sc.X = p.X + normal.X * com;
        sc.Y = p.Y + normal.Y * com;

        return area;
    }

    /// <summary>
    /// Compare the circle to another circle
    /// </summary>
    /// <param name="shape">The other circle</param>
    /// <returns>True if the two circles are the same size and have the same position</returns>
    public bool CompareTo(CircleShape shape)
    {
        return Radius == shape.Radius && Position == shape.Position;
    }

    public override Shape Clone()
    {
        CircleShape clone = new()
        {
            ShapeType = ShapeType,
            _radius = Radius,
            _2radius = _2radius, //FPE note: We also copy the cache
            _density = _density,
            _position = _position,
            MassData = MassData
        };
        return clone;
    }
}
