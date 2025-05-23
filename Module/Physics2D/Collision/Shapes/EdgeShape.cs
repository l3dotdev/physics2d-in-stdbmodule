using StdbModule.Common;

namespace StdbModule.Physics2D.Collision.Shapes;

/// <summary>
/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes.
/// The connectivity information is used to ensure correct contact normals.
/// </summary>
public class EdgeShape : Shape
{
    /// <summary>
    /// Edge start vertex
    /// </summary>
    internal Vector2 _vertex1;

    /// <summary>
    /// Edge end vertex
    /// </summary>
    internal Vector2 _vertex2;

    internal EdgeShape()
        : base(0)
    {
        ShapeType = ShapeType.Edge;
        _radius = Settings.PolygonRadius;
    }

    /// <summary>
    /// Create a new EdgeShape with the specified start and end.
    /// </summary>
    /// <param name="start">The start of the edge.</param>
    /// <param name="end">The end of the edge.</param>
    public EdgeShape(Vector2 start, Vector2 end)
        : base(0)
    {
        ShapeType = ShapeType.Edge;
        _radius = Settings.PolygonRadius;
        Set(start, end);
    }

    public override int ChildCount
    {
        get { return 1; }
    }

    /// <summary>
    /// Is true if the edge is connected to an adjacent vertex before vertex 1.
    /// </summary>
    public bool HasVertex0 { get; set; }

    /// <summary>
    /// Is true if the edge is connected to an adjacent vertex after vertex2.
    /// </summary>
    public bool HasVertex3 { get; set; }

    /// <summary>
    /// Optional adjacent vertices. These are used for smooth collision.
    /// </summary>
    public Vector2 Vertex0 { get; set; }

    /// <summary>
    /// Optional adjacent vertices. These are used for smooth collision.
    /// </summary>
    public Vector2 Vertex3 { get; set; }

    /// <summary>
    /// These are the edge vertices
    /// </summary>
    public Vector2 Vertex1
    {
        get { return _vertex1; }
        set
        {
            _vertex1 = value;
            ComputeProperties();
        }
    }

    /// <summary>
    /// These are the edge vertices
    /// </summary>
    public Vector2 Vertex2
    {
        get { return _vertex2; }
        set
        {
            _vertex2 = value;
            ComputeProperties();
        }
    }

    /// <summary>
    /// Set this as an isolated edge.
    /// </summary>
    /// <param name="start">The start.</param>
    /// <param name="end">The end.</param>
    public void Set(Vector2 start, Vector2 end)
    {
        _vertex1 = start;
        _vertex2 = end;
        HasVertex0 = false;
        HasVertex3 = false;

        ComputeProperties();
    }

    public override bool TestPoint(ref Transform transform, ref Vector2 point)
    {
        return false;
    }

    public override bool RayCast(out RayCastOutput output, ref RayCastInput input, ref Transform transform, int childIndex)
    {
        // p = p1 + t * d
        // v = v1 + s * e
        // p1 + t * d = v1 + s * e
        // s * e - t * d = p1 - v1

        output = new RayCastOutput();

        // Put the ray into the edge's frame of reference.
        Vector2 p1 = Complex.Divide(input.Point1 - transform.Position, ref transform.Rotation);
        Vector2 p2 = Complex.Divide(input.Point2 - transform.Position, ref transform.Rotation);
        Vector2 d = p2 - p1;

        Vector2 v1 = _vertex1;
        Vector2 v2 = _vertex2;
        Vector2 e = v2 - v1;
        Vector2 normal = new Vector2(e.Y, -e.X).Normalized; //TODO: Could possibly cache the normal.

        // q = p1 + t * d
        // dot(normal, q - v1) = 0
        // dot(normal, p1 - v1) + t * dot(normal, d) = 0
        double numerator = Vector2.Dot(normal, v1 - p1);
        double denominator = Vector2.Dot(normal, d);

        if (denominator == 0.0)
        {
            return false;
        }

        double t = numerator / denominator;
        if (t < 0.0 || input.MaxFraction < t)
        {
            return false;
        }

        Vector2 q = p1 + t * d;

        // q = v1 + s * r
        // s = dot(q - v1, r) / dot(r, r)
        Vector2 r = v2 - v1;
        double rr = Vector2.Dot(r, r);
        if (rr == 0.0)
        {
            return false;
        }

        double s = Vector2.Dot(q - v1, r) / rr;
        if (s < 0.0 || 1.0 < s)
        {
            return false;
        }

        output.Fraction = t;
        if (numerator > 0.0)
        {
            output.Normal = -normal;
        }
        else
        {
            output.Normal = normal;
        }
        return true;
    }

    public override void ComputeAABB(out AABB aabb, ref Transform transform, int childIndex)
    {
        // OPT: Vector2 v1 = Transform.Multiply(ref _vertex1, ref transform);            
        double v1X = _vertex1.X * transform.Rotation.Real - _vertex1.Y * transform.Rotation.i + transform.Position.X;
        double v1Y = _vertex1.Y * transform.Rotation.Real + _vertex1.X * transform.Rotation.i + transform.Position.Y;
        // OPT: Vector2 v2 = Transform.Multiply(ref _vertex2, ref transform);
        double v2X = _vertex2.X * transform.Rotation.Real - _vertex2.Y * transform.Rotation.i + transform.Position.X;
        double v2Y = _vertex2.Y * transform.Rotation.Real + _vertex2.X * transform.Rotation.i + transform.Position.Y;

        // OPT: aabb.LowerBound = Vector2.Min(v1, v2);
        // OPT: aabb.UpperBound = Vector2.Max(v1, v2);
        if (v1X < v2X)
        {
            aabb.LowerBound.X = v1X;
            aabb.UpperBound.X = v2X;
        }
        else
        {
            aabb.LowerBound.X = v2X;
            aabb.UpperBound.X = v1X;
        }
        if (v1Y < v2Y)
        {
            aabb.LowerBound.Y = v1Y;
            aabb.UpperBound.Y = v2Y;
        }
        else
        {
            aabb.LowerBound.Y = v2Y;
            aabb.UpperBound.Y = v1Y;
        }

        // OPT: Vector2 r = new Vector2(Radius, Radius);
        // OPT: aabb.LowerBound = aabb.LowerBound - r;
        // OPT: aabb.UpperBound = aabb.LowerBound + r;
        aabb.LowerBound.X -= Radius;
        aabb.LowerBound.Y -= Radius;
        aabb.UpperBound.X += Radius;
        aabb.UpperBound.Y += Radius;
    }

    protected override void ComputeProperties()
    {
        MassData.Centroid = 0.5 * (_vertex1 + _vertex2);
    }

    public override double ComputeSubmergedArea(ref Vector2 normal, double offset, ref Transform xf, out Vector2 sc)
    {
        sc = Vector2.Zero;
        return 0;
    }

    public bool CompareTo(EdgeShape shape)
    {
        return (HasVertex0 == shape.HasVertex0 &&
                HasVertex3 == shape.HasVertex3 &&
                Vertex0 == shape.Vertex0 &&
                Vertex1 == shape.Vertex1 &&
                Vertex2 == shape.Vertex2 &&
                Vertex3 == shape.Vertex3);
    }

    public override Shape Clone()
    {
        EdgeShape clone = new EdgeShape();
        clone.ShapeType = ShapeType;
        clone._radius = _radius;
        clone._density = _density;
        clone.HasVertex0 = HasVertex0;
        clone.HasVertex3 = HasVertex3;
        clone.Vertex0 = Vertex0;
        clone._vertex1 = _vertex1;
        clone._vertex2 = _vertex2;
        clone.Vertex3 = Vertex3;
        clone.MassData = MassData;
        return clone;
    }
}
