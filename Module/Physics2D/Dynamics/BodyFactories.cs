using StdbModule.Common;
using StdbModule.Physics2D.Collision.Shapes;
using StdbModule.Physics2D.Common;
using StdbModule.Physics2D.Common.Decomposition;

namespace StdbModule.Physics2D.Dynamics;

// An easy to use factory for creating bodies
public partial class Body
{
    /// <summary>
    /// Creates a fixture and attach it to this body.
    /// If the density is non-zero, this function automatically updates the mass of the body.
    /// Contacts are not created until the next time step.
    /// Warning: This method is locked during callbacks.
    /// </summary>
    /// <param name="shape">The shape.</param>
    /// <param name="userData">Application specific data</param>
    /// <returns></returns>
    public virtual Fixture CreateFixture(Shape shape)
    {
        Fixture fixture = new(shape);
        Add(fixture);
        return fixture;
    }

    public Fixture CreateEdge(Vector2 start, Vector2 end)
    {
        EdgeShape edgeShape = new(start, end);
        return CreateFixture(edgeShape);
    }

    public Fixture CreateChainShape(Vertices vertices)
    {
        ChainShape shape = new(vertices);
        return CreateFixture(shape);
    }

    public Fixture CreateLoopShape(Vertices vertices)
    {
        ChainShape shape = new(vertices, true);
        return CreateFixture(shape);
    }

    public Fixture CreateRectangle(double width, double height, double density, Vector2 offset)
    {
        Vertices rectangleVertices = PolygonTools.CreateRectangle(width / 2, height / 2);
        rectangleVertices.Translate(ref offset);
        PolygonShape rectangleShape = new(rectangleVertices, density);
        return CreateFixture(rectangleShape);
    }

    public Fixture CreateCircle(double radius, double density)
    {
        if (radius <= 0)
            throw new ArgumentOutOfRangeException(nameof(radius), "Radius must be more than 0 meters");

        CircleShape circleShape = new(radius, density);
        return CreateFixture(circleShape);
    }

    public Fixture CreateCircle(double radius, double density, Vector2 offset)
    {
        if (radius <= 0)
            throw new ArgumentOutOfRangeException(nameof(radius), "Radius must be more than 0 meters");

        CircleShape circleShape = new(radius, density);
        circleShape.Position = offset;
        return CreateFixture(circleShape);
    }

    public Fixture CreatePolygon(Vertices vertices, double density)
    {
        if (vertices.Count <= 1)
            throw new ArgumentOutOfRangeException(nameof(vertices), "Too few points to be a polygon");

        PolygonShape polygon = new(vertices, density);
        return CreateFixture(polygon);
    }

    public Fixture CreateEllipse(double xRadius, double yRadius, int edges, double density)
    {
        if (xRadius <= 0)
            throw new ArgumentOutOfRangeException(nameof(xRadius), "X-radius must be more than 0");

        if (yRadius <= 0)
            throw new ArgumentOutOfRangeException(nameof(yRadius), "Y-radius must be more than 0");

        Vertices ellipseVertices = PolygonTools.CreateEllipse(xRadius, yRadius, edges);
        PolygonShape polygonShape = new(ellipseVertices, density);
        return CreateFixture(polygonShape);
    }

    public List<Fixture> CreateCompoundPolygon(List<Vertices> list, double density)
    {
        List<Fixture> res = new(list.Count);

        //Then we create several fixtures using the body
        foreach (Vertices vertices in list)
        {
            if (vertices.Count == 2)
            {
                EdgeShape shape = new(vertices[0], vertices[1]);
                res.Add(CreateFixture(shape));
            }
            else
            {
                PolygonShape shape = new(vertices, density);
                res.Add(CreateFixture(shape));
            }
        }

        return res;
    }

    public Fixture CreateLineArc(double radians, int sides, double radius, bool closed)
    {
        Vertices arc = PolygonTools.CreateArc(radians, sides, radius);
        arc.Rotate((MathUtils.Pi - radians) / 2);
        return closed ? CreateLoopShape(arc) : CreateChainShape(arc);
    }

    public List<Fixture> CreateSolidArc(double density, double radians, int sides, double radius)
    {
        Vertices arc = PolygonTools.CreateArc(radians, sides, radius);
        arc.Rotate((MathUtils.Pi - radians) / 2);

        //Close the arc
        arc.Add(arc[0]);

        List<Vertices> triangles = Triangulate.ConvexPartition(arc, TriangulationAlgorithm.Earclip);

        return CreateCompoundPolygon(triangles, density);
    }
}