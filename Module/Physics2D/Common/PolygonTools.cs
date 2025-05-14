using System.Diagnostics;
using StdbModule.Common;

namespace StdbModule.Physics2D.Common;

public static class PolygonTools
{
    /// <summary>
    /// Build vertices to represent an axis-aligned box.
    /// </summary>
    /// <param name="hx">the half-width.</param>
    /// <param name="hy">the half-height.</param>
    public static Vertices CreateRectangle(double hx, double hy)
    {
        Vertices vertices =
        [
            new Vector2(-hx, -hy),
            new Vector2(hx, -hy),
            new Vector2(hx, hy),
            new Vector2(-hx, hy),
        ];

        return vertices;
    }

    /// <summary>
    /// Build vertices to represent an oriented box.
    /// </summary>
    /// <param name="hx">the half-width.</param>
    /// <param name="hy">the half-height.</param>
    /// <param name="center">the center of the box in local coordinates.</param>
    /// <param name="angle">the rotation of the box in local coordinates.</param>
    public static Vertices CreateRectangle(double hx, double hy, Vector2 center, double angle)
    {
        Vertices vertices = CreateRectangle(hx, hy);

        Transform xf = new(center, angle);

        // Transform vertices
        for (int i = 0; i < 4; ++i)
        {
            vertices[i] = Transform.Multiply(vertices[i], ref xf);
        }

        return vertices;
    }

    //Rounded rectangle contributed by Jonathan Smars - jsmars@gmail.com

    /// <summary>
    /// Creates a rounded rectangle with the specified width and height.
    /// </summary>
    /// <param name="width">The width.</param>
    /// <param name="height">The height.</param>
    /// <param name="xRadius">The rounding X radius.</param>
    /// <param name="yRadius">The rounding Y radius.</param>
    /// <param name="segments">The number of segments to subdivide the edges.</param>
    /// <returns></returns>
    public static Vertices CreateRoundedRectangle(double width, double height, double xRadius, double yRadius,
                                                    int segments)
    {
        if (yRadius > height / 2 || xRadius > width / 2)
            throw new Exception("Rounding amount can't be more than half the height and width respectively.");
        if (segments < 0)
            throw new Exception("Segments must be zero or more.");

        //We need at least 8 vertices to create a rounded rectangle
        Debug.Assert(Settings.MaxPolygonVertices >= 8);

        Vertices vertices = [];
        if (segments == 0)
        {
            vertices.Add(new Vector2(width * .5 - xRadius, -height * .5));
            vertices.Add(new Vector2(width * .5, -height * .5 + yRadius));

            vertices.Add(new Vector2(width * .5, height * .5 - yRadius));
            vertices.Add(new Vector2(width * .5 - xRadius, height * .5));

            vertices.Add(new Vector2(-width * .5 + xRadius, height * .5));
            vertices.Add(new Vector2(-width * .5, height * .5 - yRadius));

            vertices.Add(new Vector2(-width * .5, -height * .5 + yRadius));
            vertices.Add(new Vector2(-width * .5 + xRadius, -height * .5));
        }
        else
        {
            int numberOfEdges = segments * 4 + 8;

            double stepSize = MathUtils.Tau / (numberOfEdges - 4);
            int perPhase = numberOfEdges / 4;

            Vector2 posOffset = new Vector2(width / 2 - xRadius, height / 2 - yRadius);
            vertices.Add(posOffset + new Vector2(xRadius, -yRadius + yRadius));
            short phase = 0;
            for (int i = 1; i < numberOfEdges; i++)
            {
                if (i - perPhase == 0 || i - perPhase * 3 == 0)
                {
                    posOffset.X *= -1;
                    phase--;
                }
                else if (i - perPhase * 2 == 0)
                {
                    posOffset.Y *= -1;
                    phase--;
                }

                vertices.Add(posOffset + new Vector2(xRadius * Math.Cos(stepSize * -(i + phase)),
                                                        -yRadius * Math.Sin(stepSize * -(i + phase))));
            }
        }

        return vertices;
    }

    /// <summary>
    /// Set this as a single edge.
    /// </summary>
    /// <param name="start">The first point.</param>
    /// <param name="end">The second point.</param>
    public static Vertices CreateLine(Vector2 start, Vector2 end)
    {
        Vertices vertices = [start, end];

        return vertices;
    }

    /// <summary>
    /// Creates a circle with the specified radius and number of edges.
    /// </summary>
    /// <param name="radius">The radius.</param>
    /// <param name="numberOfEdges">The number of edges. The more edges, the more it resembles a circle</param>
    /// <returns></returns>
    public static Vertices CreateCircle(double radius, int numberOfEdges)
    {
        return CreateEllipse(radius, radius, numberOfEdges);
    }

    /// <summary>
    /// Creates a ellipse with the specified width, height and number of edges.
    /// </summary>
    /// <param name="xRadius">Width of the ellipse.</param>
    /// <param name="yRadius">Height of the ellipse.</param>
    /// <param name="numberOfEdges">The number of edges. The more edges, the more it resembles an ellipse</param>
    /// <returns></returns>
    public static Vertices CreateEllipse(double xRadius, double yRadius, int numberOfEdges)
    {
        Vertices vertices = [];

        double stepSize = MathUtils.Tau / numberOfEdges;

        vertices.Add(new Vector2(xRadius, 0));
        for (int i = numberOfEdges - 1; i > 0; --i)
            vertices.Add(new Vector2(xRadius * Math.Cos(stepSize * i),
                                        -yRadius * Math.Sin(stepSize * i)));

        return vertices;
    }

    public static Vertices CreateArc(double radians, int sides, double radius)
    {
        Debug.Assert(radians > 0, "The arc needs to be larger than 0");
        Debug.Assert(sides > 1, "The arc needs to have more than 1 sides");
        Debug.Assert(radius > 0, "The arc needs to have a radius larger than 0");

        Vertices vertices = [];

        double stepSize = radians / sides;
        for (int i = sides - 1; i > 0; i--)
        {
            vertices.Add(new Vector2(radius * Math.Cos(stepSize * i),
                                        radius * Math.Sin(stepSize * i)));
        }

        return vertices;
    }

    //Capsule contributed by Yobiv

    /// <summary>
    /// Creates an capsule with the specified height, radius and number of edges.
    /// A capsule has the same form as a pill capsule.
    /// </summary>
    /// <param name="height">Height (inner height + 2 * radius) of the capsule.</param>
    /// <param name="endRadius">Radius of the capsule ends.</param>
    /// <param name="edges">The number of edges of the capsule ends. The more edges, the more it resembles an capsule</param>
    /// <returns></returns>
    public static Vertices CreateCapsule(double height, double endRadius, int edges)
    {
        if (endRadius >= height / 2)
            throw new ArgumentException(
                "The radius must be lower than height / 2. Higher values of radius would create a circle, and not a half circle.",
                nameof(endRadius));

        return CreateCapsule(height, endRadius, edges, endRadius, edges);
    }

    /// <summary>
    /// Creates an capsule with the specified  height, radius and number of edges.
    /// A capsule has the same form as a pill capsule.
    /// </summary>
    /// <param name="height">Height (inner height + radii) of the capsule.</param>
    /// <param name="topRadius">Radius of the top.</param>
    /// <param name="topEdges">The number of edges of the top. The more edges, the more it resembles an capsule</param>
    /// <param name="bottomRadius">Radius of bottom.</param>
    /// <param name="bottomEdges">The number of edges of the bottom. The more edges, the more it resembles an capsule</param>
    /// <returns></returns>
    public static Vertices CreateCapsule(double height, double topRadius, int topEdges, double bottomRadius,
                                            int bottomEdges)
    {
        if (height <= 0)
            throw new ArgumentException("Height must be longer than 0", nameof(height));

        if (topRadius <= 0)
            throw new ArgumentException("The top radius must be more than 0", nameof(topRadius));

        if (topEdges <= 0)
            throw new ArgumentException("Top edges must be more than 0", nameof(topEdges));

        if (bottomRadius <= 0)
            throw new ArgumentException("The bottom radius must be more than 0", nameof(bottomRadius));

        if (bottomEdges <= 0)
            throw new ArgumentException("Bottom edges must be more than 0", nameof(bottomEdges));

        if (topRadius >= height / 2)
            throw new ArgumentException(
                "The top radius must be lower than height / 2. Higher values of top radius would create a circle, and not a half circle.",
                nameof(topRadius));

        if (bottomRadius >= height / 2)
            throw new ArgumentException(
                "The bottom radius must be lower than height / 2. Higher values of bottom radius would create a circle, and not a half circle.",
                nameof(bottomRadius));

        Vertices vertices = [];

        double newHeight = (height - topRadius - bottomRadius) * 0.5;

        // top
        vertices.Add(new Vector2(topRadius, newHeight));

        double stepSize = MathUtils.Pi / topEdges;
        for (int i = 1; i < topEdges; i++)
        {
            vertices.Add(new Vector2(topRadius * Math.Cos(stepSize * i),
                                        topRadius * Math.Sin(stepSize * i) + newHeight));
        }

        vertices.Add(new Vector2(-topRadius, newHeight));

        // bottom
        vertices.Add(new Vector2(-bottomRadius, -newHeight));

        stepSize = MathUtils.Pi / bottomEdges;
        for (int i = 1; i < bottomEdges; i++)
        {
            vertices.Add(new Vector2(-bottomRadius * Math.Cos(stepSize * i),
                                        -bottomRadius * Math.Sin(stepSize * i) - newHeight));
        }

        vertices.Add(new Vector2(bottomRadius, -newHeight));

        return vertices;
    }

    /// <summary>
    /// Creates a gear shape with the specified radius and number of teeth.
    /// </summary>
    /// <param name="radius">The radius.</param>
    /// <param name="numberOfTeeth">The number of teeth.</param>
    /// <param name="tipPercentage">The tip percentage.</param>
    /// <param name="toothHeight">Height of the tooth.</param>
    /// <returns></returns>
    public static Vertices CreateGear(double radius, int numberOfTeeth, double tipPercentage, double toothHeight)
    {
        Vertices vertices = new Vertices();

        double stepSize = MathUtils.Tau / numberOfTeeth;
        tipPercentage /= 100;
        MathUtils.Clamp(tipPercentage, 0, 1);
        double toothTipStepSize = stepSize / 2 * tipPercentage;

        double toothAngleStepSize = (stepSize - (toothTipStepSize * 2)) / 2;

        for (int i = numberOfTeeth - 1; i >= 0; --i)
        {
            if (toothTipStepSize > 0)
            {
                vertices.Add(
                    new Vector2(radius *
                                Math.Cos(stepSize * i + toothAngleStepSize * 2 + toothTipStepSize),
                                -radius *
                                Math.Sin(stepSize * i + toothAngleStepSize * 2 + toothTipStepSize)));

                vertices.Add(
                    new Vector2((radius + toothHeight) *
                                Math.Cos(stepSize * i + toothAngleStepSize + toothTipStepSize),
                                -(radius + toothHeight) *
                                Math.Sin(stepSize * i + toothAngleStepSize + toothTipStepSize)));
            }

            vertices.Add(new Vector2((radius + toothHeight) *
                                        Math.Cos(stepSize * i + toothAngleStepSize),
                                        -(radius + toothHeight) *
                                        Math.Sin(stepSize * i + toothAngleStepSize)));

            vertices.Add(new Vector2(radius * Math.Cos(stepSize * i),
                                        -radius * Math.Sin(stepSize * i)));
        }

        return vertices;
    }

}
