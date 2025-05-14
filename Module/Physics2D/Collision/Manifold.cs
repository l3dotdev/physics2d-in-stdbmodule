using StdbModule.Common;

namespace StdbModule.Physics2D.Collision;

/// <summary>
/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -ShapeType.Circles: the local center of circleB
/// -SeparationFunction.FaceA: the local center of cirlceB or the clip point of polygonB
/// -SeparationFunction.FaceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
/// </summary>
public struct ManifoldPoint
{
    /// <summary>
    /// Uniquely identifies a contact point between two Shapes
    /// </summary>
    public ContactID Id;

    /// <summary>
    /// Usage depends on manifold type
    /// </summary>
    public Vector2 LocalPoint;

    /// <summary>
    /// The non-penetration impulse
    /// </summary>
    public double NormalImpulse;

    /// <summary>
    /// The friction impulse
    /// </summary>
    public double TangentImpulse;
}

public enum ManifoldType
{
    Circles,
    FaceA,
    FaceB
}

/// <summary>
/// A manifold for two touching convex Shapes.
/// Box2D supports multiple types of contact:
/// - Clip point versus plane with radius
/// - Point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// - ShapeType.Circles: the local center of circleA
/// - SeparationFunction.FaceA: the center of faceA
/// - SeparationFunction.FaceB: the center of faceB
/// Similarly the local normal usage:
/// - ShapeType.Circles: not used
/// - SeparationFunction.FaceA: the normal on polygonA
/// - SeparationFunction.FaceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
/// </summary>
public struct Manifold
{
    /// <summary>
    /// Not use for Type.SeparationFunction.Points
    /// </summary>
    public Vector2 LocalNormal;

    /// <summary>
    /// Usage depends on manifold type
    /// </summary>
    public Vector2 LocalPoint;

    /// <summary>
    /// The number of manifold points
    /// </summary>
    public int PointCount;

    /// <summary>
    /// The points of contact
    /// </summary>
    public FixedArray2<ManifoldPoint> Points;

    public ManifoldType Type;
}