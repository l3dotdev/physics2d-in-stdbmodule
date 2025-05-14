
using StdbModule.Common;

namespace StdbModule.Physics2D;

public static class Settings
{
    /// <summary>
    /// Enabling diagnistics causes the engine to gather timing information.
    /// You can see how much time it took to solve the contacts, solve CCD
    /// and update the controllers.
    /// NOTE: If you are using a debug view that shows performance counters,
    /// you might want to enable this.
    /// </summary>
    public const bool EnableDiagnostics = true;

    /// <summary>
    /// The number of velocity iterations used in the solver.
    /// </summary>
    public const int VelocityIterations = 8;

    /// <summary>
    /// The number of position iterations used in the solver.
    /// </summary>
    public const int PositionIterations = 3;

    /// <summary>
    /// Enable/Disable Continuous Collision Detection (CCD)
    /// </summary>
    public const bool ContinuousPhysics = true;

    /// <summary>
    /// If true, it will run a GiftWrap convex hull on all polygon inputs.
    /// This makes for a more stable engine when given random input,
    /// but if speed of the creation of polygons are more important,
    /// you might want to set this to false.
    /// </summary>
    public const bool UseConvexHullPolygons = true;

    /// <summary>
    /// The number of velocity iterations in the TOI solver
    /// </summary>
    public const int TOIVelocityIterations = VelocityIterations;

    /// <summary>
    /// The number of position iterations in the TOI solver
    /// </summary>
    public const int TOIPositionIterations = 20;

    /// <summary>
    /// Maximum number of sub-steps per contact in continuous physics simulation.
    /// </summary>
    public const int MaxSubSteps = 8;

    /// <summary>
    /// Enable/Disable sleeping
    /// </summary>
    public const bool AllowSleep = true;

    /// <summary>
    /// The maximum number of vertices on a convex polygon.
    /// </summary>
    public const int MaxPolygonVertices = 8;

    /// <summary>
    /// The maximum number of contact points between two convex shapes.
    /// DO NOT CHANGE THIS VALUE!
    /// </summary>
    public const int MaxManifoldPoints = 2;

    /// <summary>
    /// This is used to fatten AABBs in the dynamic tree. This allows proxies
    /// to move by a small amount without triggering a tree adjustment.
    /// This is in meters.
    /// </summary>
    public const double AABBExtension = 0.1;

    /// <summary>
    /// This is used to fatten AABBs in the dynamic tree. This is used to predict
    /// the future position based on the current displacement.
    /// This is a dimensionless multiplier.
    /// </summary>
    public const double AABBMultiplier = 2.0;

    /// <summary>
    /// A small length used as a collision and constraint tolerance. Usually it is
    /// chosen to be numerically significant, but visually insignificant.
    /// </summary>
    public const double LinearSlop = 0.005;

    /// <summary>
    /// A small angle used as a collision and constraint tolerance. Usually it is
    /// chosen to be numerically significant, but visually insignificant.
    /// </summary>
    public const double AngularSlop = 2.0 / 180.0 * MathUtils.Pi;

    /// <summary>
    /// The radius of the polygon/edge shape skin. This should not be modified. Making
    /// this smaller means polygons will have an insufficient buffer for continuous collision.
    /// Making it larger may create artifacts for vertex collision.
    /// </summary>
    public const double PolygonRadius = 2.0 * LinearSlop;

    // Dynamics

    /// <summary>
    /// Maximum number of contacts to be handled to solve a TOI impact.
    /// </summary>
    public const int MaxTOIContacts = 32;

    /// <summary>
    /// A velocity threshold for elastic collisions. Any collision with a relative linear
    /// velocity below this threshold will be treated as inelastic.
    /// </summary>
    public const double VelocityThreshold = 1.0;

    /// <summary>
    /// The maximum linear position correction used when solving constraints. This helps to
    /// prevent overshoot.
    /// </summary>
    public const double MaxLinearCorrection = 0.2;

    /// <summary>
    /// The maximum angular position correction used when solving constraints. This helps to
    /// prevent overshoot.
    /// </summary>
    public const double MaxAngularCorrection = 8.0 / 180.0 * MathUtils.Pi;

    /// <summary>
    /// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
    /// that overlap is removed in one time step. However using values close to 1 often lead
    /// to overshoot.
    /// </summary>
    public const double Baumgarte = 0.2;

    // Sleep
    /// <summary>
    /// The time that a body must be still before it will go to sleep.
    /// </summary>
    public const double TimeToSleep = 0.5;

    /// <summary>
    /// A body cannot sleep if its linear velocity is above this tolerance.
    /// </summary>
    public const double LinearSleepTolerance = 0.01;

    /// <summary>
    /// A body cannot sleep if its angular velocity is above this tolerance.
    /// </summary>
    public const double AngularSleepTolerance = 2.0 / 180.0 * MathUtils.Pi;

    /// <summary>
    /// The maximum linear velocity of a body. This limit is very large and is used
    /// to prevent numerical problems. You shouldn't need to adjust this.
    /// </summary>
    public const double MaxTranslation = 2.0;

    public const double MaxTranslationSquared = MaxTranslation * MaxTranslation;

    /// <summary>
    /// The maximum angular velocity of a body. This limit is very large and is used
    /// to prevent numerical problems. You shouldn't need to adjust this.
    /// </summary>
    public const double MaxRotation = 0.5 * MathUtils.Pi;

    public const double MaxRotationSquared = MaxRotation * MaxRotation;

    /// <summary>
    /// Defines the maximum number of iterations made by the GJK algorithm.
    /// </summary>
    public const int MaxGJKIterations = 20;

    /// <summary>
    /// By default, forces are cleared automatically after each call to Step.
    /// The default behavior is modified with this setting.
    /// The purpose of this setting is to support sub-stepping. Sub-stepping is often used to maintain
    /// a fixed sized time step under a variable frame-rate.
    /// When you perform sub-stepping you should disable auto clearing of forces and instead call
    /// ClearForces after all sub-steps are complete in one pass of your game loop.
    /// </summary>
    public const bool AutoClearForces = true;

    /// <summary>
    /// Friction mixing law. Feel free to customize this.
    /// </summary>
    /// <param name="friction1">The friction1.</param>
    /// <param name="friction2">The friction2.</param>
    /// <returns></returns>
    public static double MixFriction(double friction1, double friction2)
    {
        return (double)Math.Sqrt(friction1 * friction2);
    }

    /// <summary>
    /// Restitution mixing law. Feel free to customize this.
    /// </summary>
    /// <param name="restitution1">The restitution1.</param>
    /// <param name="restitution2">The restitution2.</param>
    /// <returns></returns>
    public static double MixRestitution(double restitution1, double restitution2)
    {
        return restitution1 > restitution2 ? restitution1 : restitution2;
    }
}
