using StdbModule.Common;
using StdbModule.Physics2D.Collision;
using StdbModule.Physics2D.Dynamics;

namespace StdbModule.Physics2D.Common.Logic;

/// <summary>
/// Creates a simple explosion that ignores other bodies hiding behind static bodies.
/// </summary>
public sealed class SimpleExplosion(World world) : PhysicsLogic(world)
{
    /// <summary>
    /// This is the power used in the power function. A value of 1 means the force
    /// applied to bodies in the explosion is linear. A value of 2 means it is exponential.
    /// </summary>
    public double Power { get; set; } = 1; //linear

    /// <summary>
    /// Activate the explosion at the specified position.
    /// </summary>
    /// <param name="pos">The position (center) of the explosion.</param>
    /// <param name="radius">The radius of the explosion.</param>
    /// <param name="force">The force applied</param>
    /// <param name="maxForce">A maximum amount of force. When force gets over this value, it will be equal to maxForce</param>
    /// <returns>A list of bodies and the amount of force that was applied to them.</returns>
    public Dictionary<Body, Vector2> Activate(Vector2 pos, double radius, double force, double maxForce = double.MaxValue)
    {
        HashSet<Body> affectedBodies = [];

        AABB aabb;
        aabb.LowerBound = pos - new Vector2(radius);
        aabb.UpperBound = pos + new Vector2(radius);

        // Query the world for bodies within the radius.
        World.QueryAABB(fixture =>
        {
            if (Vector2.Distance(fixture.Body.Position, pos) <= radius)
            {
                if (!affectedBodies.Contains(fixture.Body))
                    affectedBodies.Add(fixture.Body);
            }

            return true;
        }, ref aabb);

        return ApplyImpulse(pos, radius, force, maxForce, affectedBodies);
    }

    private Dictionary<Body, Vector2> ApplyImpulse(Vector2 pos, double radius, double force, double maxForce, HashSet<Body> overlappingBodies)
    {
        Dictionary<Body, Vector2> forces = new Dictionary<Body, Vector2>(overlappingBodies.Count);

        foreach (Body overlappingBody in overlappingBodies)
        {
            if (IsActiveOn(overlappingBody))
            {
                double distance = Vector2.Distance(pos, overlappingBody.Position);
                double forcePercent = GetPercent(distance, radius);

                Vector2 forceVector = pos - overlappingBody.Position;
                forceVector *= 1 / Math.Sqrt(forceVector.X * forceVector.X + forceVector.Y * forceVector.Y);
                forceVector *= Math.Min(force * forcePercent, maxForce);
                forceVector *= -1;

                overlappingBody.ApplyLinearImpulse(forceVector);
                forces.Add(overlappingBody, forceVector);
            }
        }

        return forces;
    }

    private double GetPercent(double distance, double radius)
    {
        //(1-(distance/radius))^power-1
        double percent = Math.Pow(1 - ((distance - radius) / radius), Power) - 1;

        if (double.IsNaN(percent))
            return 0;

        return MathUtils.Clamp(percent, 0, 1);
    }
}
