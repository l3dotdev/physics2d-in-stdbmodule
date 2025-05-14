using System.Diagnostics;
using StdbModule.Common;

namespace StdbModule.Physics2D.Dynamics.Joints;

/// <summary>
/// Maintains a fixed angle between two bodies
/// </summary>
public class AngleJoint : Joint
{
    private double _bias;
    private double _jointError;
    private double _massFactor;
    private double _targetAngle;

    internal AngleJoint()
    {
        JointType = JointType.Angle;
    }

    /// <summary>
    /// Constructor for AngleJoint
    /// </summary>
    /// <param name="bodyA">The first body</param>
    /// <param name="bodyB">The second body</param>
    public AngleJoint(Body bodyA, Body bodyB)
        : base(bodyA, bodyB)
    {
        JointType = JointType.Angle;
        BiasFactor = .2;
        MaxImpulse = double.MaxValue;
    }

    public override Vector2 WorldAnchorA
    {
        get { return BodyA.Position; }
        set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
    }

    public override Vector2 WorldAnchorB
    {
        get { return BodyB.Position; }
        set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
    }

    /// <summary>
    /// The desired angle between BodyA and BodyB
    /// </summary>
    public double TargetAngle
    {
        get { return _targetAngle; }
        set
        {
            if (value != _targetAngle)
            {
                _targetAngle = value;
                WakeBodies();
            }
        }
    }

    /// <summary>
    /// Gets or sets the bias factor.
    /// Defaults to 0.2
    /// </summary>
    public double BiasFactor { get; set; }

    /// <summary>
    /// Gets or sets the maximum impulse
    /// Defaults to double.MaxValue
    /// </summary>
    public double MaxImpulse { get; set; }

    /// <summary>
    /// Gets or sets the softness of the joint
    /// Defaults to 0
    /// </summary>
    public double Softness { get; set; }

    public override Vector2 GetReactionForce(double invDt)
    {
        //TODO
        //return _inv_dt * _impulse;
        return Vector2.Zero;
    }

    public override double GetReactionTorque(double invDt)
    {
        return 0;
    }

    internal override void InitVelocityConstraints(ref SolverData data)
    {
        int indexA = BodyA.IslandIndex;
        int indexB = BodyB.IslandIndex;

        double aW = data.positions[indexA].a;
        double bW = data.positions[indexB].a;

        _jointError = bW - aW - TargetAngle;
        _bias = -BiasFactor * data.step.inv_dt * _jointError;
        _massFactor = (1 - Softness) / (BodyA._invI + BodyB._invI);
    }

    internal override void SolveVelocityConstraints(ref SolverData data)
    {
        int indexA = BodyA.IslandIndex;
        int indexB = BodyB.IslandIndex;

        double p = (_bias - data.velocities[indexB].w + data.velocities[indexA].w) * _massFactor;

        data.velocities[indexA].w -= BodyA._invI * Math.Sign(p) * Math.Min(Math.Abs(p), MaxImpulse);
        data.velocities[indexB].w += BodyB._invI * Math.Sign(p) * Math.Min(Math.Abs(p), MaxImpulse);
    }

    internal override bool SolvePositionConstraints(ref SolverData data)
    {
        //no position solving for this joint
        return true;
    }
}
