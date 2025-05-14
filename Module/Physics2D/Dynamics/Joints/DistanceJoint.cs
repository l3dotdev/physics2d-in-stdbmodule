using System.Diagnostics;
using StdbModule.Common;

namespace StdbModule.Physics2D.Dynamics.Joints;

// 1-D rained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k * 

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

/// <summary>
/// A distance joint rains two points on two bodies
/// to remain at a fixed distance from each other. You can view
/// this as a massless, rigid rod.
/// </summary>
public class DistanceJoint : Joint
{
    // Solver shared
    private double _bias;
    private double _gamma;
    private double _impulse;

    // Solver temp
    private int _indexA;
    private int _indexB;
    private Vector2 _u;
    private Vector2 _rA;
    private Vector2 _rB;
    private Vector2 _localCenterA;
    private Vector2 _localCenterB;
    private double _invMassA;
    private double _invMassB;
    private double _invIA;
    private double _invIB;
    private double _mass;

    internal DistanceJoint()
    {
        JointType = JointType.Distance;
    }

    /// <summary>
    /// This requires defining an
    /// anchor point on both bodies and the non-zero length of the
    /// distance joint. If you don't supply a length, the local anchor points
    /// is used so that the initial configuration can violate the constraint
    /// slightly. This helps when saving and loading a game.
    /// Warning Do not use a zero or short length.
    /// </summary>
    /// <param name="bodyA">The first body</param>
    /// <param name="bodyB">The second body</param>
    /// <param name="anchorA">The first body anchor</param>
    /// <param name="anchorB">The second body anchor</param>
    /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
    public DistanceJoint(Body bodyA, Body bodyB, Vector2 anchorA, Vector2 anchorB, bool useWorldCoordinates = false)
        : base(bodyA, bodyB)
    {
        JointType = JointType.Distance;

        if (useWorldCoordinates)
        {
            LocalAnchorA = bodyA.GetLocalPoint(ref anchorA);
            LocalAnchorB = bodyB.GetLocalPoint(ref anchorB);
            Length = (anchorB - anchorA).Length;
        }
        else
        {
            LocalAnchorA = anchorA;
            LocalAnchorB = anchorB;
            Length = (BodyB.GetWorldPoint(ref anchorB) - BodyA.GetWorldPoint(ref anchorA)).Length;
        }
    }

    /// <summary>
    /// The local anchor point relative to bodyA's origin.
    /// </summary>
    public Vector2 LocalAnchorA { get; set; }

    /// <summary>
    /// The local anchor point relative to bodyB's origin.
    /// </summary>
    public Vector2 LocalAnchorB { get; set; }

    public override sealed Vector2 WorldAnchorA
    {
        get { return BodyA.GetWorldPoint(LocalAnchorA); }
        set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
    }

    public override sealed Vector2 WorldAnchorB
    {
        get { return BodyB.GetWorldPoint(LocalAnchorB); }
        set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
    }

    /// <summary>
    /// The natural length between the anchor points.
    /// Manipulating the length can lead to non-physical behavior when the frequency is zero.
    /// </summary>
    public double Length { get; set; }

    /// <summary>
    /// The mass-spring-damper frequency in Hertz. A value of 0
    /// disables softness.
    /// </summary>
    public double Frequency { get; set; }

    /// <summary>
    /// The damping ratio. 0 = no damping, 1 = critical damping.
    /// </summary>
    public double DampingRatio { get; set; }

    /// <summary>
    /// Get the reaction force given the inverse time step. Unit is N.
    /// </summary>
    /// <param name="invDt"></param>
    /// <returns></returns>
    public override Vector2 GetReactionForce(double invDt)
    {
        Vector2 F = invDt * _impulse * _u;
        return F;
    }

    /// <summary>
    /// Get the reaction torque given the inverse time step.
    /// Unit is N*m. This is always zero for a distance joint.
    /// </summary>
    /// <param name="invDt"></param>
    /// <returns></returns>
    public override double GetReactionTorque(double invDt)
    {
        return 0.0;
    }

    internal override void InitVelocityConstraints(ref SolverData data)
    {
        _indexA = BodyA.IslandIndex;
        _indexB = BodyB.IslandIndex;
        _localCenterA = BodyA._sweep.LocalCenter;
        _localCenterB = BodyB._sweep.LocalCenter;
        _invMassA = BodyA._invMass;
        _invMassB = BodyB._invMass;
        _invIA = BodyA._invI;
        _invIB = BodyB._invI;

        Vector2 cA = data.positions[_indexA].c;
        double aA = data.positions[_indexA].a;
        Vector2 vA = data.velocities[_indexA].v;
        double wA = data.velocities[_indexA].w;

        Vector2 cB = data.positions[_indexB].c;
        double aB = data.positions[_indexB].a;
        Vector2 vB = data.velocities[_indexB].v;
        double wB = data.velocities[_indexB].w;

        Complex qA = Complex.FromAngle(aA);
        Complex qB = Complex.FromAngle(aB);

        _rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
        _rB = Complex.Multiply(LocalAnchorB - _localCenterB, ref qB);
        _u = cB + _rB - cA - _rA;

        // Handle singularity.
        double length = _u.Length;
        if (length > Settings.LinearSlop)
        {
            _u *= 1.0 / length;
        }
        else
        {
            _u = Vector2.Zero;
        }

        double crAu = MathUtils.Cross(ref _rA, ref _u);
        double crBu = MathUtils.Cross(ref _rB, ref _u);
        double invMass = _invMassA + _invIA * crAu * crAu + _invMassB + _invIB * crBu * crBu;

        // Compute the effective mass matrix.
        _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

        if (Frequency > 0.0)
        {
            double C = length - Length;

            // Frequency
            double omega = MathUtils.Tau * Frequency;

            // Damping coefficient
            double d = 2.0 * _mass * DampingRatio * omega;

            // Spring stiffness
            double k = _mass * omega * omega;

            // magic formulas
            double h = data.step.dt;
            _gamma = h * (d + h * k);
            _gamma = _gamma != 0.0 ? 1.0 / _gamma : 0.0;
            _bias = C * h * k * _gamma;

            invMass += _gamma;
            _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;
        }
        else
        {
            _gamma = 0.0;
            _bias = 0.0;
        }

        if (data.step.warmStarting)
        {
            // Scale the impulse to support a variable time step.
            _impulse *= data.step.dtRatio;

            Vector2 P = _impulse * _u;
            vA -= _invMassA * P;
            wA -= _invIA * MathUtils.Cross(ref _rA, ref P);
            vB += _invMassB * P;
            wB += _invIB * MathUtils.Cross(ref _rB, ref P);
        }
        else
        {
            _impulse = 0.0;
        }

        data.velocities[_indexA].v = vA;
        data.velocities[_indexA].w = wA;
        data.velocities[_indexB].v = vB;
        data.velocities[_indexB].w = wB;
    }

    internal override void SolveVelocityConstraints(ref SolverData data)
    {
        Vector2 vA = data.velocities[_indexA].v;
        double wA = data.velocities[_indexA].w;
        Vector2 vB = data.velocities[_indexB].v;
        double wB = data.velocities[_indexB].w;

        // Cdot = dot(u, v + cross(w, r))
        Vector2 vpA = vA + MathUtils.Cross(wA, ref _rA);
        Vector2 vpB = vB + MathUtils.Cross(wB, ref _rB);
        double Cdot = Vector2.Dot(_u, vpB - vpA);

        double impulse = -_mass * (Cdot + _bias + _gamma * _impulse);
        _impulse += impulse;

        Vector2 P = impulse * _u;
        vA -= _invMassA * P;
        wA -= _invIA * MathUtils.Cross(ref _rA, ref P);
        vB += _invMassB * P;
        wB += _invIB * MathUtils.Cross(ref _rB, ref P);

        data.velocities[_indexA].v = vA;
        data.velocities[_indexA].w = wA;
        data.velocities[_indexB].v = vB;
        data.velocities[_indexB].w = wB;

    }

    internal override bool SolvePositionConstraints(ref SolverData data)
    {
        if (Frequency > 0.0)
        {
            // There is no position correction for soft distance constraints.
            return true;
        }

        Vector2 cA = data.positions[_indexA].c;
        double aA = data.positions[_indexA].a;
        Vector2 cB = data.positions[_indexB].c;
        double aB = data.positions[_indexB].a;

        Complex qA = Complex.FromAngle(aA);
        Complex qB = Complex.FromAngle(aB);

        Vector2 rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
        Vector2 rB = Complex.Multiply(LocalAnchorB - _localCenterB, ref qB);
        Vector2 u = cB + rB - cA - rA;

        double length = u.Length;
        u = u.Normalized;
        double C = length - Length;
        C = MathUtils.Clamp(C, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);

        double impulse = -_mass * C;
        Vector2 P = impulse * u;

        cA -= _invMassA * P;
        aA -= _invIA * MathUtils.Cross(ref rA, ref P);
        cB += _invMassB * P;
        aB += _invIB * MathUtils.Cross(ref rB, ref P);

        data.positions[_indexA].c = cA;
        data.positions[_indexA].a = aA;
        data.positions[_indexB].c = cB;
        data.positions[_indexB].a = aB;

        return Math.Abs(C) < Settings.LinearSlop;
    }
}
