using System.Diagnostics;
using StdbModule.Common;

namespace StdbModule.Physics2D.Dynamics.Joints;

// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C0 = (length1 + ratio * length2)_initial
// C = C0 - (length1 + ratio * length2)
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

/// <summary>
/// The pulley joint is connected to two bodies and two fixed world points.
/// The pulley supports a ratio such that:
/// <![CDATA[length1 + ratio * length2 <= constant]]>
/// Yes, the force transmitted is scaled by the ratio.
/// 
/// Warning: the pulley joint can get a bit squirrelly by itself. They often
/// work better when combined with prismatic joints. You should also cover the
/// the anchor points with static shapes to prevent one side from going to zero length.
/// </summary>
public class PulleyJoint : Joint
{
    // Solver shared
    private double _impulse;

    // Solver temp
    private int _indexA;
    private int _indexB;
    private Vector2 _uA;
    private Vector2 _uB;
    private Vector2 _rA;
    private Vector2 _rB;
    private Vector2 _localCenterA;
    private Vector2 _localCenterB;
    private double _invMassA;
    private double _invMassB;
    private double _invIA;
    private double _invIB;
    private double _mass;

    internal PulleyJoint()
    {
        JointType = JointType.Pulley;
    }

    /// <summary>
    /// Constructor for PulleyJoint.
    /// </summary>
    /// <param name="bodyA">The first body.</param>
    /// <param name="bodyB">The second body.</param>
    /// <param name="anchorA">The anchor on the first body.</param>
    /// <param name="anchorB">The anchor on the second body.</param>
    /// <param name="worldAnchorA">The world anchor for the first body.</param>
    /// <param name="worldAnchorB">The world anchor for the second body.</param>
    /// <param name="ratio">The ratio.</param>
    /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
    public PulleyJoint(Body bodyA, Body bodyB, Vector2 anchorA, Vector2 anchorB, Vector2 worldAnchorA, Vector2 worldAnchorB, double ratio, bool useWorldCoordinates = false)
        : base(bodyA, bodyB)
    {
        JointType = JointType.Pulley;

        WorldAnchorA = worldAnchorA;
        WorldAnchorB = worldAnchorB;

        if (useWorldCoordinates)
        {
            LocalAnchorA = BodyA.GetLocalPoint(anchorA);
            LocalAnchorB = BodyB.GetLocalPoint(anchorB);

            Vector2 dA = anchorA - worldAnchorA;
            LengthA = dA.Length;
            Vector2 dB = anchorB - worldAnchorB;
            LengthB = dB.Length;
        }
        else
        {
            LocalAnchorA = anchorA;
            LocalAnchorB = anchorB;

            Vector2 dA = anchorA - BodyA.GetLocalPoint(worldAnchorA);
            LengthA = dA.Length;
            Vector2 dB = anchorB - BodyB.GetLocalPoint(worldAnchorB);
            LengthB = dB.Length;
        }

        Debug.Assert(ratio != 0.0);
        Debug.Assert(ratio > MathUtils.Epsilon);

        Ratio = ratio;
        Constant = LengthA + ratio * LengthB;
        _impulse = 0.0;
    }

    /// <summary>
    /// The local anchor point on BodyA
    /// </summary>
    public Vector2 LocalAnchorA { get; set; }

    /// <summary>
    /// The local anchor point on BodyB
    /// </summary>
    public Vector2 LocalAnchorB { get; set; }

    /// <summary>
    /// Get the first world anchor.
    /// </summary>
    /// <value></value>
    public override sealed Vector2 WorldAnchorA { get; set; }

    /// <summary>
    /// Get the second world anchor.
    /// </summary>
    /// <value></value>
    public override sealed Vector2 WorldAnchorB { get; set; }

    /// <summary>
    /// Get the current length of the segment attached to body1.
    /// </summary>
    /// <value></value>
    public double LengthA { get; set; }

    /// <summary>
    /// Get the current length of the segment attached to body2.
    /// </summary>
    /// <value></value>
    public double LengthB { get; set; }

    /// <summary>
    /// The current length between the anchor point on BodyA and WorldAnchorA
    /// </summary>
    public double CurrentLengthA
    {
        get
        {
            Vector2 p = BodyA.GetWorldPoint(LocalAnchorA);
            Vector2 s = WorldAnchorA;
            Vector2 d = p - s;
            return d.Length;
        }
    }

    /// <summary>
    /// The current length between the anchor point on BodyB and WorldAnchorB
    /// </summary>
    public double CurrentLengthB
    {
        get
        {
            Vector2 p = BodyB.GetWorldPoint(LocalAnchorB);
            Vector2 s = WorldAnchorB;
            Vector2 d = p - s;
            return d.Length;
        }
    }

    /// <summary>
    /// Get the pulley ratio.
    /// </summary>
    /// <value></value>
    public double Ratio { get; set; }

    //FPE note: Only used for serialization.
    internal double Constant { get; set; }

    public override Vector2 GetReactionForce(double invDt)
    {
        Vector2 P = _impulse * _uB;
        return invDt * P;
    }

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

        // Get the pulley axes.
        _uA = cA + _rA - WorldAnchorA;
        _uB = cB + _rB - WorldAnchorB;

        double lengthA = _uA.Length;
        double lengthB = _uB.Length;

        if (lengthA > 10.0 * Settings.LinearSlop)
        {
            _uA *= 1.0 / lengthA;
        }
        else
        {
            _uA = Vector2.Zero;
        }

        if (lengthB > 10.0 * Settings.LinearSlop)
        {
            _uB *= 1.0 / lengthB;
        }
        else
        {
            _uB = Vector2.Zero;
        }

        // Compute effective mass.
        double ruA = MathUtils.Cross(ref _rA, ref _uA);
        double ruB = MathUtils.Cross(ref _rB, ref _uB);

        double mA = _invMassA + _invIA * ruA * ruA;
        double mB = _invMassB + _invIB * ruB * ruB;

        _mass = mA + Ratio * Ratio * mB;

        if (_mass > 0.0)
        {
            _mass = 1.0 / _mass;
        }

        if (data.step.warmStarting)
        {
            // Scale impulses to support variable time steps.
            _impulse *= data.step.dtRatio;

            // Warm starting.
            Vector2 PA = -_impulse * _uA;
            Vector2 PB = -Ratio * _impulse * _uB;

            vA += _invMassA * PA;
            wA += _invIA * MathUtils.Cross(ref _rA, ref PA);
            vB += _invMassB * PB;
            wB += _invIB * MathUtils.Cross(ref _rB, ref PB);
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

        Vector2 vpA = vA + MathUtils.Cross(wA, ref _rA);
        Vector2 vpB = vB + MathUtils.Cross(wB, ref _rB);

        double Cdot = -Vector2.Dot(_uA, vpA) - Ratio * Vector2.Dot(_uB, vpB);
        double impulse = -_mass * Cdot;
        _impulse += impulse;

        Vector2 PA = -impulse * _uA;
        Vector2 PB = -Ratio * impulse * _uB;
        vA += _invMassA * PA;
        wA += _invIA * MathUtils.Cross(ref _rA, ref PA);
        vB += _invMassB * PB;
        wB += _invIB * MathUtils.Cross(ref _rB, ref PB);

        data.velocities[_indexA].v = vA;
        data.velocities[_indexA].w = wA;
        data.velocities[_indexB].v = vB;
        data.velocities[_indexB].w = wB;
    }

    internal override bool SolvePositionConstraints(ref SolverData data)
    {
        Vector2 cA = data.positions[_indexA].c;
        double aA = data.positions[_indexA].a;
        Vector2 cB = data.positions[_indexB].c;
        double aB = data.positions[_indexB].a;

        Complex qA = Complex.FromAngle(aA);
        Complex qB = Complex.FromAngle(aB);

        Vector2 rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
        Vector2 rB = Complex.Multiply(LocalAnchorB - _localCenterB, ref qB);

        // Get the pulley axes.
        Vector2 uA = cA + rA - WorldAnchorA;
        Vector2 uB = cB + rB - WorldAnchorB;

        double lengthA = uA.Length;
        double lengthB = uB.Length;

        if (lengthA > 10.0 * Settings.LinearSlop)
        {
            uA *= 1.0 / lengthA;
        }
        else
        {
            uA = Vector2.Zero;
        }

        if (lengthB > 10.0 * Settings.LinearSlop)
        {
            uB *= 1.0 / lengthB;
        }
        else
        {
            uB = Vector2.Zero;
        }

        // Compute effective mass.
        double ruA = MathUtils.Cross(ref rA, ref uA);
        double ruB = MathUtils.Cross(ref rB, ref uB);

        double mA = _invMassA + _invIA * ruA * ruA;
        double mB = _invMassB + _invIB * ruB * ruB;

        double mass = mA + Ratio * Ratio * mB;

        if (mass > 0.0)
        {
            mass = 1.0 / mass;
        }

        double C = Constant - lengthA - Ratio * lengthB;
        double linearError = Math.Abs(C);

        double impulse = -mass * C;

        Vector2 PA = -impulse * uA;
        Vector2 PB = -Ratio * impulse * uB;

        cA += _invMassA * PA;
        aA += _invIA * MathUtils.Cross(ref rA, ref PA);
        cB += _invMassB * PB;
        aB += _invIB * MathUtils.Cross(ref rB, ref PB);

        data.positions[_indexA].c = cA;
        data.positions[_indexA].a = aA;
        data.positions[_indexB].c = cB;
        data.positions[_indexB].a = aB;

        return linearError < Settings.LinearSlop;
    }
}
