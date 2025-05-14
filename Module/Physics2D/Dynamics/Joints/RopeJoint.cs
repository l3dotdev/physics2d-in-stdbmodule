using StdbModule.Common;

namespace StdbModule.Physics2D.Dynamics.Joints;

// Limit:
// C = norm(pB - pA) - L
// u = (pB - pA) / norm(pB - pA)
// Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
// J = [-u -cross(rA, u) u cross(rB, u)]
// K = J * invM * JT
//   = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2

/// <summary>
/// A rope joint enforces a maximum distance between two points on two bodies. It has no other effect.
/// It can be used on ropes that are made up of several connected bodies, and if there is a need to support a heavy body.
/// This joint is used for stabiliation of heavy objects on soft constraint joints.
/// 
/// Warning: if you attempt to change the maximum length during the simulation you will get some non-physical behavior.
/// Use the DistanceJoint instead if you want to dynamically control the length.
/// </summary>
public class RopeJoint : Joint
{
    // Solver shared
    private double _impulse;
    private double _length;

    // Solver temp
    private int _indexA;
    private int _indexB;
    private Vector2 _localCenterA;
    private Vector2 _localCenterB;
    private double _invMassA;
    private double _invMassB;
    private double _invIA;
    private double _invIB;
    private double _mass;
    private Vector2 _rA, _rB;
    private Vector2 _u;

    internal RopeJoint()
    {
        JointType = JointType.Rope;
    }

    /// <summary>
    /// Constructor for RopeJoint.
    /// </summary>
    /// <param name="bodyA">The first body</param>
    /// <param name="bodyB">The second body</param>
    /// <param name="anchorA">The anchor on the first body</param>
    /// <param name="anchorB">The anchor on the second body</param>
    /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
    public RopeJoint(Body bodyA, Body bodyB, Vector2 anchorA, Vector2 anchorB, bool useWorldCoordinates = false)
        : base(bodyA, bodyB)
    {
        JointType = JointType.Rope;

        if (useWorldCoordinates)
        {
            LocalAnchorA = bodyA.GetLocalPoint(anchorA);
            LocalAnchorB = bodyB.GetLocalPoint(anchorB);
        }
        else
        {
            LocalAnchorA = anchorA;
            LocalAnchorB = anchorB;
        }

        //FPE feature: Setting default MaxLength
        Vector2 d = WorldAnchorB - WorldAnchorA;
        MaxLength = d.Length;
    }

    /// <summary>
    /// The local anchor point on BodyA
    /// </summary>
    public Vector2 LocalAnchorA { get; set; }

    /// <summary>
    /// The local anchor point on BodyB
    /// </summary>
    public Vector2 LocalAnchorB { get; set; }

    public override sealed Vector2 WorldAnchorA
    {
        get { return BodyA.GetWorldPoint(LocalAnchorA); }
        set { LocalAnchorA = BodyA.GetLocalPoint(value); }
    }

    public override sealed Vector2 WorldAnchorB
    {
        get { return BodyB.GetWorldPoint(LocalAnchorB); }
        set { LocalAnchorB = BodyB.GetLocalPoint(value); }
    }

    /// <summary>
    /// Get or set the maximum length of the rope.
    /// By default, it is the distance between the two anchor points.
    /// </summary>
    public double MaxLength { get; set; }

    /// <summary>
    /// Gets the state of the joint.
    /// </summary>
    public LimitState State { get; private set; }

    public override Vector2 GetReactionForce(double invDt)
    {
        return invDt * _impulse * _u;
    }

    public override double GetReactionTorque(double invDt)
    {
        return 0;
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

        _length = _u.Length;

        double C = _length - MaxLength;
        if (C > 0.0)
        {
            State = LimitState.AtUpper;
        }
        else
        {
            State = LimitState.Inactive;
        }

        if (_length > Settings.LinearSlop)
        {
            _u *= 1.0 / _length;
        }
        else
        {
            _u = Vector2.Zero;
            _mass = 0.0;
            _impulse = 0.0;
            return;
        }

        // Compute effective mass.
        double crA = MathUtils.Cross(ref _rA, ref _u);
        double crB = MathUtils.Cross(ref _rB, ref _u);
        double invMass = _invMassA + _invIA * crA * crA + _invMassB + _invIB * crB * crB;

        _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

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
        double C = _length - MaxLength;
        double Cdot = Vector2.Dot(_u, vpB - vpA);

        // Predictive constraint.
        if (C < 0.0)
        {
            Cdot += data.step.inv_dt * C;
        }

        double impulse = -_mass * Cdot;
        double oldImpulse = _impulse;
        _impulse = Math.Min(0.0, _impulse + impulse);
        impulse = _impulse - oldImpulse;

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
        double C = length - MaxLength;

        C = MathUtils.Clamp(C, 0.0, Settings.MaxLinearCorrection);

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

        return length - MaxLength < Settings.LinearSlop;
    }
}
