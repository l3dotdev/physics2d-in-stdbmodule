using StdbModule.Common;

namespace StdbModule.Physics2D.Dynamics.Joints;

// Point-to-point constraint
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

/// <summary>
/// Friction joint. This is used for top-down friction.
/// It provides 2D translational friction and angular friction.
/// </summary>
public class FrictionJoint : Joint
{
    // Solver shared
    private Vector2 _linearImpulse;
    private double _angularImpulse;

    // Solver temp
    private int _indexA;
    private int _indexB;
    private Vector2 _rA;
    private Vector2 _rB;
    private Vector2 _localCenterA;
    private Vector2 _localCenterB;
    private double _invMassA;
    private double _invMassB;
    private double _invIA;
    private double _invIB;
    private double _angularMass;
    private Matrix2x2 _linearMass;

    internal FrictionJoint()
    {
        JointType = JointType.Friction;
    }

    /// <summary>
    /// Constructor for FrictionJoint.
    /// </summary>
    /// <param name="bodyA"></param>
    /// <param name="bodyB"></param>
    /// <param name="anchor"></param>
    /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
    public FrictionJoint(Body bodyA, Body bodyB, Vector2 anchor, bool useWorldCoordinates = false)
        : base(bodyA, bodyB)
    {
        JointType = JointType.Friction;

        if (useWorldCoordinates)
        {
            LocalAnchorA = BodyA.GetLocalPoint(anchor);
            LocalAnchorB = BodyB.GetLocalPoint(anchor);
        }
        else
        {
            LocalAnchorA = anchor;
            LocalAnchorB = anchor;
        }
    }

    /// <summary>
    /// The local anchor point on BodyA
    /// </summary>
    public Vector2 LocalAnchorA { get; set; }

    /// <summary>
    /// The local anchor point on BodyB
    /// </summary>
    public Vector2 LocalAnchorB { get; set; }

    public override Vector2 WorldAnchorA
    {
        get { return BodyA.GetWorldPoint(LocalAnchorA); }
        set { LocalAnchorA = BodyA.GetLocalPoint(value); }
    }

    public override Vector2 WorldAnchorB
    {
        get { return BodyB.GetWorldPoint(LocalAnchorB); }
        set { LocalAnchorB = BodyB.GetLocalPoint(value); }
    }

    /// <summary>
    /// The maximum friction force in N.
    /// </summary>
    public double MaxForce { get; set; }

    /// <summary>
    /// The maximum friction torque in N-m.
    /// </summary>
    public double MaxTorque { get; set; }

    public override Vector2 GetReactionForce(double invDt)
    {
        return invDt * _linearImpulse;
    }

    public override double GetReactionTorque(double invDt)
    {
        return invDt * _angularImpulse;
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

        double aA = data.positions[_indexA].a;
        Vector2 vA = data.velocities[_indexA].v;
        double wA = data.velocities[_indexA].w;

        double aB = data.positions[_indexB].a;
        Vector2 vB = data.velocities[_indexB].v;
        double wB = data.velocities[_indexB].w;

        Complex qA = Complex.FromAngle(aA);
        Complex qB = Complex.FromAngle(aB);

        // Compute the effective mass matrix.
        _rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
        _rB = Complex.Multiply(LocalAnchorB - _localCenterB, ref qB);

        // J = [-I -r1_skew I r2_skew]
        //     [ 0       -1 0       1]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

        double mA = _invMassA, mB = _invMassB;
        double iA = _invIA, iB = _invIB;

        Matrix2x2 K = new();
        K.Ex.X = mA + mB + iA * _rA.Y * _rA.Y + iB * _rB.Y * _rB.Y;
        K.Ex.Y = -iA * _rA.X * _rA.Y - iB * _rB.X * _rB.Y;
        K.Ey.X = K.Ex.Y;
        K.Ey.Y = mA + mB + iA * _rA.X * _rA.X + iB * _rB.X * _rB.X;

        _linearMass = K.Inverse;

        _angularMass = iA + iB;
        if (_angularMass > 0.0)
        {
            _angularMass = 1.0 / _angularMass;
        }

        if (data.step.warmStarting)
        {
            // Scale impulses to support a variable time step.
            _linearImpulse *= data.step.dtRatio;
            _angularImpulse *= data.step.dtRatio;

            Vector2 P = new(_linearImpulse.X, _linearImpulse.Y);
            vA -= mA * P;
            wA -= iA * (MathUtils.Cross(ref _rA, ref P) + _angularImpulse);
            vB += mB * P;
            wB += iB * (MathUtils.Cross(ref _rB, ref P) + _angularImpulse);
        }
        else
        {
            _linearImpulse = Vector2.Zero;
            _angularImpulse = 0.0;
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

        double mA = _invMassA, mB = _invMassB;
        double iA = _invIA, iB = _invIB;

        double h = data.step.dt;

        // Solve angular friction
        {
            double Cdot = wB - wA;
            double impulse = -_angularMass * Cdot;

            double oldImpulse = _angularImpulse;
            double maxImpulse = h * MaxTorque;
            _angularImpulse = MathUtils.Clamp(_angularImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = _angularImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Solve linear friction
        {
            Vector2 Cdot = vB + MathUtils.Cross(wB, ref _rB) - vA - MathUtils.Cross(wA, ref _rA);

            Vector2 impulse = -MathUtils.Mul(ref _linearMass, ref Cdot);
            Vector2 oldImpulse = _linearImpulse;
            _linearImpulse += impulse;

            double maxImpulse = h * MaxForce;

            if (_linearImpulse.LengthSquared > maxImpulse * maxImpulse)
            {
                _linearImpulse = _linearImpulse.Normalized;
                _linearImpulse *= maxImpulse;
            }

            impulse = _linearImpulse - oldImpulse;

            vA -= mA * impulse;
            wA -= iA * MathUtils.Cross(ref _rA, ref impulse);

            vB += mB * impulse;
            wB += iB * MathUtils.Cross(ref _rB, ref impulse);
        }

        data.velocities[_indexA].v = vA;
        data.velocities[_indexA].w = wA;
        data.velocities[_indexB].v = vB;
        data.velocities[_indexB].w = wB;
    }

    internal override bool SolvePositionConstraints(ref SolverData data)
    {
        return true;
    }
}
