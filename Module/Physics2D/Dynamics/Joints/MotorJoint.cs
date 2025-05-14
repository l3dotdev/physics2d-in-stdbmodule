using System.Diagnostics;
using StdbModule.Common;

namespace StdbModule.Physics2D.Dynamics.Joints;

/// <summary>
/// A motor joint is used to control the relative motion
/// between two bodies. A typical usage is to control the movement
/// of a dynamic body with respect to the ground.
/// </summary>
public class MotorJoint : Joint
{
    // Solver shared
    private Vector2 _linearOffset;
    private double _angularOffset;
    private Vector2 _linearImpulse;
    private double _angularImpulse;
    private double _maxForce;
    private double _maxTorque;

    // Solver temp
    private int _indexA;
    private int _indexB;
    private Vector2 _rA;
    private Vector2 _rB;
    private Vector2 _localCenterA;
    private Vector2 _localCenterB;
    private Vector2 _linearError;
    private double _angularError;
    private double _invMassA;
    private double _invMassB;
    private double _invIA;
    private double _invIB;
    private Matrix2x2 _linearMass;
    private double _angularMass;

    internal MotorJoint()
    {
        JointType = JointType.Motor;
    }

    /// <summary>
    /// Constructor for MotorJoint.
    /// </summary>
    /// <param name="bodyA">The first body</param>
    /// <param name="bodyB">The second body</param>
    /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
    public MotorJoint(Body bodyA, Body bodyB, bool useWorldCoordinates = false)
        : base(bodyA, bodyB)
    {
        JointType = JointType.Motor;

        Vector2 xB = BodyB.Position;

        if (useWorldCoordinates)
            _linearOffset = BodyA.GetLocalPoint(xB);
        else
            _linearOffset = xB;

        //Defaults
        _angularOffset = 0.0;
        _maxForce = 1.0;
        _maxTorque = 1.0;
        CorrectionFactor = 0.3;

        _angularOffset = BodyB.Rotation - BodyA.Rotation;
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
    /// The maximum amount of force that can be applied to BodyA
    /// </summary>
    public double MaxForce
    {
        set
        {
            Debug.Assert(MathUtils.IsValid(value) && value >= 0.0);
            _maxForce = value;
        }
        get { return _maxForce; }
    }

    /// <summary>
    /// The maximum amount of torque that can be applied to BodyA
    /// </summary>
    public double MaxTorque
    {
        set
        {
            Debug.Assert(MathUtils.IsValid(value) && value >= 0.0);
            _maxTorque = value;
        }
        get { return _maxTorque; }
    }

    /// <summary>
    /// The linear (translation) offset.
    /// </summary>
    public Vector2 LinearOffset
    {
        set
        {
            if (_linearOffset.X != value.X || _linearOffset.Y != value.Y)
            {
                WakeBodies();
                _linearOffset = value;
            }
        }
        get { return _linearOffset; }
    }

    /// <summary>
    /// Get or set the angular offset.
    /// </summary>
    public double AngularOffset
    {
        set
        {
            if (_angularOffset != value)
            {
                WakeBodies();
                _angularOffset = value;
            }
        }
        get { return _angularOffset; }
    }

    //FPE note: Used for serialization.
    internal double CorrectionFactor { get; set; }

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

        // Compute the effective mass matrix.
        _rA = -Complex.Multiply(ref _localCenterA, ref qA);
        _rB = -Complex.Multiply(ref _localCenterB, ref qB);

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

        _linearError = cB + _rB - cA - _rA - Complex.Multiply(ref _linearOffset, ref qA);
        _angularError = aB - aA - _angularOffset;

        if (data.step.warmStarting)
        {
            // Scale impulses to support a variable time step.
            _linearImpulse *= data.step.dtRatio;
            _angularImpulse *= data.step.dtRatio;

            Vector2 P = new Vector2(_linearImpulse.X, _linearImpulse.Y);

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
        double inv_h = data.step.inv_dt;

        // Solve angular friction
        {
            double Cdot = wB - wA + inv_h * CorrectionFactor * _angularError;
            double impulse = -_angularMass * Cdot;

            double oldImpulse = _angularImpulse;
            double maxImpulse = h * _maxTorque;
            _angularImpulse = MathUtils.Clamp(_angularImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = _angularImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Solve linear friction
        {
            Vector2 Cdot = vB + MathUtils.Cross(wB, ref _rB) - vA - MathUtils.Cross(wA, ref _rA) + inv_h * CorrectionFactor * _linearError;

            Vector2 impulse = -MathUtils.Mul(ref _linearMass, ref Cdot);
            Vector2 oldImpulse = _linearImpulse;
            _linearImpulse += impulse;

            double maxImpulse = h * _maxForce;

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