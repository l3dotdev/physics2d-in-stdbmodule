using StdbModule.Common;

namespace StdbModule.Physics2D.Dynamics.Joints;

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

/// <summary>
/// A wheel joint. This joint provides two degrees of freedom: translation
/// along an axis fixed in bodyA and rotation in the plane. You can use a
/// joint limit to restrict the range of motion and a joint motor to drive
/// the rotation or to model rotational friction.
/// This joint is designed for vehicle suspensions.
/// </summary>
public class WheelJoint : Joint
{
    // Solver shared
    private Vector2 _localXAxis;
    private Vector2 _localYAxis;

    private double _impulse;
    private double _motorImpulse;
    private double _springImpulse;

    private double _maxMotorTorque;
    private double _motorSpeed;
    private bool _enableMotor;

    // Solver temp
    private int _indexA;
    private int _indexB;
    private Vector2 _localCenterA;
    private Vector2 _localCenterB;
    private double _invMassA;
    private double _invMassB;
    private double _invIA;
    private double _invIB;

    private Vector2 _ax, _ay;
    private double _sAx, _sBx;
    private double _sAy, _sBy;

    private double _mass;
    private double _motorMass;
    private double _springMass;

    private double _bias;
    private double _gamma;
    private Vector2 _axis;

    internal WheelJoint()
    {
        JointType = JointType.Wheel;
    }

    /// <summary>
    /// Constructor for WheelJoint
    /// </summary>
    /// <param name="bodyA">The first body</param>
    /// <param name="bodyB">The second body</param>
    /// <param name="anchor">The anchor point</param>
    /// <param name="axis">The axis</param>
    /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
    public WheelJoint(Body bodyA, Body bodyB, Vector2 anchor, Vector2 axis, bool useWorldCoordinates = false)
        : base(bodyA, bodyB)
    {
        JointType = JointType.Wheel;

        if (useWorldCoordinates)
        {
            LocalAnchorA = bodyA.GetLocalPoint(anchor);
            LocalAnchorB = bodyB.GetLocalPoint(anchor);
        }
        else
        {
            LocalAnchorA = bodyA.GetLocalPoint(bodyB.GetWorldPoint(anchor));
            LocalAnchorB = anchor;
        }

        Axis = axis; //FPE only: We maintain the original value as it is supposed to.
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
    /// The axis at which the suspension moves.
    /// </summary>
    public Vector2 Axis
    {
        get { return _axis; }
        set
        {
            _axis = value;
            _localXAxis = BodyA.GetLocalVector(_axis);
            _localYAxis = MathUtils.Rot90(ref _localXAxis);
        }
    }

    /// <summary>
    /// The axis in local coordinates relative to BodyA
    /// </summary>
    public Vector2 LocalXAxis { get { return _localXAxis; } }

    /// <summary>
    /// The desired motor speed in radians per second.
    /// </summary>
    public double MotorSpeed
    {
        get { return _motorSpeed; }
        set
        {
            WakeBodies();
            _motorSpeed = value;
        }
    }

    /// <summary>
    /// The maximum motor torque, usually in N-m.
    /// </summary>
    public double MaxMotorTorque
    {
        get { return _maxMotorTorque; }
        set
        {
            WakeBodies();
            _maxMotorTorque = value;
        }
    }

    /// <summary>
    /// Suspension frequency, zero indicates no suspension
    /// </summary>
    public double Frequency { get; set; }

    /// <summary>
    /// Suspension damping ratio, one indicates critical damping
    /// </summary>
    public double DampingRatio { get; set; }

    /// <summary>
    /// Gets the translation along the axis
    /// </summary>
    public double JointTranslation
    {
        get
        {
            Body bA = BodyA;
            Body bB = BodyB;

            Vector2 pA = bA.GetWorldPoint(LocalAnchorA);
            Vector2 pB = bB.GetWorldPoint(LocalAnchorB);
            Vector2 d = pB - pA;
            Vector2 axis = bA.GetWorldVector(ref _localXAxis);

            double translation = Vector2.Dot(d, axis);
            return translation;
        }
    }

    /// <summary>
    /// Gets the angular velocity of the joint
    /// </summary>
    public double JointSpeed
    {
        get
        {
            double wA = BodyA.AngularVelocity;
            double wB = BodyB.AngularVelocity;
            return wB - wA;
        }
    }

    /// <summary>
    /// Enable/disable the joint motor.
    /// </summary>
    public bool MotorEnabled
    {
        get { return _enableMotor; }
        set
        {
            WakeBodies();
            _enableMotor = value;
        }
    }

    /// <summary>
    /// Gets the torque of the motor
    /// </summary>
    /// <param name="invDt">inverse delta time</param>
    public double GetMotorTorque(double invDt)
    {
        return invDt * _motorImpulse;
    }

    public override Vector2 GetReactionForce(double invDt)
    {
        return invDt * (_impulse * _ay + _springImpulse * _ax);
    }

    public override double GetReactionTorque(double invDt)
    {
        return invDt * _motorImpulse;
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

        double mA = _invMassA, mB = _invMassB;
        double iA = _invIA, iB = _invIB;

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

        // Compute the effective masses.
        Vector2 rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
        Vector2 rB = Complex.Multiply(LocalAnchorB - _localCenterB, ref qB);
        Vector2 d1 = cB + rB - cA - rA;

        // Point to line constraint
        {
            _ay = Complex.Multiply(ref _localYAxis, ref qA);
            _sAy = MathUtils.Cross(d1 + rA, _ay);
            _sBy = MathUtils.Cross(ref rB, ref _ay);

            _mass = mA + mB + iA * _sAy * _sAy + iB * _sBy * _sBy;

            if (_mass > 0.0)
            {
                _mass = 1.0 / _mass;
            }
        }

        // Spring constraint
        _springMass = 0.0;
        _bias = 0.0;
        _gamma = 0.0;
        if (Frequency > 0.0)
        {
            _ax = Complex.Multiply(ref _localXAxis, ref qA);
            _sAx = MathUtils.Cross(d1 + rA, _ax);
            _sBx = MathUtils.Cross(ref rB, ref _ax);

            double invMass = mA + mB + iA * _sAx * _sAx + iB * _sBx * _sBx;

            if (invMass > 0.0)
            {
                _springMass = 1.0 / invMass;

                double C = Vector2.Dot(d1, _ax);

                // Frequency
                double omega = MathUtils.Tau * Frequency;

                // Damping coefficient
                double d = 2.0 * _springMass * DampingRatio * omega;

                // Spring stiffness
                double k = _springMass * omega * omega;

                // magic formulas
                double h = data.step.dt;
                _gamma = h * (d + h * k);
                if (_gamma > 0.0)
                {
                    _gamma = 1.0 / _gamma;
                }

                _bias = C * h * k * _gamma;

                _springMass = invMass + _gamma;
                if (_springMass > 0.0)
                {
                    _springMass = 1.0 / _springMass;
                }
            }
        }
        else
        {
            _springImpulse = 0.0;
        }

        // Rotational motor
        if (_enableMotor)
        {
            _motorMass = iA + iB;
            if (_motorMass > 0.0)
            {
                _motorMass = 1.0 / _motorMass;
            }
        }
        else
        {
            _motorMass = 0.0;
            _motorImpulse = 0.0;
        }

        if (data.step.warmStarting)
        {
            // Account for variable time step.
            _impulse *= data.step.dtRatio;
            _springImpulse *= data.step.dtRatio;
            _motorImpulse *= data.step.dtRatio;

            Vector2 P = _impulse * _ay + _springImpulse * _ax;
            double LA = _impulse * _sAy + _springImpulse * _sAx + _motorImpulse;
            double LB = _impulse * _sBy + _springImpulse * _sBx + _motorImpulse;

            vA -= _invMassA * P;
            wA -= _invIA * LA;

            vB += _invMassB * P;
            wB += _invIB * LB;
        }
        else
        {
            _impulse = 0.0;
            _springImpulse = 0.0;
            _motorImpulse = 0.0;
        }

        data.velocities[_indexA].v = vA;
        data.velocities[_indexA].w = wA;
        data.velocities[_indexB].v = vB;
        data.velocities[_indexB].w = wB;
    }

    internal override void SolveVelocityConstraints(ref SolverData data)
    {
        double mA = _invMassA, mB = _invMassB;
        double iA = _invIA, iB = _invIB;

        Vector2 vA = data.velocities[_indexA].v;
        double wA = data.velocities[_indexA].w;
        Vector2 vB = data.velocities[_indexB].v;
        double wB = data.velocities[_indexB].w;

        // Solve spring constraint
        {
            double Cdot = Vector2.Dot(_ax, vB - vA) + _sBx * wB - _sAx * wA;
            double impulse = -_springMass * (Cdot + _bias + _gamma * _springImpulse);
            _springImpulse += impulse;

            Vector2 P = impulse * _ax;
            double LA = impulse * _sAx;
            double LB = impulse * _sBx;

            vA -= mA * P;
            wA -= iA * LA;

            vB += mB * P;
            wB += iB * LB;
        }

        // Solve rotational motor constraint
        {
            double Cdot = wB - wA - _motorSpeed;
            double impulse = -_motorMass * Cdot;

            double oldImpulse = _motorImpulse;
            double maxImpulse = data.step.dt * _maxMotorTorque;
            _motorImpulse = MathUtils.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = _motorImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Solve point to line constraint
        {
            double Cdot = Vector2.Dot(_ay, vB - vA) + _sBy * wB - _sAy * wA;
            double impulse = -_mass * Cdot;
            _impulse += impulse;

            Vector2 P = impulse * _ay;
            double LA = impulse * _sAy;
            double LB = impulse * _sBy;

            vA -= mA * P;
            wA -= iA * LA;

            vB += mB * P;
            wB += iB * LB;
        }

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
        Vector2 d = (cB - cA) + rB - rA;

        Vector2 ay = Complex.Multiply(ref _localYAxis, ref qA);

        double sAy = MathUtils.Cross(d + rA, ay);
        double sBy = MathUtils.Cross(ref rB, ref ay);

        double C = Vector2.Dot(d, ay);

        double k = _invMassA + _invMassB + _invIA * _sAy * _sAy + _invIB * _sBy * _sBy;

        double impulse;
        if (k != 0.0)
        {
            impulse = -C / k;
        }
        else
        {
            impulse = 0.0;
        }

        Vector2 P = impulse * ay;
        double LA = impulse * sAy;
        double LB = impulse * sBy;

        cA -= _invMassA * P;
        aA -= _invIA * LA;
        cB += _invMassB * P;
        aB += _invIB * LB;

        data.positions[_indexA].c = cA;
        data.positions[_indexA].a = aA;
        data.positions[_indexB].c = cB;
        data.positions[_indexB].a = aB;

        return Math.Abs(C) <= Settings.LinearSlop;
    }
}
