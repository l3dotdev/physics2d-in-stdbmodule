using System.Diagnostics;
using StdbModule.Common;

namespace StdbModule.Physics2D.Dynamics.Joints;

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)
// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]
// Block Solver
// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
// when the mass has poor distribution (leading to large torques about the joint anchor points).
//
// The Jacobian has 3 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//     [-vT -a1 vT a2] // limit
//
// u = perp
// v = axis
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)
// M * (v2 - v1) = JT * df
// J * v2 = bias
//
// v2 = v1 + invM * JT * df
// J * (v1 + invM * JT * df) = bias
// K * df = bias - J * v1 = -Cdot
// K = J * invM * JT
// Cdot = J * v1 - bias
//
// Now solve for f2.
// df = f2 - f1
// K * (f2 - f1) = -Cdot
// f2 = invK * (-Cdot) + f1
//
// Clamp accumulated limit impulse.
// lower: f2(3) = max(f2(3), 0)
// upper: f2(3) = min(f2(3), 0)
//
// Solve for correct f2(1:2)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
//                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
//
// Now compute impulse to be applied:
// df = f2 - f1

/// <summary>
/// A prismatic joint. This joint provides one degree of freedom: translation
/// along an axis fixed in bodyA. Relative rotation is prevented. You can
/// use a joint limit to restrict the range of motion and a joint motor to
/// drive the motion or to model joint friction.
/// </summary>
public class PrismaticJoint : Joint
{
    private Vector2 _localXAxis;
    private Vector2 _localYAxisA;
    private Vector3 _impulse;
    private double _lowerTranslation;
    private double _upperTranslation;
    private double _maxMotorForce;
    private double _motorSpeed;
    private bool _enableLimit;
    private bool _enableMotor;
    private LimitState _limitState;

    // Solver temp
    private int _indexA;
    private int _indexB;
    private Vector2 _localCenterA;
    private Vector2 _localCenterB;
    private double _invMassA;
    private double _invMassB;
    private double _invIA;
    private double _invIB;
    private Vector2 _axis, _perp;
    private double _s1, _s2;
    private double _a1, _a2;
    private Matrix3x3 _K;
    private double _motorMass;
    private Vector2 _axis1;

    internal PrismaticJoint()
    {
        JointType = JointType.Prismatic;
    }

    /// <summary>
    /// This requires defining a line of
    /// motion using an axis and an anchor point. The definition uses local
    /// anchor points and a local axis so that the initial configuration
    /// can violate the constraint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space. Using local
    /// anchors and a local axis helps when saving and loading a game.
    /// </summary>
    /// <param name="bodyA">The first body.</param>
    /// <param name="bodyB">The second body.</param>
    /// <param name="anchorA">The first body anchor.</param>
    /// <param name="anchorB">The second body anchor.</param>
    /// <param name="axis">The axis.</param>
    /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
    public PrismaticJoint(Body bodyA, Body bodyB, Vector2 anchorA, Vector2 anchorB, Vector2 axis, bool useWorldCoordinates = false)
        : base(bodyA, bodyB)
    {
        Initialize(anchorA, anchorB, axis, useWorldCoordinates);
    }

    public PrismaticJoint(Body bodyA, Body bodyB, Vector2 anchor, Vector2 axis, bool useWorldCoordinates = false)
        : base(bodyA, bodyB)
    {
        Initialize(anchor, anchor, axis, useWorldCoordinates);
    }

    private void Initialize(Vector2 localAnchorA, Vector2 localAnchorB, Vector2 axis, bool useWorldCoordinates)
    {
        JointType = JointType.Prismatic;

        if (useWorldCoordinates)
        {
            LocalAnchorA = BodyA.GetLocalPoint(localAnchorA);
            LocalAnchorB = BodyB.GetLocalPoint(localAnchorB);
        }
        else
        {
            LocalAnchorA = localAnchorA;
            LocalAnchorB = localAnchorB;
        }

        Axis = axis; //FPE only: store the orignal value for use in Serialization
        ReferenceAngle = BodyB.Rotation - BodyA.Rotation;

        _limitState = LimitState.Inactive;
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
    /// Get the current joint translation, usually in meters.
    /// </summary>
    /// <value></value>
    public double JointTranslation
    {
        get
        {
            Vector2 d = BodyB.GetWorldPoint(LocalAnchorB) - BodyA.GetWorldPoint(LocalAnchorA);
            Vector2 axis = BodyA.GetWorldVector(ref _localXAxis);

            return Vector2.Dot(d, axis);
        }
    }

    /// <summary>
    /// Get the current joint translation speed, usually in meters per second.
    /// </summary>
    /// <value></value>
    public double JointSpeed
    {
        get
        {
            Transform xf1 = BodyA.GetTransform();
            Transform xf2 = BodyB.GetTransform();

            Vector2 r1 = Complex.Multiply(LocalAnchorA - BodyA.LocalCenter, ref xf1.Rotation);
            Vector2 r2 = Complex.Multiply(LocalAnchorB - BodyB.LocalCenter, ref xf2.Rotation);
            Vector2 p1 = BodyA._sweep.C + r1;
            Vector2 p2 = BodyB._sweep.C + r2;
            Vector2 d = p2 - p1;
            Vector2 axis = BodyA.GetWorldVector(ref _localXAxis);

            Vector2 v1 = BodyA._linearVelocity;
            Vector2 v2 = BodyB._linearVelocity;
            double w1 = BodyA._angularVelocity;
            double w2 = BodyB._angularVelocity;

            double speed = Vector2.Dot(d, MathUtils.Cross(w1, ref axis)) + Vector2.Dot(axis, v2 + MathUtils.Cross(w2, ref r2) - v1 - MathUtils.Cross(w1, ref r1));
            return speed;
        }
    }

    /// <summary>
    /// Is the joint limit enabled?
    /// </summary>
    /// <value><c>true</c> if [limit enabled]; otherwise, <c>false</c>.</value>
    public bool LimitEnabled
    {
        get { return _enableLimit; }
        set
        {
            Debug.Assert(BodyA.FixedRotation == false || BodyB.FixedRotation == false, "Warning: limits does currently not work with fixed rotation");

            if (value != _enableLimit)
            {
                WakeBodies();
                _enableLimit = value;
                _impulse.Z = 0;
            }
        }
    }

    /// <summary>
    /// Get the lower joint limit, usually in meters.
    /// </summary>
    /// <value></value>
    public double LowerLimit
    {
        get { return _lowerTranslation; }
        set
        {
            if (value != _lowerTranslation)
            {
                WakeBodies();
                _lowerTranslation = value;
                _impulse.Z = 0.0;
            }
        }
    }

    /// <summary>
    /// Get the upper joint limit, usually in meters.
    /// </summary>
    /// <value></value>
    public double UpperLimit
    {
        get { return _upperTranslation; }
        set
        {
            if (value != _upperTranslation)
            {
                WakeBodies();
                _upperTranslation = value;
                _impulse.Z = 0.0;
            }
        }
    }

    /// <summary>
    /// Set the joint limits, usually in meters.
    /// </summary>
    /// <param name="lower">The lower limit</param>
    /// <param name="upper">The upper limit</param>
    public void SetLimits(double lower, double upper)
    {
        if (upper != _upperTranslation || lower != _lowerTranslation)
        {
            WakeBodies();
            _upperTranslation = upper;
            _lowerTranslation = lower;
            _impulse.Z = 0.0;
        }
    }

    /// <summary>
    /// Is the joint motor enabled?
    /// </summary>
    /// <value><c>true</c> if [motor enabled]; otherwise, <c>false</c>.</value>
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
    /// Set the motor speed, usually in meters per second.
    /// </summary>
    /// <value>The speed.</value>
    public double MotorSpeed
    {
        set
        {
            WakeBodies();
            _motorSpeed = value;
        }
        get { return _motorSpeed; }
    }

    /// <summary>
    /// Set the maximum motor force, usually in N.
    /// </summary>
    /// <value>The force.</value>
    public double MaxMotorForce
    {
        get { return _maxMotorForce; }
        set
        {
            WakeBodies();
            _maxMotorForce = value;
        }
    }

    /// <summary>
    /// Get the current motor impulse, usually in N.
    /// </summary>
    /// <value></value>
    public double MotorImpulse { get; set; }

    /// <summary>
    /// Gets the motor force.
    /// </summary>
    /// <param name="invDt">The inverse delta time</param>
    public double GetMotorForce(double invDt)
    {
        return invDt * MotorImpulse;
    }

    /// <summary>
    /// The axis at which the joint moves.
    /// </summary>
    public Vector2 Axis
    {
        get { return _axis1; }
        set
        {
            _axis1 = value;
            _localXAxis = BodyA.GetLocalVector(_axis1).Normalized;
            _localYAxisA = MathUtils.Cross(1.0, ref _localXAxis);
        }
    }

    /// <summary>
    /// The axis in local coordinates relative to BodyA
    /// </summary>
    public Vector2 LocalXAxis { get { return _localXAxis; } }

    /// <summary>
    /// The reference angle.
    /// </summary>
    public double ReferenceAngle { get; set; }

    public override Vector2 GetReactionForce(double invDt)
    {
        return invDt * (_impulse.X * _perp + (MotorImpulse + _impulse.Z) * _axis);
    }

    public override double GetReactionTorque(double invDt)
    {
        return invDt * _impulse.Y;
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

        // Compute the effective masses.
        Vector2 rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
        Vector2 rB = Complex.Multiply(LocalAnchorB - _localCenterB, ref qB);
        Vector2 d = (cB - cA) + rB - rA;

        double mA = _invMassA, mB = _invMassB;
        double iA = _invIA, iB = _invIB;

        // Compute motor Jacobian and effective mass.
        {
            _axis = Complex.Multiply(ref _localXAxis, ref qA);
            _a1 = MathUtils.Cross(d + rA, _axis);
            _a2 = MathUtils.Cross(ref rB, ref _axis);

            _motorMass = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;
            if (_motorMass > 0.0)
            {
                _motorMass = 1.0 / _motorMass;
            }
        }

        // Prismatic constraint.
        {
            _perp = Complex.Multiply(ref _localYAxisA, ref qA);

            _s1 = MathUtils.Cross(d + rA, _perp);
            _s2 = MathUtils.Cross(ref rB, ref _perp);

            double k11 = mA + mB + iA * _s1 * _s1 + iB * _s2 * _s2;
            double k12 = iA * _s1 + iB * _s2;
            double k13 = iA * _s1 * _a1 + iB * _s2 * _a2;
            double k22 = iA + iB;
            if (k22 == 0.0)
            {
                // For bodies with fixed rotation.
                k22 = 1.0;
            }
            double k23 = iA * _a1 + iB * _a2;
            double k33 = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;

            _K.Ex = new Vector3(k11, k12, k13);
            _K.Ey = new Vector3(k12, k22, k23);
            _K.Ez = new Vector3(k13, k23, k33);
        }

        // Compute motor and limit terms.
        if (_enableLimit)
        {
            double jointTranslation = Vector2.Dot(_axis, d);
            if (Math.Abs(_upperTranslation - _lowerTranslation) < 2.0 * Settings.LinearSlop)
            {
                _limitState = LimitState.Equal;
            }
            else if (jointTranslation <= _lowerTranslation)
            {
                if (_limitState != LimitState.AtLower)
                {
                    _limitState = LimitState.AtLower;
                    _impulse.Z = 0.0;
                }
            }
            else if (jointTranslation >= _upperTranslation)
            {
                if (_limitState != LimitState.AtUpper)
                {
                    _limitState = LimitState.AtUpper;
                    _impulse.Z = 0.0;
                }
            }
            else
            {
                _limitState = LimitState.Inactive;
                _impulse.Z = 0.0;
            }
        }
        else
        {
            _limitState = LimitState.Inactive;
            _impulse.Z = 0.0;
        }

        if (_enableMotor == false)
        {
            MotorImpulse = 0.0;
        }

        if (data.step.warmStarting)
        {
            // Account for variable time step.
            _impulse *= data.step.dtRatio;
            MotorImpulse *= data.step.dtRatio;

            Vector2 P = _impulse.X * _perp + (MotorImpulse + _impulse.Z) * _axis;
            double LA = _impulse.X * _s1 + _impulse.Y + (MotorImpulse + _impulse.Z) * _a1;
            double LB = _impulse.X * _s2 + _impulse.Y + (MotorImpulse + _impulse.Z) * _a2;

            vA -= mA * P;
            wA -= iA * LA;

            vB += mB * P;
            wB += iB * LB;
        }
        else
        {
            _impulse = Vector3.Zero;
            MotorImpulse = 0.0;
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

        // Solve linear motor constraint.
        if (_enableMotor && _limitState != LimitState.Equal)
        {
            double Cdot = Vector2.Dot(_axis, vB - vA) + _a2 * wB - _a1 * wA;
            double impulse = _motorMass * (_motorSpeed - Cdot);
            double oldImpulse = MotorImpulse;
            double maxImpulse = data.step.dt * _maxMotorForce;
            MotorImpulse = MathUtils.Clamp(MotorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = MotorImpulse - oldImpulse;

            Vector2 P = impulse * _axis;
            double LA = impulse * _a1;
            double LB = impulse * _a2;

            vA -= mA * P;
            wA -= iA * LA;

            vB += mB * P;
            wB += iB * LB;
        }

        Vector2 Cdot1 = new Vector2();
        Cdot1.X = Vector2.Dot(_perp, vB - vA) + _s2 * wB - _s1 * wA;
        Cdot1.Y = wB - wA;

        if (_enableLimit && _limitState != LimitState.Inactive)
        {
            // Solve prismatic and limit constraint in block form.
            double Cdot2;
            Cdot2 = Vector2.Dot(_axis, vB - vA) + _a2 * wB - _a1 * wA;
            Vector3 Cdot = new Vector3(Cdot1.X, Cdot1.Y, Cdot2);

            Vector3 f1 = _impulse;
            Vector3 df = _K.Solve3x3(-Cdot);
            _impulse += df;

            if (_limitState == LimitState.AtLower)
            {
                _impulse.Z = Math.Max(_impulse.Z, 0.0);
            }
            else if (_limitState == LimitState.AtUpper)
            {
                _impulse.Z = Math.Min(_impulse.Z, 0.0);
            }

            // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
            Vector2 b = -Cdot1 - (_impulse.Z - f1.Z) * new Vector2(_K.Ez.X, _K.Ez.Y);
            Vector2 f2r = _K.Solve2x2(b) + new Vector2(f1.X, f1.Y);
            _impulse.X = f2r.X;
            _impulse.Y = f2r.Y;

            df = _impulse - f1;

            Vector2 P = df.X * _perp + df.Z * _axis;
            double LA = df.X * _s1 + df.Y + df.Z * _a1;
            double LB = df.X * _s2 + df.Y + df.Z * _a2;

            vA -= mA * P;
            wA -= iA * LA;

            vB += mB * P;
            wB += iB * LB;
        }
        else
        {
            // Limit is inactive, just solve the prismatic constraint in block form.
            Vector2 df = _K.Solve2x2(-Cdot1);
            _impulse.X += df.X;
            _impulse.Y += df.Y;

            Vector2 P = df.X * _perp;
            double LA = df.X * _s1 + df.Y;
            double LB = df.X * _s2 + df.Y;

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

        double mA = _invMassA, mB = _invMassB;
        double iA = _invIA, iB = _invIB;

        // Compute fresh Jacobians
        Vector2 rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
        Vector2 rB = Complex.Multiply(LocalAnchorB - _localCenterB, ref qB);
        Vector2 d = cB + rB - cA - rA;

        Vector2 axis = Complex.Multiply(ref _localXAxis, ref qA);
        double a1 = MathUtils.Cross(d + rA, axis);
        double a2 = MathUtils.Cross(ref rB, ref axis);
        Vector2 perp = Complex.Multiply(ref _localYAxisA, ref qA);

        double s1 = MathUtils.Cross(d + rA, perp);
        double s2 = MathUtils.Cross(ref rB, ref perp);

        Vector3 impulse;
        Vector2 C1 = new Vector2();
        C1.X = Vector2.Dot(perp, d);
        C1.Y = aB - aA - ReferenceAngle;

        double linearError = Math.Abs(C1.X);
        double angularError = Math.Abs(C1.Y);

        bool active = false;
        double C2 = 0.0;
        if (_enableLimit)
        {
            double translation = Vector2.Dot(axis, d);
            if (Math.Abs(_upperTranslation - _lowerTranslation) < 2.0 * Settings.LinearSlop)
            {
                // Prevent large angular corrections
                C2 = MathUtils.Clamp(translation, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);
                linearError = Math.Max(linearError, Math.Abs(translation));
                active = true;
            }
            else if (translation <= _lowerTranslation)
            {
                // Prevent large linear corrections and allow some slop.
                C2 = MathUtils.Clamp(translation - _lowerTranslation + Settings.LinearSlop, -Settings.MaxLinearCorrection, 0.0);
                linearError = Math.Max(linearError, _lowerTranslation - translation);
                active = true;
            }
            else if (translation >= _upperTranslation)
            {
                // Prevent large linear corrections and allow some slop.
                C2 = MathUtils.Clamp(translation - _upperTranslation - Settings.LinearSlop, 0.0, Settings.MaxLinearCorrection);
                linearError = Math.Max(linearError, translation - _upperTranslation);
                active = true;
            }
        }

        if (active)
        {
            double k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            double k12 = iA * s1 + iB * s2;
            double k13 = iA * s1 * a1 + iB * s2 * a2;
            double k22 = iA + iB;
            if (k22 == 0.0)
            {
                // For fixed rotation
                k22 = 1.0;
            }
            double k23 = iA * a1 + iB * a2;
            double k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

            Matrix3x3 K = new()
            {
                Ex = new Vector3(k11, k12, k13),
                Ey = new Vector3(k12, k22, k23),
                Ez = new Vector3(k13, k23, k33)
            };

            Vector3 C = new(C1.X, C1.Y, C2);

            impulse = K.Solve3x3(-C);
        }
        else
        {
            double k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            double k12 = iA * s1 + iB * s2;
            double k22 = iA + iB;
            if (k22 == 0.0)
            {
                k22 = 1.0;
            }

            Matrix2x2 K = new()
            {
                Ex = new Vector2(k11, k12),
                Ey = new Vector2(k12, k22)
            };

            Vector2 impulse1 = K.Solve(-C1);
            impulse = new Vector3(impulse1.X, impulse1.Y, 0.0);
        }

        Vector2 P = impulse.X * perp + impulse.Z * axis;
        double LA = impulse.X * s1 + impulse.Y + impulse.Z * a1;
        double LB = impulse.X * s2 + impulse.Y + impulse.Z * a2;

        cA -= mA * P;
        aA -= iA * LA;
        cB += mB * P;
        aB += iB * LB;

        data.positions[_indexA].c = cA;
        data.positions[_indexA].a = aA;
        data.positions[_indexB].c = cB;
        data.positions[_indexB].a = aB;

        return linearError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
    }
}
