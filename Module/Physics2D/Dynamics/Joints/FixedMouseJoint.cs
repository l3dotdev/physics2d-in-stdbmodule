using System.Diagnostics;
using StdbModule.Common;

namespace StdbModule.Physics2D.Dynamics.Joints;

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

/// <summary>
/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft constraint with a maximum
/// force. This allows the constraint to stretch and without
/// applying huge forces.
/// NOTE: this joint is not documented in the manual because it was
/// developed to be used in the testbed. If you want to learn how to
/// use the mouse joint, look at the testbed.
/// </summary>
public class FixedMouseJoint : Joint
{
    private Vector2 _worldAnchor;
    private double _frequency;
    private double _dampingRatio;
    private double _beta;

    // Solver shared
    private Vector2 _impulse;
    private double _maxForce;
    private double _gamma;

    // Solver temp
    private int _indexA;
    private Vector2 _rA;
    private Vector2 _localCenterA;
    private double _invMassA;
    private double _invIA;
    private Matrix2x2 _mass;
    private Vector2 _C;

    /// <summary>
    /// This requires a world target point,
    /// tuning parameters, and the time step.
    /// </summary>
    /// <param name="body">The body.</param>
    /// <param name="worldAnchor">The target.</param>
    public FixedMouseJoint(Body body, Vector2 worldAnchor)
        : base(body)
    {
        JointType = JointType.FixedMouse;
        Frequency = 5.0;
        DampingRatio = 0.7;
        MaxForce = 1000 * body.Mass;

        Debug.Assert(worldAnchor.IsValid());

        _worldAnchor = worldAnchor;
        LocalAnchorA = Transform.Divide(ref worldAnchor, ref BodyA._xf);
    }

    /// <summary>
    /// The local anchor point on BodyA
    /// </summary>
    public Vector2 LocalAnchorA { get; set; }

    public override Vector2 WorldAnchorA
    {
        get { return BodyA.GetWorldPoint(LocalAnchorA); }
        set { LocalAnchorA = BodyA.GetLocalPoint(value); }
    }

    public override Vector2 WorldAnchorB
    {
        get { return _worldAnchor; }
        set
        {
            WakeBodies();
            _worldAnchor = value;
        }
    }

    /// <summary>
    /// The maximum constraint force that can be exerted
    /// to move the candidate body. Usually you will express
    /// as some multiple of the weight (multiplier * mass * gravity).
    /// </summary>
    public double MaxForce
    {
        get { return _maxForce; }
        set
        {
            Debug.Assert(MathUtils.IsValid(value) && value >= 0.0);
            _maxForce = value;
        }
    }

    /// <summary>
    /// The response speed.
    /// </summary>
    public double Frequency
    {
        get { return _frequency; }
        set
        {
            Debug.Assert(MathUtils.IsValid(value) && value >= 0.0);
            _frequency = value;
        }
    }

    /// <summary>
    /// The damping ratio. 0 = no damping, 1 = critical damping.
    /// </summary>
    public double DampingRatio
    {
        get { return _dampingRatio; }
        set
        {
            Debug.Assert(MathUtils.IsValid(value) && value >= 0.0);
            _dampingRatio = value;
        }
    }

    public override Vector2 GetReactionForce(double invDt)
    {
        return invDt * _impulse;
    }

    public override double GetReactionTorque(double invDt)
    {
        return invDt * 0.0;
    }

    internal override void InitVelocityConstraints(ref SolverData data)
    {
        _indexA = BodyA.IslandIndex;
        _localCenterA = BodyA._sweep.LocalCenter;
        _invMassA = BodyA._invMass;
        _invIA = BodyA._invI;

        Vector2 cA = data.positions[_indexA].c;
        double aA = data.positions[_indexA].a;
        Vector2 vA = data.velocities[_indexA].v;
        double wA = data.velocities[_indexA].w;

        Complex qA = Complex.FromAngle(aA);

        double mass = BodyA.Mass;

        // Frequency
        double omega = MathUtils.Tau * Frequency;

        // Damping coefficient
        double d = 2.0 * mass * DampingRatio * omega;

        // Spring stiffness
        double k = mass * (omega * omega);

        // magic formulas
        // gamma has units of inverse mass.
        // beta has units of inverse time.
        double h = data.step.dt;
        Debug.Assert(d + h * k > MathUtils.Epsilon);
        _gamma = h * (d + h * k);
        if (_gamma != 0.0)
        {
            _gamma = 1.0 / _gamma;
        }

        _beta = h * k * _gamma;

        // Compute the effective mass matrix.
        _rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
        // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        //      = [1/m1+1/m2     0    ] + invI1 * [r1.Y*r1.Y -r1.X*r1.Y] + invI2 * [r1.Y*r1.Y -r1.X*r1.Y]
        //        [    0     1/m1+1/m2]           [-r1.X*r1.Y r1.X*r1.X]           [-r1.X*r1.Y r1.X*r1.X]
        Matrix2x2 K = new();
        K.Ex.X = _invMassA + _invIA * _rA.Y * _rA.Y + _gamma;
        K.Ex.Y = -_invIA * _rA.X * _rA.Y;
        K.Ey.X = K.Ex.Y;
        K.Ey.Y = _invMassA + _invIA * _rA.X * _rA.X + _gamma;

        _mass = K.Inverse;

        _C = cA + _rA - _worldAnchor;
        _C *= _beta;

        // Cheat with some damping
        wA *= 0.98;

        if (data.step.warmStarting)
        {
            _impulse *= data.step.dtRatio;
            vA += _invMassA * _impulse;
            wA += _invIA * MathUtils.Cross(ref _rA, ref _impulse);
        }
        else
        {
            _impulse = Vector2.Zero;
        }

        data.velocities[_indexA].v = vA;
        data.velocities[_indexA].w = wA;
    }

    internal override void SolveVelocityConstraints(ref SolverData data)
    {
        Vector2 vA = data.velocities[_indexA].v;
        double wA = data.velocities[_indexA].w;

        // Cdot = v + cross(w, r)
        Vector2 Cdot = vA + MathUtils.Cross(wA, ref _rA);
        Vector2 impulse = MathUtils.Mul(ref _mass, -(Cdot + _C + _gamma * _impulse));

        Vector2 oldImpulse = _impulse;
        _impulse += impulse;
        double maxImpulse = data.step.dt * MaxForce;
        if (_impulse.LengthSquared > maxImpulse * maxImpulse)
        {
            _impulse *= maxImpulse / _impulse.Length;
        }
        impulse = _impulse - oldImpulse;

        vA += _invMassA * impulse;
        wA += _invIA * MathUtils.Cross(ref _rA, ref impulse);

        data.velocities[_indexA].v = vA;
        data.velocities[_indexA].w = wA;
    }

    internal override bool SolvePositionConstraints(ref SolverData data)
    {
        return true;
    }
}