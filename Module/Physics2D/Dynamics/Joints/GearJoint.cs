using System.Diagnostics;
using StdbModule.Common;

namespace StdbModule.Physics2D.Dynamics.Joints;

// Gear Joint:
// C0 = (coordinate1 + ratio * coordinate2)_initial
// C = (coordinate1 + ratio * coordinate2) - C0 = 0
// J = [J1 ratio * J2]
// K = J * invM * JT
//   = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
// Revolute:
// coordinate = rotation
// Cdot = angularVelocity
// J = [0 0 1]
// K = J * invM * JT = invI
//
// Prismatic:
// coordinate = dot(p - pg, ug)
// Cdot = dot(v + cross(w, r), ug)
// J = [ug cross(r, ug)]
// K = J * invM * JT = invMass + invI * cross(r, ug)^2

/// <summary>
/// A gear joint is used to connect two joints together.
/// Either joint can be a revolute or prismatic joint.
/// You specify a gear ratio to bind the motions together:
/// <![CDATA[coordinate1 + ratio * coordinate2 = ant]]>
/// The ratio can be negative or positive. If one joint is a revolute joint
/// and the other joint is a prismatic joint, then the ratio will have units
/// of length or units of 1/length.
///
/// Warning: You have to manually destroy the gear joint if jointA or jointB is destroyed.
/// </summary>
public class GearJoint : Joint
{
    private JointType _typeA;
    private JointType _typeB;

    private Body _bodyA;
    private Body _bodyB;
    private Body _bodyC;
    private Body _bodyD;

    // Solver shared
    private Vector2 _localAnchorA;
    private Vector2 _localAnchorB;
    private Vector2 _localAnchorC;
    private Vector2 _localAnchorD;

    private Vector2 _localAxisC;
    private Vector2 _localAxisD;

    private double _referenceAngleA;
    private double _referenceAngleB;

    private double _constant;
    private double _ratio;

    private double _impulse;

    // Solver temp
    private int _indexA, _indexB, _indexC, _indexD;
    private Vector2 _lcA, _lcB, _lcC, _lcD;
    private double _mA, _mB, _mC, _mD;
    private double _iA, _iB, _iC, _iD;
    private Vector2 _JvAC, _JvBD;
    private double _JwA, _JwB, _JwC, _JwD;
    private double _mass;

    /// <summary>
    /// Requires two existing revolute or prismatic joints (any combination will work).
    /// The provided joints must attach a dynamic body to a static body.
    /// </summary>
    /// <param name="jointA">The first joint.</param>
    /// <param name="jointB">The second joint.</param>
    /// <param name="ratio">The ratio.</param>
    /// <param name="bodyA">The first body</param>
    /// <param name="bodyB">The second body</param>
    public GearJoint(Body bodyA, Body bodyB, Joint jointA, Joint jointB, double ratio = 1)
    {
        JointType = JointType.Gear;
        BodyA = bodyA;
        BodyB = bodyB;
        JointA = jointA;
        JointB = jointB;
        Ratio = ratio;

        _typeA = jointA.JointType;
        _typeB = jointB.JointType;

        Debug.Assert(_typeA == JointType.Revolute || _typeA == JointType.Prismatic || _typeA == JointType.FixedRevolute || _typeA == JointType.FixedPrismatic);
        Debug.Assert(_typeB == JointType.Revolute || _typeB == JointType.Prismatic || _typeB == JointType.FixedRevolute || _typeB == JointType.FixedPrismatic);

        double coordinateA, coordinateB;

        // TODO_ERIN there might be some problem with the joint edges in b2Joint.

        _bodyC = JointA.BodyA;
        _bodyA = JointA.BodyB;

        // Get geometry of joint1
        Transform xfA = _bodyA._xf;
        double aA = _bodyA._sweep.A;
        Transform xfC = _bodyC._xf;
        double aC = _bodyC._sweep.A;

        if (_typeA == JointType.Revolute)
        {
            RevoluteJoint revolute = (RevoluteJoint)jointA;
            _localAnchorC = revolute.LocalAnchorA;
            _localAnchorA = revolute.LocalAnchorB;
            _referenceAngleA = revolute.ReferenceAngle;
            _localAxisC = Vector2.Zero;

            coordinateA = aA - aC - _referenceAngleA;
        }
        else
        {
            PrismaticJoint prismatic = (PrismaticJoint)jointA;
            _localAnchorC = prismatic.LocalAnchorA;
            _localAnchorA = prismatic.LocalAnchorB;
            _referenceAngleA = prismatic.ReferenceAngle;
            _localAxisC = prismatic.LocalXAxis;

            Vector2 pC = _localAnchorC;
            Vector2 pA = Complex.Divide(Complex.Multiply(ref _localAnchorA, ref xfA.Rotation) + (xfA.Position - xfC.Position), ref xfC.Rotation);
            coordinateA = Vector2.Dot(pA - pC, _localAxisC);
        }

        _bodyD = JointB.BodyA;
        _bodyB = JointB.BodyB;

        // Get geometry of joint2
        Transform xfB = _bodyB._xf;
        double aB = _bodyB._sweep.A;
        Transform xfD = _bodyD._xf;
        double aD = _bodyD._sweep.A;

        if (_typeB == JointType.Revolute)
        {
            RevoluteJoint revolute = (RevoluteJoint)jointB;
            _localAnchorD = revolute.LocalAnchorA;
            _localAnchorB = revolute.LocalAnchorB;
            _referenceAngleB = revolute.ReferenceAngle;
            _localAxisD = Vector2.Zero;

            coordinateB = aB - aD - _referenceAngleB;
        }
        else
        {
            PrismaticJoint prismatic = (PrismaticJoint)jointB;
            _localAnchorD = prismatic.LocalAnchorA;
            _localAnchorB = prismatic.LocalAnchorB;
            _referenceAngleB = prismatic.ReferenceAngle;
            _localAxisD = prismatic.LocalXAxis;

            Vector2 pD = _localAnchorD;
            Vector2 pB = Complex.Divide(Complex.Multiply(ref _localAnchorB, ref xfB.Rotation) + (xfB.Position - xfD.Position), ref xfD.Rotation);
            coordinateB = Vector2.Dot(pB - pD, _localAxisD);
        }

        _ratio = ratio;
        _constant = coordinateA + _ratio * coordinateB;
        _impulse = 0.0;
    }

    public override Vector2 WorldAnchorA
    {
        get { return _bodyA.GetWorldPoint(_localAnchorA); }
        set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
    }

    public override Vector2 WorldAnchorB
    {
        get { return _bodyB.GetWorldPoint(_localAnchorB); }
        set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
    }

    /// <summary>
    /// The gear ratio.
    /// </summary>
    public double Ratio
    {
        get { return _ratio; }
        set
        {
            Debug.Assert(MathUtils.IsValid(value));
            _ratio = value;
        }
    }

    /// <summary>
    /// The first revolute/prismatic joint attached to the gear joint.
    /// </summary>
    public Joint JointA { get; private set; }

    /// <summary>
    /// The second revolute/prismatic joint attached to the gear joint.
    /// </summary>
    public Joint JointB { get; private set; }

    public override Vector2 GetReactionForce(double invDt)
    {
        Vector2 P = _impulse * _JvAC;
        return invDt * P;
    }

    public override double GetReactionTorque(double invDt)
    {
        double L = _impulse * _JwA;
        return invDt * L;
    }

    internal override void InitVelocityConstraints(ref SolverData data)
    {
        _indexA = _bodyA.IslandIndex;
        _indexB = _bodyB.IslandIndex;
        _indexC = _bodyC.IslandIndex;
        _indexD = _bodyD.IslandIndex;
        _lcA = _bodyA._sweep.LocalCenter;
        _lcB = _bodyB._sweep.LocalCenter;
        _lcC = _bodyC._sweep.LocalCenter;
        _lcD = _bodyD._sweep.LocalCenter;
        _mA = _bodyA._invMass;
        _mB = _bodyB._invMass;
        _mC = _bodyC._invMass;
        _mD = _bodyD._invMass;
        _iA = _bodyA._invI;
        _iB = _bodyB._invI;
        _iC = _bodyC._invI;
        _iD = _bodyD._invI;

        double aA = data.positions[_indexA].a;
        Vector2 vA = data.velocities[_indexA].v;
        double wA = data.velocities[_indexA].w;

        double aB = data.positions[_indexB].a;
        Vector2 vB = data.velocities[_indexB].v;
        double wB = data.velocities[_indexB].w;

        double aC = data.positions[_indexC].a;
        Vector2 vC = data.velocities[_indexC].v;
        double wC = data.velocities[_indexC].w;

        double aD = data.positions[_indexD].a;
        Vector2 vD = data.velocities[_indexD].v;
        double wD = data.velocities[_indexD].w;

        Complex qA = Complex.FromAngle(aA);
        Complex qB = Complex.FromAngle(aB);
        Complex qC = Complex.FromAngle(aC);
        Complex qD = Complex.FromAngle(aD);

        _mass = 0.0;

        if (_typeA == JointType.Revolute)
        {
            _JvAC = Vector2.Zero;
            _JwA = 1.0;
            _JwC = 1.0;
            _mass += _iA + _iC;
        }
        else
        {
            Vector2 u = Complex.Multiply(ref _localAxisC, ref qC);
            Vector2 rC = Complex.Multiply(_localAnchorC - _lcC, ref qC);
            Vector2 rA = Complex.Multiply(_localAnchorA - _lcA, ref qA);
            _JvAC = u;
            _JwC = MathUtils.Cross(ref rC, ref u);
            _JwA = MathUtils.Cross(ref rA, ref u);
            _mass += _mC + _mA + _iC * _JwC * _JwC + _iA * _JwA * _JwA;
        }

        if (_typeB == JointType.Revolute)
        {
            _JvBD = Vector2.Zero;
            _JwB = _ratio;
            _JwD = _ratio;
            _mass += _ratio * _ratio * (_iB + _iD);
        }
        else
        {
            Vector2 u = Complex.Multiply(ref _localAxisD, ref qD);
            Vector2 rD = Complex.Multiply(_localAnchorD - _lcD, ref qD);
            Vector2 rB = Complex.Multiply(_localAnchorB - _lcB, ref qB);
            _JvBD = _ratio * u;
            _JwD = _ratio * MathUtils.Cross(ref rD, ref u);
            _JwB = _ratio * MathUtils.Cross(ref rB, ref u);
            _mass += _ratio * _ratio * (_mD + _mB) + _iD * _JwD * _JwD + _iB * _JwB * _JwB;
        }

        // Compute effective mass.
        _mass = _mass > 0.0 ? 1.0 / _mass : 0.0;

        if (data.step.warmStarting)
        {
            vA += _mA * _impulse * _JvAC;
            wA += _iA * _impulse * _JwA;
            vB += _mB * _impulse * _JvBD;
            wB += _iB * _impulse * _JwB;
            vC -= _mC * _impulse * _JvAC;
            wC -= _iC * _impulse * _JwC;
            vD -= _mD * _impulse * _JvBD;
            wD -= _iD * _impulse * _JwD;
        }
        else
        {
            _impulse = 0.0;
        }

        data.velocities[_indexA].v = vA;
        data.velocities[_indexA].w = wA;
        data.velocities[_indexB].v = vB;
        data.velocities[_indexB].w = wB;
        data.velocities[_indexC].v = vC;
        data.velocities[_indexC].w = wC;
        data.velocities[_indexD].v = vD;
        data.velocities[_indexD].w = wD;
    }

    internal override void SolveVelocityConstraints(ref SolverData data)
    {
        Vector2 vA = data.velocities[_indexA].v;
        double wA = data.velocities[_indexA].w;
        Vector2 vB = data.velocities[_indexB].v;
        double wB = data.velocities[_indexB].w;
        Vector2 vC = data.velocities[_indexC].v;
        double wC = data.velocities[_indexC].w;
        Vector2 vD = data.velocities[_indexD].v;
        double wD = data.velocities[_indexD].w;

        double Cdot = Vector2.Dot(_JvAC, vA - vC) + Vector2.Dot(_JvBD, vB - vD);
        Cdot += _JwA * wA - _JwC * wC + (_JwB * wB - _JwD * wD);

        double impulse = -_mass * Cdot;
        _impulse += impulse;

        vA += _mA * impulse * _JvAC;
        wA += _iA * impulse * _JwA;
        vB += _mB * impulse * _JvBD;
        wB += _iB * impulse * _JwB;
        vC -= _mC * impulse * _JvAC;
        wC -= _iC * impulse * _JwC;
        vD -= _mD * impulse * _JvBD;
        wD -= _iD * impulse * _JwD;

        data.velocities[_indexA].v = vA;
        data.velocities[_indexA].w = wA;
        data.velocities[_indexB].v = vB;
        data.velocities[_indexB].w = wB;
        data.velocities[_indexC].v = vC;
        data.velocities[_indexC].w = wC;
        data.velocities[_indexD].v = vD;
        data.velocities[_indexD].w = wD;
    }

    internal override bool SolvePositionConstraints(ref SolverData data)
    {
        Vector2 cA = data.positions[_indexA].c;
        double aA = data.positions[_indexA].a;
        Vector2 cB = data.positions[_indexB].c;
        double aB = data.positions[_indexB].a;
        Vector2 cC = data.positions[_indexC].c;
        double aC = data.positions[_indexC].a;
        Vector2 cD = data.positions[_indexD].c;
        double aD = data.positions[_indexD].a;

        Complex qA = Complex.FromAngle(aA);
        Complex qB = Complex.FromAngle(aB);
        Complex qC = Complex.FromAngle(aC);
        Complex qD = Complex.FromAngle(aD);

        const double linearError = 0.0;

        double coordinateA, coordinateB;

        Vector2 JvAC, JvBD;
        double JwA, JwB, JwC, JwD;
        double mass = 0.0;

        if (_typeA == JointType.Revolute)
        {
            JvAC = Vector2.Zero;
            JwA = 1.0;
            JwC = 1.0;
            mass += _iA + _iC;

            coordinateA = aA - aC - _referenceAngleA;
        }
        else
        {
            Vector2 u = Complex.Multiply(ref _localAxisC, ref qC);
            Vector2 rC = Complex.Multiply(_localAnchorC - _lcC, ref qC);
            Vector2 rA = Complex.Multiply(_localAnchorA - _lcA, ref qA);
            JvAC = u;
            JwC = MathUtils.Cross(ref rC, ref u);
            JwA = MathUtils.Cross(ref rA, ref u);
            mass += _mC + _mA + _iC * JwC * JwC + _iA * JwA * JwA;

            Vector2 pC = _localAnchorC - _lcC;
            Vector2 pA = Complex.Divide(rA + (cA - cC), ref qC);
            coordinateA = Vector2.Dot(pA - pC, _localAxisC);
        }

        if (_typeB == JointType.Revolute)
        {
            JvBD = Vector2.Zero;
            JwB = _ratio;
            JwD = _ratio;
            mass += _ratio * _ratio * (_iB + _iD);

            coordinateB = aB - aD - _referenceAngleB;
        }
        else
        {
            Vector2 u = Complex.Multiply(ref _localAxisD, ref qD);
            Vector2 rD = Complex.Multiply(_localAnchorD - _lcD, ref qD);
            Vector2 rB = Complex.Multiply(_localAnchorB - _lcB, ref qB);
            JvBD = _ratio * u;
            JwD = _ratio * MathUtils.Cross(ref rD, ref u);
            JwB = _ratio * MathUtils.Cross(ref rB, ref u);
            mass += _ratio * _ratio * (_mD + _mB) + _iD * JwD * JwD + _iB * JwB * JwB;

            Vector2 pD = _localAnchorD - _lcD;
            Vector2 pB = Complex.Divide(rB + (cB - cD), ref qD);
            coordinateB = Vector2.Dot(pB - pD, _localAxisD);
        }

        double C = coordinateA + _ratio * coordinateB - _constant;

        double impulse = 0.0;
        if (mass > 0.0)
        {
            impulse = -C / mass;
        }

        cA += _mA * impulse * JvAC;
        aA += _iA * impulse * JwA;
        cB += _mB * impulse * JvBD;
        aB += _iB * impulse * JwB;
        cC -= _mC * impulse * JvAC;
        aC -= _iC * impulse * JwC;
        cD -= _mD * impulse * JvBD;
        aD -= _iD * impulse * JwD;

        data.positions[_indexA].c = cA;
        data.positions[_indexA].a = aA;
        data.positions[_indexB].c = cB;
        data.positions[_indexB].a = aB;
        data.positions[_indexC].c = cC;
        data.positions[_indexC].a = aC;
        data.positions[_indexD].c = cD;
        data.positions[_indexD].a = aD;

        // TODO: not implemented
        return linearError < Settings.LinearSlop;
    }
}
