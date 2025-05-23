using System.Diagnostics;
using StdbModule.Common;
using StdbModule.Physics2D.Collision;
using StdbModule.Physics2D.Collision.Shapes;
using StdbModule.Physics2D.Common.Logic;
using StdbModule.Physics2D.Dynamics.Contacts;
using StdbModule.Physics2D.Dynamics.Joints;

namespace StdbModule.Physics2D.Dynamics;

public partial class Body
{
    private double _angularDamping;
    private BodyType _bodyType;
    private double _inertia;
    private double _linearDamping;
    private double _mass;
    private bool _sleepingAllowed;
    private bool _awake;
    private bool _fixedRotation;

    internal bool _enabled;
    internal double _angularVelocity;
    internal Vector2 _linearVelocity;
    internal Vector2 _force;
    internal double _invI;
    internal double _invMass;
    internal double _sleepTime;
    internal Sweep _sweep; // the swept motion for CCD
    internal double _torque;
    internal World _world;
    internal Transform _xf; // the body origin transform
    internal bool _island;
    internal int _lock;
    internal int _lockOrder;

    public ControllerFilter ControllerFilter = new(ControllerCategory.All);

    public Body()
    {
        FixtureList = new FixtureCollection(this);

        _enabled = true;
        _awake = true;
        _sleepingAllowed = true;
        _xf.Rotation = Complex.One;

        BodyType = BodyType.Static;
    }

    /// <summary>
    /// Get the parent World of this body. This is null if the body is not attached.
    /// </summary>
    public World World { get { return _world; } }

    /// <remarks>Deprecated in version 1.6</remarks>
    // [Obsolete]
    public int IslandIndex { get; internal set; }

    /// <summary>
    /// Set the user data. Use this to store your application specific data.
    /// </summary>
    /// <value>The user data.</value>
    public object Tag;

    /// <summary>
    /// Gets the total number revolutions the body has made.
    /// </summary>
    /// <value>The revolutions.</value>
    public double Revolutions
    {
        get { return Rotation / (2 * Math.PI); }
    }

    /// <summary>
    /// Gets or sets the body type.
    /// Warning: This property is readonly during callbacks.
    /// </summary>
    /// <value>The type of body.</value>
    /// <exception cref="System.InvalidOperationException">Thrown when the world is Locked/Stepping.</exception>
    public BodyType BodyType
    {
        get { return _bodyType; }
        set
        {
            if (World != null && World.IsLocked)
                throw new InvalidOperationException("The World is locked.");

            if (_bodyType == value)
                return;

            _bodyType = value;

            ResetMassData();

            if (_bodyType == BodyType.Static)
            {
                _linearVelocity = Vector2.Zero;
                _angularVelocity = 0.0;
                _sweep.A0 = _sweep.A;
                _sweep.C0 = _sweep.C;
                SynchronizeFixtures();
            }

            Awake = true;

            _force = Vector2.Zero;
            _torque = 0.0;

            // Delete the attached contacts.
            ContactEdge ce = ContactList;
            while (ce != null)
            {
                ContactEdge ce0 = ce;
                ce = ce.Next;
                World.ContactManager.Destroy(ce0.Contact);
            }
            ContactList = null;

            if (World != null)
            {
                // Touch the proxies so that new contacts will be created (when appropriate)
                IBroadPhase broadPhase = World.ContactManager.BroadPhase;
                foreach (Fixture fixture in FixtureList)
                    fixture.TouchProxies(broadPhase);
            }
        }
    }

    /// <summary>
    /// Get or sets the linear velocity of the center of mass. Property has no effect on <see cref="BodyType.Static"/> bodies.
    /// </summary>
    /// <value>The linear velocity.</value>
    public Vector2 LinearVelocity
    {
        set
        {
            Debug.Assert(!double.IsNaN(value.X) && !double.IsNaN(value.Y));

            if (_bodyType == BodyType.Static)
                return;

            if (Vector2.Dot(value, value) > 0.0)
                Awake = true;

            _linearVelocity = value;
        }
        get { return _linearVelocity; }
    }

    /// <summary>
    /// Gets or sets the angular velocity. Radians/second.
    /// </summary>
    /// <value>The angular velocity.</value>
    public double AngularVelocity
    {
        set
        {
            Debug.Assert(!double.IsNaN(value));

            if (_bodyType == BodyType.Static)
                return;

            if (value * value > 0.0)
                Awake = true;

            _angularVelocity = value;
        }
        get { return _angularVelocity; }
    }

    /// <summary>
    /// Gets or sets the linear damping.
    /// </summary>
    /// <value>The linear damping.</value>
    public double LinearDamping
    {
        get { return _linearDamping; }
        set
        {
            Debug.Assert(!double.IsNaN(value));

            _linearDamping = value;
        }
    }

    /// <summary>
    /// Gets or sets the angular damping.
    /// </summary>
    /// <value>The angular damping.</value>
    public double AngularDamping
    {
        get { return _angularDamping; }
        set
        {
            Debug.Assert(!double.IsNaN(value));

            _angularDamping = value;
        }
    }

    /// <summary>
    /// Gets or sets a value indicating whether this body should be included in the CCD solver.
    /// </summary>
    /// <value><c>true</c> if this instance is included in CCD; otherwise, <c>false</c>.</value>
    public bool IsBullet { get; set; }

    /// <summary>
    /// You can disable sleeping on this body. If you disable sleeping, the
    /// body will be woken.
    /// </summary>
    /// <value><c>true</c> if sleeping is allowed; otherwise, <c>false</c>.</value>
    public bool SleepingAllowed
    {
        set
        {
            if (!value)
                Awake = true;

            _sleepingAllowed = value;
        }
        get { return _sleepingAllowed; }
    }

    /// <summary>
    /// Set the sleep state of the body. A sleeping body has very
    /// low CPU cost.
    /// </summary>
    /// <value><c>true</c> if awake; otherwise, <c>false</c>.</value>
    public bool Awake
    {
        set
        {
            if (value)
            {
                if (!_awake)
                {
                    _sleepTime = 0.0;

#if USE_ACTIVE_CONTACT_SET
                    World.ContactManager.UpdateActiveContacts(ContactList, true);
#endif

#if USE_AWAKE_BODY_SET
                    if (InWorld && !World.AwakeBodySet.Contains(this))
                        World.AwakeBodySet.Add(this);
#endif
                }
            }
            else
            {
#if USE_AWAKE_BODY_SET
                // Check even for BodyType.Static because if this body had just been changed to Static it will have
                // set Awake = false in the process.
                if (InWorld && World.AwakeBodySet.Contains(this))
                    World.AwakeBodySet.Remove(this);
#endif
                ResetDynamics();
                _sleepTime = 0.0;

#if USE_ACTIVE_CONTACT_SET
                World.ContactManager.UpdateActiveContacts(ContactList, false);
#endif
            }

            _awake = value;
        }
        get { return _awake; }
    }

    /// <summary>
    /// Set the active state of the body. An inactive body is not
    /// simulated and cannot be collided with or woken up.
    /// If you pass a flag of true, all fixtures will be added to the
    /// broad-phase.
    /// If you pass a flag of false, all fixtures will be removed from
    /// the broad-phase and all contacts will be destroyed.
    /// Fixtures and joints are otherwise unaffected. You may continue
    /// to create/destroy fixtures and joints on inactive bodies.
    /// Fixtures on an inactive body are implicitly inactive and will
    /// not participate in collisions, ray-casts, or queries.
    /// Joints connected to an inactive body are implicitly inactive.
    /// An inactive body is still owned by a b2World object and remains
    /// in the body list.
    /// Warning: This property is readonly during callbacks.
    /// </summary>
    /// <value><c>true</c> if active; otherwise, <c>false</c>.</value>
    /// <exception cref="System.InvalidOperationException">Thrown when the world is Locked/Stepping.</exception>
    public bool Enabled
    {
        get { return _enabled; }
        set
        {
            if (World != null && World.IsLocked)
                throw new InvalidOperationException("The World is locked.");

            if (value == _enabled)
                return;

            _enabled = value;

            if (Enabled)
            {
                if (World != null)
                    CreateProxies();

                // Contacts are created the next time step.
            }
            else
            {
                if (World != null)
                {
                    DestroyProxies();
                    DestroyContacts();
                }
            }
        }
    }

    /// <summary>
    /// Create all proxies.
    /// </summary>
    internal void CreateProxies()
    {
        IBroadPhase broadPhase = World.ContactManager.BroadPhase;
        for (int i = 0; i < FixtureList._list.Count; i++)
            FixtureList._list[i].CreateProxies(broadPhase, ref _xf);
    }

    /// <summary>
    /// Destroy all proxies.
    /// </summary>
    internal void DestroyProxies()
    {
        IBroadPhase broadPhase = World.ContactManager.BroadPhase;
        for (int i = 0; i < FixtureList._list.Count; i++)
            FixtureList._list[i].DestroyProxies(broadPhase);
    }

    /// <summary>
    /// Destroy the attached contacts.
    /// </summary>
    private void DestroyContacts()
    {
        ContactEdge ce = ContactList;
        while (ce != null)
        {
            ContactEdge ce0 = ce;
            ce = ce.Next;
            World.ContactManager.Destroy(ce0.Contact);
        }
        ContactList = null;
    }


    /// <summary>
    /// Set this body to have fixed rotation. This causes the mass
    /// to be reset.
    /// </summary>
    /// <value><c>true</c> if it has fixed rotation; otherwise, <c>false</c>.</value>
    public bool FixedRotation
    {
        set
        {
            if (_fixedRotation == value)
                return;

            _fixedRotation = value;

            _angularVelocity = 0;
            ResetMassData();
        }
        get { return _fixedRotation; }
    }

    /// <summary>
    /// Gets all the fixtures attached to this body.
    /// </summary>
    /// <value>The fixture list.</value>
    public readonly FixtureCollection FixtureList;

    /// <summary>
    /// Get the list of all joints attached to this body.
    /// </summary>
    /// <value>The joint list.</value>
    public JointEdge JointList { get; internal set; }

    /// <summary>
    /// Get the list of all contacts attached to this body.
    /// Warning: this list changes during the time step and you may
    /// miss some collisions if you don't use callback events.
    /// </summary>
    /// <value>The contact list.</value>
    public ContactEdge ContactList { get; internal set; }

    /// <summary>
    /// Get the world body origin position.
    /// </summary>
    /// <returns>Return the world position of the body's origin.</returns>
    public Vector2 Position
    {
        get { return _xf.Position; }
        set
        {
            Debug.Assert(!double.IsNaN(value.X) && !double.IsNaN(value.Y));

            if (World == null)
                _xf.Position = value;
            else
                SetTransform(ref value, Rotation);
        }
    }

    /// <summary>
    /// Get the angle in radians.
    /// </summary>
    /// <returns>Return the current world rotation angle in radians.</returns>
    public double Rotation
    {
        get { return _sweep.A; }
        set
        {
            Debug.Assert(!double.IsNaN(value));

            if (World == null)
                _sweep.A = value;
            else
                SetTransform(ref _xf.Position, value);
        }
    }

    /// <summary>
    /// Gets or sets a value indicating whether this body ignores gravity.
    /// </summary>
    /// <value><c>true</c> if  it ignores gravity; otherwise, <c>false</c>.</value>
    public bool IgnoreGravity { get; set; }

    /// <summary>
    /// Get the world position of the center of mass.
    /// </summary>
    /// <value>The world position.</value>
    public Vector2 WorldCenter
    {
        get { return _sweep.C; }
    }

    /// <summary>
    /// Get the local position of the center of mass.
    /// Warning: This property is readonly during callbacks.
    /// </summary>
    /// <value>The local position.</value>
    /// <exception cref="System.InvalidOperationException">Thrown when the world is Locked/Stepping.</exception>
    public Vector2 LocalCenter
    {
        get { return _sweep.LocalCenter; }
        set
        {
            if (World != null && World.IsLocked)
                throw new InvalidOperationException("The World is locked.");

            if (_bodyType != BodyType.Dynamic)
                return;

            // Move center of mass.
            Vector2 oldCenter = _sweep.C;
            _sweep.LocalCenter = value;
            _sweep.C0 = _sweep.C = Transform.Multiply(ref _sweep.LocalCenter, ref _xf);

            // Update center of mass velocity.
            Vector2 a = _sweep.C - oldCenter;
            _linearVelocity += new Vector2(-_angularVelocity * a.Y, _angularVelocity * a.X);
        }
    }

    /// <summary>
    /// Gets or sets the mass. Usually in kilograms (kg).
    /// Warning: This property is readonly during callbacks.
    /// </summary>
    /// <value>The mass.</value>
    /// <exception cref="System.InvalidOperationException">Thrown when the world is Locked/Stepping.</exception>
    public double Mass
    {
        get { return _mass; }
        set
        {
            if (World != null && World.IsLocked)
                throw new InvalidOperationException("The World is locked.");

            Debug.Assert(!double.IsNaN(value));

            if (_bodyType != BodyType.Dynamic) //Make an assert
                return;

            _mass = value;

            if (_mass <= 0.0)
                _mass = 1.0;

            _invMass = 1.0 / _mass;
        }
    }

    /// <summary>
    /// Get or set the rotational inertia of the body about the local origin. usually in kg-m^2.
    /// Warning: This property is readonly during callbacks.
    /// </summary>
    /// <value>The inertia.</value>
    /// <exception cref="System.InvalidOperationException">Thrown when the world is Locked/Stepping.</exception>
    public double Inertia
    {
        get { return _inertia + Mass * Vector2.Dot(_sweep.LocalCenter, _sweep.LocalCenter); }
        set
        {
            if (World != null && World.IsLocked)
                throw new InvalidOperationException("The World is locked.");

            Debug.Assert(!double.IsNaN(value));

            if (_bodyType != BodyType.Dynamic) //Make an assert
                return;

            if (value > 0.0 && !_fixedRotation) //Make an assert
            {
                _inertia = value - Mass * Vector2.Dot(LocalCenter, LocalCenter);
                Debug.Assert(_inertia > 0.0);
                _invI = 1.0 / _inertia;
            }
        }
    }

    public bool IgnoreCCD { get; set; }

    /// <summary>
    /// Resets the dynamics of this body.
    /// Sets torque, force and linear/angular velocity to 0
    /// </summary>
    public void ResetDynamics()
    {
        _torque = 0;
        _angularVelocity = 0;
        _force = Vector2.Zero;
        _linearVelocity = Vector2.Zero;
    }

    ///<summary>
    /// Warning: This method is locked during callbacks.
    /// </summary>>
    /// <exception cref="System.InvalidOperationException">Thrown when the world is Locked/Stepping.</exception>
    public void Add(Fixture fixture)
    {
        if (World != null && World.IsLocked)
            throw new InvalidOperationException("The World is locked.");
        if (fixture == null)
            throw new ArgumentNullException(nameof(fixture));
        if (fixture.Body != null)
        {
            if (fixture.Body == this)
                throw new ArgumentException("You are adding the same fixture more than once.", nameof(fixture));
            else
                throw new ArgumentException("fixture belongs to another body.", nameof(fixture));
        }

        fixture.Body = this;
        FixtureList._list.Add(fixture);
        FixtureList._generationStamp++;
#if DEBUG
        if (fixture.Shape.ShapeType == ShapeType.Polygon)
            ((PolygonShape)fixture.Shape).Vertices.AttachedToBody = true;
#endif

        // Adjust mass properties if needed.
        if (fixture.Shape._density > 0.0)
            ResetMassData();

        if (World != null)
        {
            if (Enabled)
            {
                IBroadPhase broadPhase = World.ContactManager.BroadPhase;
                fixture.CreateProxies(broadPhase, ref _xf);
            }

            // Let the world know we have a new fixture. This will cause new contacts
            // to be created at the beginning of the next time step.
            World._worldHasNewFixture = true;

            var fixtureAddedHandler = World.FixtureAdded;
            if (fixtureAddedHandler != null)
                fixtureAddedHandler(World, this, fixture);
        }
    }

    /// <summary>
    /// Destroy a fixture. This removes the fixture from the broad-phase and
    /// destroys all contacts associated with this fixture. This will
    /// automatically adjust the mass of the body if the body is dynamic and the
    /// fixture has positive density.
    /// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
    /// Warning: This method is locked during callbacks.
    /// </summary>
    /// <param name="fixture">The fixture to be removed.</param>
    /// <exception cref="System.InvalidOperationException">Thrown when the world is Locked/Stepping.</exception>
    public virtual void Remove(Fixture fixture)
    {
        if (World != null && World.IsLocked)
            throw new InvalidOperationException("The World is locked.");
        ArgumentNullException.ThrowIfNull(fixture);
        if (fixture.Body != this)
            throw new ArgumentException("You are removing a fixture that does not belong to this Body.", nameof(fixture));

        // Destroy any contacts associated with the fixture.
        ContactEdge edge = ContactList;
        while (edge != null)
        {
            Contact c = edge.Contact;
            edge = edge.Next;

            Fixture fixtureA = c.FixtureA;
            Fixture fixtureB = c.FixtureB;

            if (fixture == fixtureA || fixture == fixtureB)
            {
                // This destroys the contact and removes it from
                // this body's contact list.
                World.ContactManager.Destroy(c);
            }
        }

        if (Enabled)
        {
            IBroadPhase broadPhase = World.ContactManager.BroadPhase;
            fixture.DestroyProxies(broadPhase);
        }

        fixture.Body = null;
        FixtureList._list.Remove(fixture);
        FixtureList._generationStamp++;
#if DEBUG
        if (fixture.Shape.ShapeType == ShapeType.Polygon)
            ((PolygonShape)fixture.Shape).Vertices.AttachedToBody = false;
#endif

        var fixtureRemovedHandler = World.FixtureRemoved;
        if (fixtureRemovedHandler != null)
            fixtureRemovedHandler(World, this, fixture);

        ResetMassData();
    }

    /// <summary>
    /// Set the position of the body's origin and rotation.
    /// This breaks any contacts and wakes the other bodies.
    /// Manipulating a body's transform may cause non-physical behavior.
    /// Warning: This method is locked during callbacks.
    /// </summary>
    /// <param name="position">The world position of the body's local origin.</param>
    /// <param name="rotation">The world rotation in radians.</param>
    /// <exception cref="System.InvalidOperationException">Thrown when the world is Locked/Stepping.</exception>
    public void SetTransform(ref Vector2 position, double rotation)
    {
        SetTransformIgnoreContacts(ref position, rotation);

        World.ContactManager.FindNewContacts();
    }

    /// <summary>
    /// Set the position of the body's origin and rotation.
    /// This breaks any contacts and wakes the other bodies.
    /// Manipulating a body's transform may cause non-physical behavior.
    /// Warning: This method is locked during callbacks.
    /// </summary>
    /// <param name="position">The world position of the body's local origin.</param>
    /// <param name="rotation">The world rotation in radians.</param>
    /// <exception cref="System.InvalidOperationException">Thrown when the world is Locked/Stepping.</exception>
    public void SetTransform(Vector2 position, double rotation)
    {
        SetTransform(ref position, rotation);
    }

    /// <summary>
    /// For teleporting a body without considering new contacts immediately.
    /// Warning: This method is locked during callbacks.
    /// </summary>
    /// <param name="position">The position.</param>
    /// <param name="angle">The angle.</param>
    /// <exception cref="System.InvalidOperationException">Thrown when the world is Locked/Stepping.</exception>
    public void SetTransformIgnoreContacts(ref Vector2 position, double angle)
    {
        Debug.Assert(World != null);
        if (World.IsLocked)
            throw new InvalidOperationException("The World is locked.");

        _xf.Rotation.Phase = angle;
        _xf.Position = position;

        _sweep.C = Transform.Multiply(ref _sweep.LocalCenter, ref _xf);
        _sweep.A = angle;

        _sweep.C0 = _sweep.C;
        _sweep.A0 = angle;

        IBroadPhase broadPhase = World.ContactManager.BroadPhase;
        for (int i = 0; i < FixtureList._list.Count; i++)
            FixtureList._list[i].Synchronize(broadPhase, ref _xf, ref _xf);
    }

    /// <summary>
    /// Get the body transform for the body's origin.
    /// </summary>
    /// <param name="transform">The transform of the body's origin.</param>
    public Transform GetTransform()
    {
        return _xf;
    }

    /// <summary>
    /// Get the body transform for the body's origin.
    /// </summary>
    /// <param name="transform">The transform of the body's origin.</param>
    public void GetTransform(out Transform transform)
    {
        transform = _xf;
    }

    /// <summary>
    /// Apply a force at a world point. If the force is not
    /// applied at the center of mass, it will generate a torque and
    /// affect the angular velocity. This wakes up the body.
    /// </summary>
    /// <param name="force">The world force vector, usually in Newtons (N).</param>
    /// <param name="point">The world position of the point of application.</param>
    public void ApplyForce(Vector2 force, Vector2 point)
    {
        ApplyForce(ref force, ref point);
    }

    /// <summary>
    /// Applies a force at the origin position.
    /// If the origin position is not the same as the center of mass,
    /// it will generate a torque and affect the angular velocity.
    /// This wakes up the body.
    /// </summary>
    /// <param name="force">The force.</param>
    public void ApplyForce(ref Vector2 force)
    {
        ApplyForce(ref force, ref _xf.Position);
    }

    /// <summary>
    /// Applies a force at the origin position.
    /// If the origin position is not the same as the center of mass,
    /// it will generate a torque and affect the angular velocity.
    /// This wakes up the body.
    /// </summary>
    /// <param name="force">The force.</param>
    public void ApplyForce(Vector2 force)
    {
        ApplyForce(ref force, ref _xf.Position);
    }

    /// <summary>
    /// Apply a force at a world point. If the force is not
    /// applied at the center of mass, it will generate a torque and
    /// affect the angular velocity. This wakes up the body.
    /// </summary>
    /// <param name="force">The world force vector, usually in Newtons (N).</param>
    /// <param name="point">The world position of the point of application.</param>
    public void ApplyForce(ref Vector2 force, ref Vector2 point)
    {
        Debug.Assert(!double.IsNaN(force.X));
        Debug.Assert(!double.IsNaN(force.Y));
        Debug.Assert(!double.IsNaN(point.X));
        Debug.Assert(!double.IsNaN(point.Y));

        if (_bodyType == BodyType.Dynamic)
        {
            if (Awake == false)
                Awake = true;

            _force += force;
            _torque += (point.X - _sweep.C.X) * force.Y - (point.Y - _sweep.C.Y) * force.X;
        }
    }

    /// <summary>
    /// Apply a torque. This affects the angular velocity
    /// without affecting the linear velocity of the center of mass.
    /// This wakes up the body.
    /// </summary>
    /// <param name="torque">The torque about the z-axis (out of the screen), usually in N-m.</param>
    public void ApplyTorque(double torque)
    {
        Debug.Assert(!double.IsNaN(torque));

        if (_bodyType == BodyType.Dynamic)
        {
            if (Awake == false)
                Awake = true;

            _torque += torque;
        }
    }

    /// <summary>
    /// Apply an impulse at a point. This immediately modifies the velocity.
    /// This wakes up the body.
    /// </summary>
    /// <param name="impulse">The world impulse vector, usually in N-seconds or kg-m/s.</param>
    public void ApplyLinearImpulse(Vector2 impulse)
    {
        ApplyLinearImpulse(ref impulse);
    }

    /// <summary>
    /// Apply an impulse at a point. This immediately modifies the velocity.
    /// It also modifies the angular velocity if the point of application
    /// is not at the center of mass.
    /// This wakes up the body.
    /// </summary>
    /// <param name="impulse">The world impulse vector, usually in N-seconds or kg-m/s.</param>
    /// <param name="point">The world position of the point of application.</param>
    public void ApplyLinearImpulse(Vector2 impulse, Vector2 point)
    {
        ApplyLinearImpulse(ref impulse, ref point);
    }

    /// <summary>
    /// Apply an impulse at a point. This immediately modifies the velocity.
    /// This wakes up the body.
    /// </summary>
    /// <param name="impulse">The world impulse vector, usually in N-seconds or kg-m/s.</param>
    public void ApplyLinearImpulse(ref Vector2 impulse)
    {
        if (_bodyType != BodyType.Dynamic)
        {
            return;
        }
        if (Awake == false)
        {
            Awake = true;
        }
        _linearVelocity += _invMass * impulse;
    }

    /// <summary>
    /// Apply an impulse at a point. This immediately modifies the velocity.
    /// It also modifies the angular velocity if the point of application
    /// is not at the center of mass.
    /// This wakes up the body.
    /// </summary>
    /// <param name="impulse">The world impulse vector, usually in N-seconds or kg-m/s.</param>
    /// <param name="point">The world position of the point of application.</param>
    public void ApplyLinearImpulse(ref Vector2 impulse, ref Vector2 point)
    {
        if (_bodyType != BodyType.Dynamic)
            return;

        if (Awake == false)
            Awake = true;

        _linearVelocity += _invMass * impulse;
        _angularVelocity += _invI * ((point.X - _sweep.C.X) * impulse.Y - (point.Y - _sweep.C.Y) * impulse.X);
    }

    /// <summary>
    /// Apply an angular impulse.
    /// </summary>
    /// <param name="impulse">The angular impulse in units of kg*m*m/s.</param>
    public void ApplyAngularImpulse(double impulse)
    {
        if (_bodyType != BodyType.Dynamic)
        {
            return;
        }

        if (Awake == false)
        {
            Awake = true;
        }

        _angularVelocity += _invI * impulse;
    }

    /// <summary>
    /// This resets the mass properties to the sum of the mass properties of the fixtures.
    /// This normally does not need to be called unless you called SetMassData to override
    /// the mass and you later want to reset the mass.
    /// </summary>
    public void ResetMassData()
    {
        // Compute mass data from shapes. Each shape has its own density.
        _mass = 0.0;
        _invMass = 0.0;
        _inertia = 0.0;
        _invI = 0.0;
        _sweep.LocalCenter = Vector2.Zero;

        // Kinematic bodies have zero mass.
        if (BodyType == BodyType.Kinematic)
        {
            _sweep.C0 = _xf.Position;
            _sweep.C = _xf.Position;
            _sweep.A0 = _sweep.A;
            return;
        }

        Debug.Assert(BodyType == BodyType.Dynamic || BodyType == BodyType.Static);

        // Accumulate mass over all fixtures.
        Vector2 localCenter = Vector2.Zero;
        foreach (Fixture f in FixtureList)
        {
            if (f.Shape._density == 0)
            {
                continue;
            }

            MassData massData = f.Shape.MassData;
            _mass += massData.Mass;
            localCenter += massData.Mass * massData.Centroid;
            _inertia += massData.Inertia;
        }

        //FPE: Static bodies only have mass, they don't have other properties. A little hacky tho...
        if (BodyType == BodyType.Static)
        {
            _sweep.C0 = _sweep.C = _xf.Position;
            return;
        }

        // Compute center of mass.
        if (_mass > 0.0)
        {
            _invMass = 1.0 / _mass;
            localCenter *= _invMass;
        }
        else
        {
            // Force all dynamic bodies to have a positive mass.
            _mass = 1.0;
            _invMass = 1.0;
        }

        if (_inertia > 0.0 && !_fixedRotation)
        {
            // Center the inertia about the center of mass.
            _inertia -= _mass * Vector2.Dot(localCenter, localCenter);

            Debug.Assert(_inertia > 0.0);
            _invI = 1.0 / _inertia;
        }
        else
        {
            _inertia = 0.0;
            _invI = 0.0;
        }

        // Move center of mass.
        Vector2 oldCenter = _sweep.C;
        _sweep.LocalCenter = localCenter;
        _sweep.C0 = _sweep.C = Transform.Multiply(ref _sweep.LocalCenter, ref _xf);

        // Update center of mass velocity.
        Vector2 a = _sweep.C - oldCenter;
        _linearVelocity += new Vector2(-_angularVelocity * a.Y, _angularVelocity * a.X);
    }

    /// <summary>
    /// Get the world coordinates of a point given the local coordinates.
    /// </summary>
    /// <param name="localPoint">A point on the body measured relative the the body's origin.</param>
    /// <returns>The same point expressed in world coordinates.</returns>
    public Vector2 GetWorldPoint(ref Vector2 localPoint)
    {
        return Transform.Multiply(ref localPoint, ref _xf);
    }

    /// <summary>
    /// Get the world coordinates of a point given the local coordinates.
    /// </summary>
    /// <param name="localPoint">A point on the body measured relative the the body's origin.</param>
    /// <returns>The same point expressed in world coordinates.</returns>
    public Vector2 GetWorldPoint(Vector2 localPoint)
    {
        return GetWorldPoint(ref localPoint);
    }

    /// <summary>
    /// Get the world coordinates of a vector given the local coordinates.
    /// Note that the vector only takes the rotation into account, not the position.
    /// </summary>
    /// <param name="localVector">A vector fixed in the body.</param>
    /// <returns>The same vector expressed in world coordinates.</returns>
    public Vector2 GetWorldVector(ref Vector2 localVector)
    {
        return Complex.Multiply(ref localVector, ref _xf.Rotation);
    }

    /// <summary>
    /// Get the world coordinates of a vector given the local coordinates.
    /// </summary>
    /// <param name="localVector">A vector fixed in the body.</param>
    /// <returns>The same vector expressed in world coordinates.</returns>
    public Vector2 GetWorldVector(Vector2 localVector)
    {
        return GetWorldVector(ref localVector);
    }

    /// <summary>
    /// Gets a local point relative to the body's origin given a world point.
    /// Note that the vector only takes the rotation into account, not the position.
    /// </summary>
    /// <param name="worldPoint">A point in world coordinates.</param>
    /// <returns>The corresponding local point relative to the body's origin.</returns>
    public Vector2 GetLocalPoint(ref Vector2 worldPoint)
    {
        return Transform.Divide(ref worldPoint, ref _xf);
    }

    /// <summary>
    /// Gets a local point relative to the body's origin given a world point.
    /// </summary>
    /// <param name="worldPoint">A point in world coordinates.</param>
    /// <returns>The corresponding local point relative to the body's origin.</returns>
    public Vector2 GetLocalPoint(Vector2 worldPoint)
    {
        return GetLocalPoint(ref worldPoint);
    }

    /// <summary>
    /// Gets a local vector given a world vector.
    /// Note that the vector only takes the rotation into account, not the position.
    /// </summary>
    /// <param name="worldVector">A vector in world coordinates.</param>
    /// <returns>The corresponding local vector.</returns>
    public Vector2 GetLocalVector(ref Vector2 worldVector)
    {
        return Complex.Divide(ref worldVector, ref _xf.Rotation);
    }

    /// <summary>
    /// Gets a local vector given a world vector.
    /// Note that the vector only takes the rotation into account, not the position.
    /// </summary>
    /// <param name="worldVector">A vector in world coordinates.</param>
    /// <returns>The corresponding local vector.</returns>
    public Vector2 GetLocalVector(Vector2 worldVector)
    {
        return GetLocalVector(ref worldVector);
    }

    /// <summary>
    /// Get the world linear velocity of a world point attached to this body.
    /// </summary>
    /// <param name="worldPoint">A point in world coordinates.</param>
    /// <returns>The world velocity of a point.</returns>
    public Vector2 GetLinearVelocityFromWorldPoint(Vector2 worldPoint)
    {
        return GetLinearVelocityFromWorldPoint(ref worldPoint);
    }

    /// <summary>
    /// Get the world linear velocity of a world point attached to this body.
    /// </summary>
    /// <param name="worldPoint">A point in world coordinates.</param>
    /// <returns>The world velocity of a point.</returns>
    public Vector2 GetLinearVelocityFromWorldPoint(ref Vector2 worldPoint)
    {
        return _linearVelocity +
                new Vector2(-_angularVelocity * (worldPoint.Y - _sweep.C.Y),
                            _angularVelocity * (worldPoint.X - _sweep.C.X));
    }

    /// <summary>
    /// Get the world velocity of a local point.
    /// </summary>
    /// <param name="localPoint">A point in local coordinates.</param>
    /// <returns>The world velocity of a point.</returns>
    public Vector2 GetLinearVelocityFromLocalPoint(Vector2 localPoint)
    {
        return GetLinearVelocityFromLocalPoint(ref localPoint);
    }

    /// <summary>
    /// Get the world velocity of a local point.
    /// </summary>
    /// <param name="localPoint">A point in local coordinates.</param>
    /// <returns>The world velocity of a point.</returns>
    public Vector2 GetLinearVelocityFromLocalPoint(ref Vector2 localPoint)
    {
        return GetLinearVelocityFromWorldPoint(GetWorldPoint(ref localPoint));
    }

    internal void SynchronizeFixtures()
    {
        Transform xf1 = new Transform(Vector2.Zero, _sweep.A0);
        xf1.Position = _sweep.C0 - Complex.Multiply(ref _sweep.LocalCenter, ref xf1.Rotation);

        IBroadPhase broadPhase = World.ContactManager.BroadPhase;
        for (int i = 0; i < FixtureList._list.Count; i++)
        {
            FixtureList._list[i].Synchronize(broadPhase, ref xf1, ref _xf);
        }
    }

    internal void SynchronizeTransform()
    {
        _xf.Rotation.Phase = _sweep.A;
        _xf.Position = _sweep.C - Complex.Multiply(ref _sweep.LocalCenter, ref _xf.Rotation);
    }

    /// <summary>
    /// This is used to prevent connected bodies from colliding.
    /// It may lie, depending on the collideConnected flag.
    /// </summary>
    /// <param name="other">The other body.</param>
    /// <returns></returns>
    internal bool ShouldCollide(Body other)
    {
        // At least one body should be dynamic.
        if (_bodyType != BodyType.Dynamic && other._bodyType != BodyType.Dynamic)
        {
            return false;
        }

        // Does a joint prevent collision?
        for (JointEdge jn = JointList; jn != null; jn = jn.Next)
        {
            if (jn.Other == other)
            {
                if (jn.Joint.CollideConnected == false)
                {
                    return false;
                }
            }
        }

        return true;
    }

    internal void Advance(double alpha)
    {
        // Advance to the new safe time. This doesn't sync the broad-phase.
        _sweep.Advance(alpha);
        _sweep.C = _sweep.C0;
        _sweep.A = _sweep.A0;
        _xf.Rotation.Phase = _sweep.A;
        _xf.Position = _sweep.C - Complex.Multiply(ref _sweep.LocalCenter, ref _xf.Rotation);
    }

    internal OnCollisionEventHandler onCollisionEventHandler;
    public event OnCollisionEventHandler OnCollision
    {
        add { onCollisionEventHandler += value; }
        remove { onCollisionEventHandler -= value; }
    }

    internal OnSeparationEventHandler onSeparationEventHandler;
    public event OnSeparationEventHandler OnSeparation
    {
        add { onSeparationEventHandler += value; }
        remove { onSeparationEventHandler -= value; }
    }


    /// <summary>
    /// Makes a clone of the body. Fixtures and therefore shapes are not included.
    /// Use DeepClone() to clone the body, as well as fixtures and shapes.
    /// </summary>
    /// <param name="world"></param>
    /// <returns></returns>
    public Body Clone(World world = null)
    {
        world ??= World;

        Body body = new()
        {
            Position = Position,
            Rotation = Rotation,
            BodyType = BodyType.Static
        };
#if LEGACY_ASYNCADDREMOVE
        world.AddAsync(body);
#else
        world.Add(body);
#endif
        body._bodyType = _bodyType;
        body._linearVelocity = _linearVelocity;
        body._angularVelocity = _angularVelocity;
        body.Tag = Tag;
        body._enabled = _enabled;
        body._fixedRotation = _fixedRotation;
        body._sleepingAllowed = _sleepingAllowed;
        body._linearDamping = _linearDamping;
        body._angularDamping = _angularDamping;
        body._awake = _awake;
        body.IsBullet = IsBullet;
        body.IgnoreCCD = IgnoreCCD;
        body.IgnoreGravity = IgnoreGravity;
        body._torque = _torque;

        return body;
    }

    /// <summary>
    /// Clones the body and all attached fixtures and shapes. Simply said, it makes a complete copy of the body.
    /// </summary>
    /// <param name="world"></param>
    /// <returns></returns>
    public Body DeepClone(World world = null)
    {
        Body body = Clone(world ?? World);

        int count = FixtureList._list.Count; //Make a copy of the count. Otherwise it causes an infinite loop.
        for (int i = 0; i < count; i++)
        {
            FixtureList._list[i].CloneOnto(body);
        }

        return body;
    }
}