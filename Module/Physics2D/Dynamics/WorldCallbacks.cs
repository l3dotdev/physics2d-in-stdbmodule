using StdbModule.Common;
using StdbModule.Physics2D.Collision;
using StdbModule.Physics2D.Controllers;
using StdbModule.Physics2D.Dynamics.Contacts;
using StdbModule.Physics2D.Dynamics.Joints;

namespace StdbModule.Physics2D.Dynamics;

/// <summary>
/// Called for each fixture found in the query.
/// <returns>true: Continues the query, false: Terminate the query</returns>
/// </summary>
public delegate bool QueryReportFixtureDelegate(Fixture fixture);

/// <summary>
/// Called for each fixture found in the query. You control how the ray cast
/// proceeds by returning a double:
/// return -1: ignore this fixture and continue
/// return 0: terminate the ray cast
/// return fraction: clip the ray to this point
/// return 1: don't clip the ray and continue
/// @param fixture the fixture hit by the ray
/// @param point the point of initial intersection
/// @param normal the normal vector at the point of intersection
/// @return 0 to terminate, fraction to clip the ray for closest hit, 1 to continue
/// </summary>
public delegate double RayCastReportFixtureDelegate(Fixture fixture, Vector2 point, Vector2 normal, double fraction);

/// <summary>
/// This delegate is called when a contact is deleted
/// </summary>
public delegate void EndContactDelegate(Contact contact);

/// <summary>
/// This delegate is called when a contact is created
/// </summary>
public delegate bool BeginContactDelegate(Contact contact);

public delegate void PreSolveDelegate(Contact contact, ref Manifold oldManifold);

public delegate void PostSolveDelegate(Contact contact, ContactVelocityConstraint impulse);

public delegate void FixtureDelegate(World sender, Body body, Fixture fixture);

public delegate void JointDelegate(World sender, Joint joint);

public delegate void BodyDelegate(World sender, Body body);

public delegate void ControllerDelegate(World sender, Controller controller);

public delegate bool CollisionFilterDelegate(Fixture fixtureA, Fixture fixtureB);

public delegate bool BeforeCollisionEventHandler(Fixture sender, Fixture other);

public delegate bool OnCollisionEventHandler(Fixture sender, Fixture other, Contact contact);

public delegate void AfterCollisionEventHandler(Fixture sender, Fixture other, Contact contact, ContactVelocityConstraint impulse);

public delegate void OnSeparationEventHandler(Fixture sender, Fixture other, Contact contact);