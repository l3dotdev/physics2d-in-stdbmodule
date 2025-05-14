# Aether.Physics2D in a StdbModule

This is a template of a StdbModule that has a copy of Aether.Physics2D (v2.2) in the assembly.

Aether.Physics2D: https://github.com/nkast/Aether.Physics2D/releases/tag/v2.2

All of the Math types that you'll want to store in your tables to persist state are correctly setup as SpacetimeDB types:

-   Complex
-   Vector2
-   Vector3
-   Matrix2x2
-   Matrix3x3
-   Transform

### Example usage:

```cs
public static class Module
{
    [Table(Name = "TickSchedule", Scheduled = nameof(Module.Tick), ScheduledAt = nameof(ScheduledAt))]
    public partial struct TickSchedule
    {
        [PrimaryKey, AutoInc]
        public ulong ScheduleId;
        public ScheduleAt ScheduledAt;

        public Timestamp PreviousTick;
    }

    public static readonly TimeSpan TICK_INTERVAL = TimeSpan.FromMilliseconds(50);
    public static readonly World World = new World();

    [Reducer(ReducerKind.Init)]
    public static void Init(ReducerContext ctx)
    {
        ctx.Db.TickSchedule.Insert(new TickSchedule
        {
            ScheduledAt = new ScheduleAt.Interval(TICK_INTERVAL),
            PreviousTick = ctx.Timestamp
        });
    }

    [Reducer]
    public static void Tick(ReducerContext ctx, TickSchedule schedule)
    {
        if (ctx.Sender != ctx.Identity)
        {
            throw new Exception("Reducer may not be invoked by clients");
        }

        double μsSinceLastTick = ctx.Timestamp.TimeDurationSince(schedule.PreviousTick).Microseconds;
        double delta = μsSinceLastTick / 1_000_000;

        // Apply your forces and such here

        World.Step(delta);

        // Update your tables with state from physics bodies here

        schedule.PreviousTick = ctx.Timestamp;
        ctx.Db.TickSchedule.ScheduleId.Update(schedule);
    }
}
```

> NOTE: There are nullable warnings that need fixing in the physics code, but these can be safely ignored until fixed
