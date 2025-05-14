using StdbModule.Physics2D.Dynamics;

namespace StdbModule.Physics2D.Common.Logic;

public abstract class PhysicsLogic(World world) : FilterData
{
    public ControllerCategory ControllerCategory = ControllerCategory.Cat01;

    public World World { get; internal set; } = world;

    public override bool IsActiveOn(Body body)
    {
        if (body.ControllerFilter.IsControllerIgnored(ControllerCategory))
            return false;

        return base.IsActiveOn(body);
    }

}
