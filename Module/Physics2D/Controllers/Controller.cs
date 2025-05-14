using StdbModule.Physics2D.Common.Logic;
using StdbModule.Physics2D.Dynamics;

namespace StdbModule.Physics2D.Controllers;

public abstract class Controller : FilterData
{
    public ControllerCategory ControllerCategory = ControllerCategory.Cat01;

    public bool Enabled = true;
    public World World { get; internal set; }

    public Controller() { }

    public override bool IsActiveOn(Body body)
    {
        if (body.ControllerFilter.IsControllerIgnored(ControllerCategory))
            return false;

        return base.IsActiveOn(body);
    }

    public abstract void Update(double dt);
}
