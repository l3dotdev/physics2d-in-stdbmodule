namespace StdbModule.Physics2D.Common.Decomposition.Seidel;

internal class Point(double x, double y)
{
    // Pointers to next and previous points in Monontone Mountain
    public Point? Next = null, Prev = null;
    public double X = x, Y = y;

    public static Point operator -(Point p1, Point p2)
    {
        return new Point(p1.X - p2.X, p1.Y - p2.Y);
    }

    public static Point operator +(Point p1, Point p2)
    {
        return new Point(p1.X + p2.X, p1.Y + p2.Y);
    }

    public static Point operator -(Point p1, double f)
    {
        return new Point(p1.X - f, p1.Y - f);
    }

    public static Point operator +(Point p1, double f)
    {
        return new Point(p1.X + f, p1.Y + f);
    }

    public double Cross(Point p)
    {
        return X * p.Y - Y * p.X;
    }

    public double Dot(Point p)
    {
        return X * p.X + Y * p.Y;
    }

    public bool Neq(Point p)
    {
        return p.X != X || p.Y != Y;
    }

    public double Orient2D(Point pb, Point pc)
    {
        double acx = X - pc.X;
        double bcx = pb.X - pc.X;
        double acy = Y - pc.Y;
        double bcy = pb.Y - pc.Y;
        return acx * bcy - acy * bcx;
    }
}
