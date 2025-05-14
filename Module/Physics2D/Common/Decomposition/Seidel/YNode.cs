namespace StdbModule.Physics2D.Common.Decomposition.Seidel;

internal class YNode(Edge edge, Node lChild, Node rChild) : Node(lChild, rChild)
{
    private Edge _edge = edge;

    public override Sink Locate(Edge edge)
    {
        if (_edge.IsAbove(edge.P))
            return RightChild.Locate(edge); // Move down the graph

        if (_edge.IsBelow(edge.P))
            return LeftChild.Locate(edge); // Move up the graph

        // s and segment share the same endpoint, p
        if (edge.Slope < _edge.Slope)
            return RightChild.Locate(edge); // Move down the graph

        // Move up the graph
        return LeftChild.Locate(edge);
    }
}
