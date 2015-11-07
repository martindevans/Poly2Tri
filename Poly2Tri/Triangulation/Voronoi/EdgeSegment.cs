using Poly2Tri.Utility;

namespace Poly2Tri.Triangulation.Voronoi
{
    public struct EdgeSegment
    {
        public Point2D Start { get; private set; }
        public Point2D Direction { get; private set; }
        public double Length { get; private set; }

        public EdgeSegment(Point2D start, Point2D direction, double length)
        {
            Start = start;
            Direction = direction;
            Length = length;
        }
    }
}
