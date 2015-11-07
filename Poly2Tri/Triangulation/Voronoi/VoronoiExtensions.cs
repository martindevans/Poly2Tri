using System.Collections.Generic;
using Poly2Tri.Triangulation.Delaunay;
using Poly2Tri.Triangulation.Delaunay.Sweep;
using Poly2Tri.Utility;

namespace Poly2Tri.Triangulation.Voronoi
{
    public static class VoronoiExtensions
    {
        #region voronoi
        public static IEnumerable<EdgeSegment> Voronoi(this IEnumerable<DelaunayTriangle> triangulation)
        {
            var results = new List<EdgeSegment>();
            var done = new HashSet<DelaunayTriangle>();

            foreach (var triangle in triangulation)
            {
                double x, y;
                triangle.CircumCircleCenter(out x, out y);

                foreach (var point in triangle.Points)
                {
                    //Get the neighbour opposite this point
                    var n = triangle.NeighborAcrossFrom(point);

                    //If the neighbour is not null we can proceed with connecting these triangles
                    if (n != null)
                    {
                        //Already done this triangle, nothing more needs doing
                        if (done.Contains(n))
                            continue;

                        //Get the center of the neighbour
                        double nx, ny;
                        n.CircumCircleCenter(out nx, out ny);

                        //Connect these triangles
                        var dir = new Point2D(nx - x, ny - y);
                        var length = dir.Magnitude();
                        dir.Normalize();
                        results.Add(new EdgeSegment(new Point2D(x, y), dir, length));
                    }
                    else
                    {
                        //No neighbour on this side, construct an edge perpendicular to the edge
                        DTSweepConstraint edge;
                        triangle.GetEdgeAcross(point, out edge);

                        //Construct perpendicular direction
                        var edgeDir = edge.EdgeEnd - edge.EdgeStart;
                        edgeDir.Normalize();
                        var perp = new Point2D(-edgeDir.Y, edgeDir.X);

                        //output edge
                        results.Add(new EdgeSegment(new Point2D(x, y), perp, double.PositiveInfinity));
                    }
                }

                //This triangle is done, no future work involving it needs doing
                done.Add(triangle);
            }

            return results;
        }
        #endregion
    }
}
