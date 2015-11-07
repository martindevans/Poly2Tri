using System;
using System.Linq;
using System.Text;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Poly2Tri.Triangulation.Polygon;
using Poly2Tri.Utility;

namespace Poly2Tri.Test
{
    [TestClass]
    public class Voronoi
    {
        [TestMethod]
        public void TestMethod1()
        {
            var shape = new PolygonPoint[] {
                new PolygonPoint(200, 100),
                new PolygonPoint(200, -200),
                new PolygonPoint(100, -200),
                new PolygonPoint(100, -100),
                new PolygonPoint(-100, -100),
                new PolygonPoint(-100, 100),
            };

            var polys = new Polygon(shape);
            P2T.Triangulate(polys);

            var result = P2T.Voronoi(polys.Triangles);

            //Display result
            StringBuilder pathVoronoiEdges = new StringBuilder();

            var min = new Point2D(-100, -200);
            foreach (var edge in result)
            {
                var b = edge.Start + edge.Direction * (double.IsPositiveInfinity(edge.Length) ? 1000 : edge.Length);
                pathVoronoiEdges.Append($"<path d=\"M {edge.Start.X} {edge.Start.Y} L {b.X} {b.Y} \" stroke=\"black\"></path>");
            }

            StringBuilder shapeData = new StringBuilder();
            shapeData.Append($"M {shape.First().X} {shape.First().Y}");
            foreach (var point in shape.Skip(1))
                shapeData.Append($"L {point.X} {point.Y} ");
            shapeData.Append("Z");

            Console.WriteLine(
                "<svg width=\"500px\" height=\"500px\"><g transform=\"translate({2} {3})\">{0}<path d=\"{1}\" fill=\"none\" stroke=\"green\"></path></g></svg>",
                pathVoronoiEdges,
                shapeData,
                -min.X + 1,
                -min.Y + 1
            );
        }
    }
}
