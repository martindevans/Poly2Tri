/* Poly2Tri
 * Copyright (c) 2009-2010, Poly2Tri Contributors
 * http://code.google.com/p/poly2tri/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

using System;

namespace Poly2Tri.Utility
{
    public class Point2D : IComparable<Point2D>
    {
        private double _x;
        public virtual double X { get { return _x; } set { _x = value; } }
        private double _y;
        public virtual double Y { get { return _y; } set { _y = value; } }

        public float Xf { get { return (float)X; } }
        public float Yf { get { return (float)Y; } }


        public Point2D()
        {
            _x = 0.0;
            _y = 0.0;
        }


        public Point2D(double x, double y)
        {
            _x = x;
            _y = y;
        }

        public override string ToString()
        {
            return string.Format("[{0},{1}]", X, Y);
        }


        public override int GetHashCode()
        {
            unchecked
            {
                return 378163771 * _x.GetHashCode()
                     + 113137337 * _y.GetHashCode();
            }
        }


        public override bool Equals(Object obj)
        {
            Point2D p = obj as Point2D;
            if (p != null)
                return Equals(p);

            return false;
        }

        public bool Equals(Point2D p, double epsilon = 0.0)
        {
            if (p == null || !MathUtil.AreValuesEqual(X, p.X, epsilon) || !MathUtil.AreValuesEqual(Y, p.Y, epsilon))
                return false;

            return true;
        }


        public int CompareTo(Point2D other)
        {
            if (Y < other.Y)
            {
                return -1;
            }
            else if (Y > other.Y)
            {
                return 1;
            }
            else
            {
                if (X < other.X)
                {
                    return -1;
                }
                else if (X > other.X)
                {
                    return 1;
                }
            }

            return 0;
        }


        public virtual void Set(double x, double y) { X = x; Y = y; }

        public void Subtract(Point2D p) { X -= p.X; Y -= p.Y; }
        private void Multiply(double scalar) { X *= scalar; Y *= scalar; }
        public double Magnitude() { return Math.Sqrt(MagnitudeSquared()); }
        public double MagnitudeSquared() { return (X * X) + (Y * Y); }
        private double MagnitudeReciprocal() { return 1.0 / Magnitude(); }
        public void Normalize() { Multiply(MagnitudeReciprocal()); }
        public double Dot(Point2D p) { return (X * p.X) + (Y * p.Y); }
        public double Cross(Point2D p) { return (X * p.Y) - (Y * p.X); }

        public static double Dot(Point2D lhs, Point2D rhs) { return (lhs.X * rhs.X) + (lhs.Y * rhs.Y); }
        public static double Cross(Point2D lhs, Point2D rhs) { return (lhs.X * rhs.Y) - (lhs.Y * rhs.X); }

        // returns a scaled perpendicular vector.  Which direction it goes depends on the order in which the arguments are passed
        public static Point2D Perpendicular(Point2D lhs, double scalar) { Point2D p = new Point2D(lhs.Y * scalar, lhs.X * -scalar); return p; }
        public static Point2D Perpendicular(double scalar, Point2D rhs) { Point2D p = new Point2D(-scalar * rhs.Y, scalar * rhs.X); return p; }

        
        //
        // operator overloading
        //

        // Binary Operators
        // Note that in C#, when a binary operator is overloaded, its corresponding compound assignment operator is also automatically
        // overloaded.  So, for example, overloading operator + implicitly overloads += as well
        public static Point2D operator +(Point2D lhs, Point2D rhs) { return new Point2D(lhs.X + rhs.X, lhs.Y + rhs.Y); }
        public static Point2D operator +(Point2D lhs, double scalar) { return new Point2D(lhs.X + scalar, lhs.Y + scalar); }
        public static Point2D operator -(Point2D lhs, Point2D rhs) { return new Point2D(lhs.X - rhs.X, lhs.Y - rhs.Y); }
        public static Point2D operator -(Point2D lhs, double scalar) { return new Point2D(lhs.X - scalar, lhs.Y - scalar); }
        public static Point2D operator *(Point2D lhs, Point2D rhs) { return new Point2D(lhs.X * rhs.X, lhs.Y * rhs.Y); }
        public static Point2D operator *(Point2D lhs, double scalar) { return new Point2D(lhs.X * scalar, lhs.Y * scalar); }
        public static Point2D operator *(double scalar, Point2D rhs) { return rhs * scalar; }
        public static Point2D operator /(Point2D lhs, Point2D rhs) { return new Point2D(lhs.X / rhs.X, lhs.Y / rhs.Y); }
        public static Point2D operator /(Point2D lhs, double scalar) { return new Point2D(lhs.X / scalar, lhs.Y / scalar); }

        // Unary Operators
        public static Point2D operator -(Point2D p) { return new Point2D(-p.X, -p.Y); }

        // Relational Operators
        //public static bool operator ==(Point2D lhs, Point2D rhs) { if ((object)lhs != null) { return lhs.Equals(rhs, 0.0); } if ((object)rhs == null) { return true; } else { return false; } }
        //public static bool operator !=(Point2D lhs, Point2D rhs) { if ((object)lhs != null) { return !lhs.Equals(rhs, 0.0); } if ((object)rhs == null) { return false; } else { return true; } }
        public static bool operator <(Point2D lhs, Point2D rhs) { return (lhs.CompareTo(rhs) == -1); }
        public static bool operator >(Point2D lhs, Point2D rhs) { return (lhs.CompareTo(rhs) == 1); }
        public static bool operator <=(Point2D lhs, Point2D rhs) { return (lhs.CompareTo(rhs) <= 0); }
        public static bool operator >=(Point2D lhs, Point2D rhs) { return (lhs.CompareTo(rhs) >= 0); }
    }
}
