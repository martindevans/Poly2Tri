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
    public struct Rect2D
    {
        private readonly double _minX;
        private readonly double _maxX;
        private readonly double _minY;
        private readonly double _maxY;

        public double MinX { get { return _minX; } }
        public double MaxX { get { return _maxX; } }
        public double MinY { get { return _minY; } }
        public double MaxY { get { return _maxY; } }
        public double Left { get { return _minX; } }
        public double Right { get { return _maxX; } }
        public double Top { get { return _maxY; } }
        public double Bottom { get { return _minY; } }

        public double Width { get { return (Right - Left); } }
        public double Height { get { return (Top - Bottom); } }

        public bool IsEmpty
        {
            get
            {
                return Math.Abs(Width) < float.Epsilon || Math.Abs(Height) < float.Epsilon;
            }
        }

        private Rect2D(double minX, double maxX, double minY, double maxY)
        {
            _minX = minX;
            _maxX = maxX;
            _minY = minY;
            _maxY = maxY;
        }

        public override int GetHashCode()
        {
            unchecked
            {
// ReSharper disable ImpureMethodCallOnReadonlyValueField
                return 54734431 * _minX.GetHashCode()
                    + 1122547711 * _minY.GetHashCode()
                    + 1097393683 * _maxX.GetHashCode()
                    + 1198754321 * _maxY.GetHashCode();
// ReSharper restore ImpureMethodCallOnReadonlyValueField
            }

        }

        public override bool Equals(Object obj)
        {
            return obj is Rect2D && Equals((Rect2D)obj);
        }

        private bool Equals(Rect2D r, double epsilon = MathUtil.EPSILON)
        {
            return MathUtil.AreValuesEqual(MinX, r.MinX, epsilon)
                && MathUtil.AreValuesEqual(MaxX, r.MaxX, epsilon)
                && MathUtil.AreValuesEqual(MinY, r.MinY, epsilon)
                && MathUtil.AreValuesEqual(MaxY, r.MaxY, epsilon);
        }

        public bool Intersects(Rect2D r)
        {
            return  (Right > r.Left) &&
                    (Left < r.Right) &&
                    (Bottom < r.Top) &&
                    (Top > r.Bottom);
        }

        public Rect2D AddPoint(Point2D p)
        {
            return new Rect2D(
                Math.Min(MinX, p.X),
                Math.Max(MaxX, p.X),
                Math.Min(MinY, p.Y),
                Math.Max(MaxY, p.Y)
            );
        }
    }
}
