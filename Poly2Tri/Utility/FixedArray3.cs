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
using System.Collections;
using System.Collections.Generic;

namespace Poly2Tri.Utility
{
    public struct FixedArray3<T> : IEnumerable<T> where T : IEquatable<T>
    {
        public T Item0;
        public T Item1;
        public T Item2;

        public T this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0:
                        return Item0;
                    case 1:
                        return Item1;
                    case 2:
                        return Item2;
                    default:
                        throw new IndexOutOfRangeException();
                }
            }
            set
            {
                switch (index)
                {
                    case 0:
                        Item0 = value;
                        break;
                    case 1:
                        Item1 = value;
                        break;
                    case 2:
                        Item2 = value;
                        break;
                    default:
                        throw new IndexOutOfRangeException();
                }
            }
        }


        public bool Contains(T value)
        {
            return IndexOf(value) != -1;
        }


        public int IndexOf(T value)
        {
            for (int i = 0; i < 3; ++i)
            {
                if (!this[i].Equals(default(T)) && this[i].Equals(value))
                {
                    return i;
                }
            }

            return -1;
        }

        
        public void Clear()
        {
            Item0 = Item1 = Item2 = default(T);
        }

        
        public void Clear(T value)
        {
            for (int i = 0; i < 3; ++i)
            {
                if (this[i].Equals(default(T)) && this[i].Equals(value))
                {
                    this[i] = default(T);
                }
            }
        }


        private IEnumerable<T> Enumerate()
        {
            for (int i = 0; i < 3; ++i)
            {
                yield return this[i];
            }
        }

        
        public IEnumerator<T> GetEnumerator() { return Enumerate().GetEnumerator(); }

        
        IEnumerator IEnumerable.GetEnumerator() { return GetEnumerator(); }
    }
}
