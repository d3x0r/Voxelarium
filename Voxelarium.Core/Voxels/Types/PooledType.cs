/*
 * This file is part of Voxelarium.
 * Loosely based on PooledType from Bullet-XNA (C# port of Bullet)
 *
 * Copyright 2015-2016 James Buckeyne  *** Added 11/22/2015
 *
 * Voxelarium is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Voxelarium is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
using System;
using System.Collections.Generic;

using System.Text;
using System.Diagnostics;

namespace Voxelarium.Voxels.Types
{

    public class PooledType<T> where T : new()
    {

        public T Get()
        {
            if (m_pool.Count == 0)
            {
                m_pool.Push(new T());
            }
            return m_pool.Pop();
        }

        public void Free(T obj)
        {
            Debug.Assert(!m_pool.Contains(obj));
            m_pool.Push(obj);
        }


        private Stack<T> m_pool = new Stack<T>();
    }
}
