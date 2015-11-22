/*
 * This file is part of Voxelarium.
 * Original code from Xperdex; and SACK (System abstraction component kit) 
 * https://github.com/d3x0r/sack
 * https://sf.net/projects/xperdex
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
using System.Collections;

namespace Voxelarium.NeuralNetwork.Types
{
	public class GenericSlab<T>
	{
		public bool Empty // has none left
		{
			get
			{
				if( Count == Capacity )
					return true;
				return false;
			}
		}
		int Capacity;
		int Count;
		BitArray used;
		T[] data;

		public T this[int n]
		{
			get
			{
				if( used[n] ) 
					return data[n];
				return default(T);
			}
		}

		public GenericSlab()
		{
			Capacity = 64;
			Count = 0;
			used = new BitArray( 64 );
			data = new T[64];
		}
		public GenericSlab( int slabsize )
		{
			Capacity = slabsize;
			Count = 0;
			used = new BitArray( slabsize );
			data = new T[ slabsize ];
		}
		public static implicit operator T(GenericSlab<T> slab)
		{
			int n;
			for( n = 0; n < slab.Capacity; n++ )
			{
				if( !slab.used[n] )
				{
					slab.Count++;
					slab.data[n] = (T)Activator.CreateInstance( typeof( T ) );
					slab.used[n] = true;
					return slab.data[n];
				}
			}
			return default(T);
		}
		public void Drop( T node )
		{
			int n;
			for( n = 0; n < Capacity; n++ )
			{
				if( ((object)node).Equals( data[n] ) )
				{
					Count--;
					used[n] = false;
				}
			}
		}
	}
}
