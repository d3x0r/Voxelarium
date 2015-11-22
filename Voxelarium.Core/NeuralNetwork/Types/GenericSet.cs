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
	public class GenericSet<T> : ICollection<T>, IList<T>
	{
		LinkedList<GenericSlab<T>> slabs;
		int _slabsize;
		public GenericSet()
		{
			_slabsize = 64;
			slabs = new LinkedList<GenericSlab<T>>();
		}
		public GenericSet( int slabsize )
		{
			_slabsize = slabsize;
			slabs = new LinkedList<GenericSlab<T>>();
		}

		public T Get()
		{
			LinkedListNode<GenericSlab<T>> slabnode = slabs.First;
			while( (slabnode != null) && slabnode.Value.Empty )
			{
				slabnode = slabnode.Next;
			}
			if( slabnode == null )
				slabnode = slabs.AddFirst( new GenericSlab<T>( _slabsize ) );

			GenericSlab<T> slab = slabnode.Value;
			return slab;
		}

		#region ICollection<T> Members

		void ICollection<T>.Add( T item )
		{
			throw new Exception( "Add is not legal with Slab allocator, please use another method to get object from slab." );
		}

		void ICollection<T>.Clear()
		{

			throw new Exception( "The method or operation is not implemented." );
		}

		bool ICollection<T>.Contains( T item )
		{
			throw new Exception( "The method or operation is not implemented." );
		}

		void ICollection<T>.CopyTo( T[] array, int arrayIndex )
		{
			throw new Exception( "The method or operation is not implemented." );
		}

		int ICollection<T>.Count
		{
			get { throw new Exception( "The method or operation is not implemented." ); }
		}

		bool ICollection<T>.IsReadOnly
		{
			get { throw new Exception( "The method or operation is not implemented." ); }
		}

		bool ICollection<T>.Remove( T item )
		{
			throw new Exception( "The method or operation is not implemented." );
		}

		#endregion

		#region IEnumerable<T> Members

		class EnumThing : IEnumerator<T>
		{
			GenericSet<T> _set;
			LinkedListNode<GenericSlab<T>> slab;
			int n;

			public EnumThing( GenericSet<T> set )
			{
				_set = set;
				slab = _set.slabs.First;
				n = -1;
			}

			#region IEnumerator<T> Members

			T IEnumerator<T>.Current
			{
				get { return slab.Value[n]; }
			}

			#endregion

			#region IDisposable Members

			void IDisposable.Dispose()
			{
				_set = null;
				slab = null;
				//throw new Exception( "The method or operation is not implemented." );
			}

			#endregion

			#region IEnumerator Members

			object IEnumerator.Current
			{

				get { return slab.Value[n]; }
			}

			bool IEnumerator.MoveNext()
			{
				do
				{
					n++;
					if( n >= _set._slabsize )
					{
						n = 0;
						slab = slab.Next;
					}
					if( slab == null )
						break;
				}
				while( slab.Value[n] == null );
				if( slab == null )
					return false;
				return true;
			}

			void IEnumerator.Reset()
			{
				slab = _set.slabs.First;
				n = 0;
			}

			#endregion
		}

		IEnumerator<T> IEnumerable<T>.GetEnumerator()
		{

			return new EnumThing( this );
			//throw new Exception( "The method or operation is not implemented." );
		}

		#endregion

		#region IEnumerable Members

		IEnumerator IEnumerable.GetEnumerator()
		{
			throw new Exception( "The method or operation is not implemented." );
		}

		#endregion

		#region IList<T> Members

		int IList<T>.IndexOf( T item )
		{
			throw new Exception( "The method or operation is not implemented." );
		}

		void IList<T>.Insert( int index, T item )
		{
			throw new Exception( "The method or operation is not implemented." );
		}

		void IList<T>.RemoveAt( int index )
		{
			throw new Exception( "The method or operation is not implemented." );
		}

		T IList<T>.this[int index]
		{
			get
			{
				throw new Exception( "The method or operation is not implemented." );
			}
			set
			{
				throw new Exception( "The method or operation is not implemented." );
			}
		}

		#endregion
	}
}
