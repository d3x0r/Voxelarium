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
