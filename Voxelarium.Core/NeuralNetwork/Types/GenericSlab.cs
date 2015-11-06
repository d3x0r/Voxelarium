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
