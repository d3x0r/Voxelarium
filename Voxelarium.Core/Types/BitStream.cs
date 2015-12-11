using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Voxelarium.Core.Types
{
	public class BitStream
	{
		static uint[] bit_masks;
		byte[] storage;
		int available, used;
		int used_bits;

		static BitStream()
		{
			int n;
			uint mask = 0;
			bit_masks = new uint[33];
			for( n = 0; n <= 32; n++ )
			{
				bit_masks[n] = mask;
				mask = ( mask << 1 ) | 1;
			}
		}

		public static int GetMinBitsNeededForValue( int value )
		{
			for( int n = 1; n <= 32; n++ )
			{
				if( ( ( value & bit_masks[n] ) ^ value ) == 0 )
					return n;
			}
			return 32;
		}

		public BitStream()
		{
			available = 512;
			storage = new byte[available];
		}

		public BitStream(byte[] data)
		{
			available = data.Length;
			storage = data;
		}

		public void GetBytes( out byte[] data, out int bytes_used )
		{
			data = storage;
			bytes_used = used + 1;
		}

		public void Expand()
		{
			int new_size = available * 2;
			byte[] new_storage = new byte[new_size];
			Buffer.BlockCopy( storage, 0, new_storage, 0, used );
			storage = new_storage;
			available = new_size;
			
		}

		public void Seek( int position )
		{
			if( (position >> 3) >= available )
				throw new Exception( "attempt to seek beyond end of data" );

			used = position >> 3;
			used_bits = position & 0x7;
		}

		public void Write( byte value, int bits )
		{
			if( bits > 8 )
				throw new Exception( "Attempt to write more bits than data passed" );
			byte tmp;
			if( used == available ) Expand();
			tmp = storage[used];
			tmp |= (byte)(value << used_bits);
			storage[used] = tmp;
			int bit_counter = 8 - used_bits;
			used_bits += bits;
			if( used_bits >= 8 )
			{
				used_bits -= 8;
				used++;
				if( used == available ) Expand();
				if( used_bits > 0 )
				{
					tmp = 0;
					storage[used] = (byte)( value >> ( bit_counter ) );
				}
			}
		}
		public void Write( ushort value, int bits )
		{
			if( bits > 16 )
				throw new Exception( "Attempt to write more bits than data passed" );
			byte tmp;
			if( used == available ) Expand();
			tmp = storage[used];
			tmp |= (byte)(value << used_bits);
			storage[used] = tmp;
			int bit_counter = 8 - used_bits;
			used_bits += bits;
			while( used_bits >= 8 )
			{
				used_bits -= 8;
				used++;
				if( used == available ) Expand();
				if( used_bits > 0 )
				{
					storage[used] = (byte)( value >> ( bit_counter ) );
					bit_counter += 8;
				}
			}
		}
		public void Write( uint value, int bits )
		{
			if( bits > 32 )
				throw new Exception( "Attempt to write more bits than data passed" );
			byte tmp;
			tmp = storage[used];
			tmp |= (byte)( value << used_bits );
			storage[used] = tmp;
			int bit_counter = 8 - used_bits;
			used_bits += bits;
			while( used_bits >= 8 )
			{
				used_bits -= 8;
				used++;
				if( used == available ) Expand();
				if( used_bits > 0 )
				{
					storage[used] = (byte)( value >> ( bit_counter ) );
					bit_counter += 8;
				}
			}
		}


		public void Read( int bits, out byte result )
		{
			if( bits > 8 )
				throw new Exception( "Attempt to read more bits than data passed" );
			int tmp;
			if( used == available )
				throw new Exception( "No more data" );
			tmp = storage[used];
			result = (byte)( tmp >> used_bits );
			int bit_counter = 8 - used_bits;
			used_bits += bits;
			while( used_bits >= 8 )
			{
				used_bits -= 8;
				used++;
				if( used_bits > 0 )
				{
					tmp = storage[used];
					result |= (byte)( tmp << ( bit_counter ) );
					bit_counter += 8;
				}
			}
			result &= (byte)bit_masks[bits];
		}

		public void Read( int bits, out ushort result )
		{
			if( bits > 16 )
				throw new Exception( "Attempt to read more bits than data passed" );
			int tmp;
			if( used == available )
				throw new Exception( "No more data" );
			tmp = storage[used];
			result = (ushort)( tmp >> used_bits );
			int bit_counter = 8 - used_bits;
			used_bits += bits;
			while( used_bits >= 8 )
			{
				used_bits -= 8;
				used++;
				if( used_bits > 0 )
				{
					tmp = storage[used];
					result |= (byte)( tmp << ( bit_counter ) );
					bit_counter += 8;
				}
			}
			result &= (ushort)bit_masks[bits];
		}

		public void Read( int bits, out uint result )
		{
			if( bits > 32 )
				throw new Exception( "Attempt to read more bits than data passed" );
			int tmp;
			if( used == available )
				throw new Exception( "No more data" );
			tmp = storage[used];
			result = (uint)( tmp >> used_bits );
			int bit_counter = 8 - used_bits;
			used_bits += bits;
			while( used_bits >= 8 )
			{
				used_bits -= 8;
				used++;
				if( used_bits > 0 )
				{
					tmp = storage[used];
					result |= (uint)( tmp << ( bit_counter ) );
					bit_counter += 8;
				}
			}
			result &= (uint)bit_masks[bits];
		}

	}
}
