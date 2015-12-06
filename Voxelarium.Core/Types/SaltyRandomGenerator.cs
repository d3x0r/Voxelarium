/*
 * This file is part of Voxelarium.
 * Originally part of Xperdex; copied and re-namespaced.
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

namespace Voxelarium.Core
{
	public class SaltyRandomGenerator : ICloneable
	{
		public class SaltData
		{
			public List<byte[]> salt_data;
			internal SaltData()
			{
				salt_data = new List<byte[]>();
			}

			public static SaltData operator +( SaltData salt_data, byte[] salt )
			{
				salt_data.salt_data.Add( salt );
				return salt_data;
			}

			public void Clear()
			{
				salt_data.Clear();
			}
		}

		System.Security.Cryptography.SHA512 bits = System.Security.Cryptography.SHA512.Create();

		public delegate void GetSalt( SaltData add_data_here );
		public event GetSalt getsalt;

		SaltData salt_data = new SaltData();

		byte[] entropy;

		int bits_used;
		int bits_avail;

		public SaltyRandomGenerator()
		{
		}

		public SaltyRandomGenerator( SaltyRandomGenerator clone )
		{
			this.entropy = clone.entropy;
			this.bits_avail = clone.bits_avail;
			this.bits_used = clone.bits_used;
			this.getsalt = clone.getsalt;
		}

		object ICloneable.Clone()
		{
			return new SaltyRandomGenerator( this );
		}

		void GetBits()
		{
			if( getsalt != null )
			{
				getsalt( salt_data );
			}
			int total_bytes;
			if( entropy != null )
			{
				total_bytes = entropy.Length;
			}
			else
				total_bytes = 0;
			foreach( byte[] data in salt_data.salt_data )
			{
				total_bytes += data.Length;
			}
			if( total_bytes > 0 )
			{
				byte[] inbuf = new byte[total_bytes];
				if( entropy != null )
				{
					total_bytes = entropy.Length;
					System.Array.Copy( entropy, 0, inbuf, 0, entropy.Length );
				}
				else
					total_bytes = 0;
				foreach( byte[] data in salt_data.salt_data )
				{
					System.Array.Copy( data, 0, inbuf, total_bytes, data.Length );
					total_bytes += data.Length;
				}

				entropy = bits.ComputeHash( inbuf );

				bits_used = 0;
				bits_avail = entropy.Length * 8;
			}
			else
				entropy = bits.ComputeHash( new byte[0] );
		}

		uint my_mask_mask( int length )
		{
			return ( ( 0xFFU ) >> ( 8 - ( length ) ) );
		}

		int my_get_mask( byte[] data, int bits )
		{
			uint result;
			int offset = 0;
			// how many bits were used of the first byte
			int first_used_bits = ( bits_used & 7 );
			// how many bits are availablt in first byte (to align to byte read)
			int first_bits = 8 - first_used_bits;
			if( first_used_bits > 0 )
			{
				if( bits <= first_bits )
				{
					result = ( ( (uint)data[bits_used / 8] >> first_used_bits ) & my_mask_mask( bits ) );
					bits_used += bits;
					return (int)result;
				}
				else
				{
					result = (uint)data[bits_used / 8] >> first_used_bits;
					bits -= first_bits;
					bits_used += first_bits;
					offset = first_bits;
				}
			}
			else
			{
				result = 0;
			}
			while( bits >= 8 )
			{
				result |= (uint)data[(bits_used/8)] << offset;
				//result |= data[b++];
				bits -= 8;
				offset += 8;
				bits_used += 8;
			}
			if( bits > 0 )
			{
				result |= ( data[bits_used / 8] & my_mask_mask( bits ) ) << offset;
				bits_used += bits;
			}
			return (int)result;
		}

		public int GetEntropy( int bits, bool signed )
		{
			int tmp;
			int partial_tmp = 0; // redundant init; this does not have to be initialized.
			int partial_bits = 0;


			if( bits > ( bits_avail - bits_used ) )
			{
				if( bits_avail - bits_used > 0 )
				{
					partial_bits = bits_avail - bits_used;
					partial_tmp = my_get_mask( entropy, partial_bits );
					bits -= partial_bits;
				}
				GetBits();
			}
			{
				tmp = my_get_mask( entropy, bits );

				if( partial_bits > 0 )
				{
					tmp = partial_tmp | ( tmp << partial_bits );
					bits += partial_bits; // restore bit counter for signed computation
				}
				if( signed )
					if( ( tmp & ( 1 << ( bits - 1 ) ) ) != 0 )
					{
						uint negone = ~(uint)0;
						negone <<= bits;
						return (int)( (uint)tmp | negone );
					}
			}
			return ( tmp );
		}

		public void Reset()
		{
			salt_data.Clear();
			bits_used = 0;
			bits_avail = 0;
			entropy = null;
		}


#if include_bit_shift_debug
		class BitCollector
		{
			internal byte[] buffer;
			int bits_used;
			int bits_avail;

			internal BitCollector( int size )
			{
				int n;
				bits_avail = size;
				buffer = new byte[size];
				size = ( size + 7 ) / 8;
				for( n = 0; n < size; n++ )
					buffer[n] = 0;
			}

			internal void Clear()
			{
				int n;
				bits_used = 0;
				for( n = 0; n < bits_avail / 8; n++ )
					buffer[n] = 0;
			}

			internal void AddBits( uint value, int bits )
			{
				if( ( bits_avail - bits ) >= bits_used )
				{
					int offset = 0;
					int first_use_bits = ( bits_used & 7 );
					// how many bits fit in the first byte .... how many bits to align read offset to a byte
					int first_bits = 8 - first_use_bits;
					//Console.WriteLine( "Add bit "+value+" at " + bits_used );
					if( bits <= first_bits )
					{
						// all bits will fit in the first byte
						buffer[bits_used / 8] |= (byte)( ( value & ( 0xFF >> ( 8 - bits ) ) ) << first_use_bits );
						bits_used += bits;
						return;
					}
					else if( first_use_bits > 0 ) // already used some of the bits....
					{
						buffer[bits_used / 8] |= (byte)( ( value & ( 0xFF >> ( 8 - first_bits ) ) ) << first_use_bits );
						bits_used += first_bits;
						bits -= first_bits;
						offset += first_bits;
					}

					while( bits >= 8 )
					{
						buffer[bits_used / 8] = (byte)( ( value & ( 0xFF << offset ) ) >> offset );
						bits -= 8;
						bits_used += 8;
						offset += 8;
					}
					if( bits > 0 )
					{
						buffer[bits_used / 8] = (byte)( ( value & ( ( 0xFF >> ( 8 - bits ) ) << offset ) ) >> offset );
						bits_used += bits;
					}
				}
				else
				{
					Console.WriteLine( "collector overflow." );
				}
			}
		}
#endif

#if include_bit_shift_debug

		public static void Test()
		{
			SaltyRandomGenerator srg = new SaltyRandomGenerator();
			srg.getsalt += srg_getsalt;
			BitCollector collector = new BitCollector( 512 );
			int bits;
			int n;
			for( bits = 1; bits < 28; bits++ )
			{
				srg.Reset();
				collector.Clear();
				for( n = 0; n < ( 512 - bits ); n += bits )
				{
					int value = srg.GetEntropy( bits, false );
					collector.AddBits( (uint)value, bits );
				}
				{
					int final_value = srg.GetEntropy( 512 - n, false );
					collector.AddBits( (uint)final_value, 512 - n );
				}
				byte[] one = srg.entropy;
				byte[] two = collector.buffer;
				for( n = 0; n < 512 / 8; n++ )
				{
					if( one[n] != two[n] )
						Console.WriteLine( "Byte " + n + " differed " + one[n].ToString( "x" ) + " " + two[n].ToString( "x" ) );
				}
			}

			for( bits = 1; bits < 28; bits++ )
			{
				srg.Reset();
				collector.Clear();
				for( n = 0; n < ( 512 - bits ); n += bits )
				{
					int value = srg.GetEntropy( bits, true );
					collector.AddBits( (uint)value, bits );
					//Console.WriteLine( "data is : " + value );
				}
				{
					int final_value = srg.GetEntropy( 512 - n, false );
					collector.AddBits( (uint)final_value, 512 - n );
				}
				byte[] one = srg.entropy;
				byte[] two = collector.buffer;
				for( n = 0; n < 512 / 8; n++ )
				{
					if( one[n] != two[n] )
						Console.WriteLine( "Byte " + n + " differed " + one[n].ToString( "x" ) + " " + two[n].ToString( "x" ) );
				}
			}
		}

		static void srg_getsalt( SaltyRandomGenerator.SaltData add_data_here )
		{
			add_data_here = add_data_here + BitConverter.GetBytes( 0 );
			//DateTime.Now.ToBinary() );
		}
#endif
	}
}
