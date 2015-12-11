using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Voxelarium.Common;
using Voxelarium.Core.Types;

namespace Voxelarium.Core.Voxels.Types
{
	/*
					{
					VoxelDataCompressor vdc = new VoxelDataCompressor();
					ushort[] decompress = new ushort[32 * 32 * 32];
					byte[] data;
					int bytes;
					vdc.CompressVoxelData( Sector.Data.Data, out data, out bytes );
					Log.log( "compressed sector is {0}", bytes, 0 );
					vdc.DecompressVoxelData( data, decompress );
					{
						int n;
						for( n = 0; n < 32 * 32 * 32; n++ )
							if( decompress[n] != Sector.Data.Data[n] )
							{
								int a = 3;
							}
					}
				}
 
	*/

		/// <summary>
		/// Handles compressing and decompressing ushort array that is the sector content
		/// </summary>
	public class VoxelDataCompressor
	{

		public VoxelDataCompressor()
		{
			//stream = new BitStream();
		}

		void WriteCubes( BitStream stream, int bits, uint count, uint index )
		{
			// 1 count 2 bits = 3   vs 9
			// 1 count 3 bits = 4   vs 10
			// 1 count  4 bits  = 5     vs 11
			// 1 count  5 bits  = 6     vs 12
			// 1 count  6 bits  = 7     vs 13
			// 1 count  7 bits  = 8     vs 14

			// 2 count 2 bits = 6   vs 9
			// 2 count 3 bits = 8   vs 10
			// 2 count  4 bits  = 10    vs 11
			// 2 count  5 bits  = 12    vs 12
			// 2 count  6 bits  = 14    vs 13
			// 2 count  7 bits  = 16    vs 14

			// 3 count 2 bits = 9   vs 9
			// 3 count 3 bits = 12  vs 10
			// 3 count  4 bits  = 15    vs 11
			// 3 count  5 bits  = 18    vs 12
			// 3 count  6 bits  = 21    vs 13
			// 3 count  7 bits  = 24    vs 14


			if( ( (bits +1) * count ) <= ( 7 + bits ) )
			{
				for( int n = 0; n < count; n++ )
				{
					stream.Write( 0, 1 );
					stream.Write( index, bits );
				}
			}
			else
			{
				stream.Write( 1, 1 );
				//Log.log( "Write count {0}", count );
				/*
				if( ( count & 0x8000 ) != 0 )
				{
					stream.Write( ( ( count >> 15 ) & 0x1F ) | 0x20, 7 );
					stream.Write( ( ( count >> 10 ) & 0x1F ) | 0x20, 7 );
					stream.Write( ( ( count >> 5 ) & 0x1F ) | 0x20, 7 );
					stream.Write( ( ( count  ) & 0x1F ), 7 );
				}
				else */
				count -= 2; // always at least 1, and then this value always starts at atleast 2; and very small delta sets can be 3, but typically 2.
				if( ( count & 0xFC00 ) != 0 )
				{
					stream.Write( ( ( count >> 10 ) & 0x1F ) | 0x20, 6 );
					stream.Write( ( ( count >> 5 ) & 0x1F ) | 0x20, 6 );
					stream.Write( ( ( count ) & 0x1F ), 6 );
				}
				else if( ( count & 0xFFE0 ) != 0 )
				{
					stream.Write( ( ( count >> 5 ) & 0x1F ) | 0x20, 6 );
					stream.Write( ( ( count ) & 0x1F ) , 6 );
				}
				else
				{
					stream.Write( ( ( count ) & 0x1F ), 6 );
				}
				//stream.Write( count, 16 );
				stream.Write( index, bits );
			}

		}

		public void CompressVoxelData( ushort[] data, out byte[] result, out int bytes_used )
		{
			BitStream stream = new BitStream();
			int len = data.Length;
			int n;
			List<ushort> types = new List<ushort>();
			for( n = 0; n < len; n++ )
			{
				if( !types.Contains( data[n] ) )
					types.Add( data[n] );
			}
			int bits = BitStream.GetMinBitsNeededForValue( types.Count - 1 ); // 4 is 0,1,2,3; needs only 2 bits...
			//bits = (bits + 7 ) & 0xf8;
			stream.Seek( 16 ); // seek bit count
			stream.Write( (byte)bits, 8 );
			stream.Write( (uint)types.Count-1, bits );
			foreach( ushort type in types )
				stream.Write( type, 16 );
			ushort prior_cube = 0xFFFF, cube;
			uint index = 0;
			int count = 0;
            for( n = 0; n < len; n++ )
			{
				cube = data[n];
				if( prior_cube != cube )
				{
					if( count > 0 )
						WriteCubes( stream, bits, (uint)count, index );
					index = (uint)types.IndexOf( cube );
					prior_cube = cube;
					count = 1;
				}
				else
					count++;
			}
			WriteCubes( stream, bits, (uint)count, index );

			stream.GetBytes( out result, out bytes_used );
			result[0] = (byte)(( bytes_used ) & 0xFF);
			result[1] = (byte)( ( bytes_used ) >> 8);
			if( bytes_used < 20 )
				return;

			MemoryStream final_stream = new MemoryStream();
			final_stream.Seek( 2, SeekOrigin.Begin );
			//DeflateStream gz_stream = new DeflateStream( final_stream, CompressionMode.Compress );
			//GZipStream gz_stream = new GZipStream( final_stream, CompressionMode.Compress );
			GZipStream gz_stream = new GZipStream( final_stream, CompressionLevel.Fastest );
			//for( n = 2; n < bytes_used; n++ )
			//	gz_stream.WriteByte( result[n] );
			gz_stream.Write( result, 2, bytes_used - 2 );
			gz_stream.Close();
			result = final_stream.ToArray();//.GetBuffer();
			Log.log( "Compressed another {0}%", (float)( 100.0f * (float)( bytes_used - result.Length ) / ( bytes_used - 2 ) ) );
			// store old length so we know how much to request decompressing.
			result[0] = (byte)( ( bytes_used ) & 0xFF );
			result[1] = (byte)( ( bytes_used ) >> 8 );
			bytes_used = result.Length;// (int)final_stream.Length;
			gz_stream.Dispose();
		}

		public void DecompressVoxelData( byte[] data, ushort[] result )
		{
			BitStream stream;
			ushort DataBytes;
			DataBytes = BitConverter.ToUInt16( data, 0 );
			if( DataBytes > 20 )
			{
				MemoryStream initial_stream = new MemoryStream( data );
				initial_stream.Seek( 2, SeekOrigin.Begin );
				byte[] uncompressed_data = new byte[DataBytes];
				GZipStream gz_stream = new GZipStream( initial_stream, CompressionMode.Decompress );
				gz_stream.Read( uncompressed_data, 0, DataBytes );
				stream = new BitStream( uncompressed_data );
			}
			else
			{
				stream = new BitStream( data );
				stream.Seek( 16 ); // seek by bit position
			}
			List<ushort> types = new List<ushort>();

			byte bits;
			uint TypeCount;
			stream.Read( 8, out bits );
			stream.Read( bits, out TypeCount );
			TypeCount++; // always at least one.
			int n;
			for( n = 0; n < TypeCount; n++ )
			{
				ushort val;
				stream.Read( 16, out val );
				types.Add( val );
			}

			int outpos = 0;
			do
			{
				ushort vox;
				uint val;
				uint count;
				stream.Read( 1, out val );
				if( val == 0 )
				{
					stream.Read( bits, out val );
					vox = types[(int)val];
					result[outpos++] = vox;
				}
				else
				{
					uint count_tmp;
					stream.Read( 6, out count_tmp );
					count = count_tmp & 0x1F;
					if( ( count_tmp & 0x20 ) != 0 )
					{
						count <<= 5;
						stream.Read( 6, out count_tmp );
						count |= count_tmp & 0x1F;
						if( ( count_tmp & 0x20 ) != 0 )
						{
							count <<= 5;
							stream.Read( 6, out count_tmp );
							count |= count_tmp & 0x1F;
						}
					}
					count += 2;
					//stream.Read( 16, out count );
					//Log.log( "Read count {0}", count );
					stream.Read( bits, out val );
					vox = types[(int)val];
					for( n = 0; n < count; n++ )
						result[outpos++] = vox;
				}

			} while( outpos < 32768 );

		}

	}
}
