using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels.IO
{
	internal class SectorTagHash
	{
		const int Tag_Size_x = 256;
        const int Tag_Size_y = 16;
            const int Tag_Size_z = 256;
            const int Tag_Mask_x = 0xff;
            const int Tag_Mask_y = 0xf;
            const int Tag_Mask_z = 0xff;
            const int Tag_Shift_x = 8;
            const int Tag_Shift_y = 4;
            const int Tag_Shift_z = 8;

		internal class TagEntry
		{
			internal TagEntry Next;
			internal TagEntry Prev;

			internal int x, y, z;
		};

		TagEntry[] TagHash;
		uint TagHashSize;

		//static ZMonoSizeMemoryPool DefaultMemoryPool;
		//ZMemoryPool* MemPool;

		internal SectorTagHash()
		{
			TagHashSize = Tag_Size_x * Tag_Size_y * Tag_Size_z;
			TagHash = new TagEntry[TagHashSize];
		}

		~SectorTagHash()
		{
			int i;
			TagEntry Entry, NewEntry;

			for( i = 0; i < TagHashSize; i++ )
			{
				Entry = TagHash[i];
				while( Entry != null )
				{
					Entry.Prev = null;
					NewEntry = Entry.Next;
					Entry.Next = null;
					Entry = NewEntry;
				}
				TagHash[i] = null;
			}

			TagHash = null;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal bool find( int x, int y, int z )
		{
			TagEntry Entry;
			uint Offset;

			Offset = ( (uint)x & Tag_Mask_x )
					+ ( ( (uint)y << Tag_Shift_x ) & Tag_Mask_y )
					+ ( ( (uint)z << ( Tag_Shift_x + Tag_Shift_y ) ) & Tag_Mask_z );

			Entry = TagHash[Offset];
			while( Entry != null )
			{
				if( ( Entry.x == x ) && ( Entry.y == y ) && ( Entry.z == z ) ) return ( true );
				Entry = Entry.Next;
			}
			return ( false );

		}

		internal void Add( int x, int y, int z )
		{
			TagEntry Entry;
			uint Offset;

			// Offset = x + (y << Tag_Shift_x) + (z << (Tag_Shift_x + Tag_Shift_y));

			Offset = ( (uint)x & Tag_Mask_x )
					+ ( ( (uint)y << Tag_Shift_x ) & Tag_Mask_y )
					+ ( ( (uint)z << ( Tag_Shift_x + Tag_Shift_y ) ) & Tag_Mask_z );

			Entry = TagHash[Offset];
			while( Entry != null )
			{
				if( ( Entry.x == x ) && ( Entry.y == y ) && ( Entry.z == z ) ) return;
				Entry = Entry.Next;
			}

			Entry = new TagEntry();

			Entry.x = x; Entry.y = y; Entry.z = z;

			Entry.Prev = null;
			if( ( Entry.Next = TagHash[Offset] ) != null ) Entry.Next.Prev = Entry;
			TagHash[Offset] = Entry;
		}

		internal void Remove( int x, int y, int z )
		{
			TagEntry Entry;
			uint Offset;

			//Offset = x + (y << Tag_Shift_x) + (z << (Tag_Shift_x + Tag_Shift_y));
			Offset = ( (uint)x & Tag_Mask_x )
					+ ( ( (uint)y << Tag_Shift_x ) & Tag_Mask_y )
					+ ( ( (uint)z << ( Tag_Shift_x + Tag_Shift_y ) ) & Tag_Mask_z );

			Entry = TagHash[Offset];
			while( Entry != null )
			{
				if( ( Entry.x == x ) && ( Entry.y == y ) && ( Entry.z == z ) )
				{
					if( Entry.Prev == null ) TagHash[Offset] = Entry.Next;
					else Entry.Prev.Next = Entry.Next;
					if( Entry.Next != null ) Entry.Next.Prev = Entry.Prev;

					Entry.Next = null;
					Entry.Prev = null;

					return;
				}
				Entry = Entry.Next;
			}
		}
	}
}
