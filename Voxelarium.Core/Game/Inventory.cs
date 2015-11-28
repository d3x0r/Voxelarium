/*
 * This file is part of Voxelarium.
 *
 * Copyright 2010-2014 Laurent Thiebaut & Olivia Merle
 * Copyright 2015-2016 James Buckeyne
 *
 * Blackvoxel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Blackvoxel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
 * ZInventory.cpp
 *
 *  Created on: 18 juin 2011
 *      Author: laurent
 *  Ported on 27 nov, 2015
 *      Porter: James Buckeyne
 */

using ProtoBuf;
using System.IO;

namespace Voxelarium.Core.Game
{
	class Inventory
	{
		
		[ProtoContract]
		internal struct Entry
		{
			[ProtoMember( 1 )]
			internal ushort VoxelType;
			[ProtoMember( 2 )]
			internal int Quantity;
		};

		internal int SlotCount;
		internal Entry[] SlotTable;

		internal int ActualItem;
		internal int ActualTool;

	    internal enum SlotType {
			Inventory_StartSlot = 0,
			Inventory_EndSlot = 39,
			Tools_StartSlot = 40,
			Tools_EndSlot = 49,
			Powers_StartSlot = 50,
			Powers_EndSlot = 59
		};

		internal Inventory()
		{
			int i;

			SlotCount = 60;

			ActualItem = (int)SlotType.Inventory_StartSlot;
			ActualTool = (int)SlotType.Tools_StartSlot;

			SlotTable = new Entry[SlotCount];

			for( i = 0; i < SlotCount; i++ ) { SlotTable[i].VoxelType = 0; SlotTable[i].Quantity = 0; }
		}

		internal void Clear()
		{
			int i;

			for( i = (int)SlotType.Inventory_StartSlot; i <= (int)SlotType.Inventory_EndSlot; i++ ) { SlotTable[i].VoxelType = 0; SlotTable[i].Quantity = 0; }
			for( i = (int)SlotType.Tools_StartSlot; i <= (int)SlotType.Tools_EndSlot; i++ ) { SlotTable[i].VoxelType = 0; SlotTable[i].Quantity = 0; }
			for( i = (int)SlotType.Powers_StartSlot; i <= (int)SlotType.Powers_EndSlot; i++ ) { SlotTable[i].VoxelType = 0; SlotTable[i].Quantity = 0; }
		}

		internal void SetSlot( int SlotNum, ushort Type, int Quantity )
		{
			if( SlotNum >= SlotCount ) return;

			SlotTable[SlotNum].VoxelType = Type;
			SlotTable[SlotNum].Quantity = Quantity;
		}

		internal int GetSlotRef( int SlotNum ) { return SlotNum; }

		internal int GetActualItemSlot() { return ActualItem; }
		internal int GetActualToolSlot() { return ActualTool; }

		internal Inventory( int SlotCount )
		{
			int i;

			ActualItem = 0;
			ActualTool = 0;

			this.SlotCount = SlotCount;
			SlotTable = new Entry[SlotCount];
			for( i = 0; i < SlotCount; i++ ) { SlotTable[i].VoxelType = 0; SlotTable[i].Quantity = 0; }
		}

		~Inventory()
		{
			SlotTable = null;
			SlotCount = 0;
		}

		internal bool FindSlot( ushort VoxelType, out int Slot)
		{
			int i;

			for( i = 0; i < SlotCount; i++ )
			{
				if( SlotTable[i].VoxelType == VoxelType && SlotTable[i].Quantity > 0 ) { Slot = i; return ( true ); }
			}
			Slot = -1;
			return ( false );
		}

		internal bool FindFreeSlot( out int Slot)
		{
			int i;

			for( i = 0; i < SlotCount; i++ )
			{
				if( SlotTable[i].Quantity == 0 ) { Slot = i; return ( true ); }
			}
			Slot = -1;
			return ( false );
		}

		internal bool StoreBlocks( ushort VoxelType, int VoxelQuantity )
		{
			int Slot;

			if( FindSlot( VoxelType, out Slot ) )
			{
				SlotTable[Slot].Quantity += VoxelQuantity;
				return ( true );
			}
			else if( FindFreeSlot( out Slot ) )
			{
				SlotTable[Slot].VoxelType = VoxelType;
				SlotTable[Slot].Quantity = VoxelQuantity;
				return ( true );
			}

			return ( false );
		}

		internal int UnstoreBlocks( ushort VoxelType, int VoxelQuantity )
		{
			int i, UnstoredCount, Quantity;
			UnstoredCount = 0;
			for( i = (int)SlotType.Inventory_StartSlot; i <= (int)SlotType.Inventory_EndSlot; i++ )
			{
				if( SlotTable[i].VoxelType == VoxelType )
				{
					Quantity = ( SlotTable[i].Quantity >= VoxelQuantity ) ? VoxelQuantity : SlotTable[i].Quantity;
					UnstoredCount += Quantity;
					SlotTable[i].Quantity -= Quantity;
					if( SlotTable[i].Quantity == 0 ) SlotTable[i].VoxelType = 0;
				}
				if( UnstoredCount == VoxelQuantity ) return ( UnstoredCount );
			}
			return ( UnstoredCount );
		}

		void Select_NextItem() { ActualItem++; if( ActualItem > (int)SlotType.Inventory_EndSlot ) ActualItem = (int)SlotType.Inventory_StartSlot; }
		void Select_PreviousItem() { if( ActualItem == (int)SlotType.Inventory_StartSlot ) ActualItem = (int)SlotType.Inventory_EndSlot; else ActualItem--; }
		int GetActualItemSlotNum() { return ( ActualItem ); }
		void SetActualItemSlotNum( int SlotNum ) { ActualItem = SlotNum; }
		int GetActualToolSlotNum() { return ( ActualTool ); }

		int GetNextUsedItemSlotNum( int SlotNum )
		{
			int i;

			if( SlotNum == - 1 ) return ( - 1 );

			i = SlotNum;
			while( i != SlotNum )
			{
				i++;
				if( i > (int)SlotType.Inventory_EndSlot ) i = (int)SlotType.Inventory_StartSlot;
				if( SlotTable[i].VoxelType > 0 && SlotTable[i].Quantity > 0 ) return ( i );
			}
			return ( -1 );
		}

		int GetPreviousUsedItemSlotNum( int SlotNum )
		{
			int i;

			if( SlotNum ==  - 1 ) return ( - 1 );
			i = SlotNum;
			while( --i != SlotNum )
			{
				if( i < (int)SlotType.Inventory_StartSlot ) i = (int)SlotType.Inventory_EndSlot;
				if( SlotTable[i].VoxelType > 0 && SlotTable[i].Quantity > 0 ) return ( (int)i );
			}
			return ( -1 );
		}

		bool Save( Stream Stream )
		{
			Serializer.Serialize( Stream, SlotTable );
			return ( true );
		}

		bool Load( Stream stream )
		{
			SlotTable = Serializer.Deserialize<Entry[]>( stream );
			return true;
		}



	}
}
