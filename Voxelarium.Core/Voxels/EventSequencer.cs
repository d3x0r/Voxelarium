/*
 * Before porting, this header appeared inmost sources.  Of course
 * the change from C++ to C# required significant changes an no part
 * is entirely original.
 * 
 * This file is part of Blackvoxel. (Now Voxelarium)
 *
 * Copyright 2010-2014 Laurent Thiebaut & Olivia Merle
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
using Voxelarium.Common;
using Voxelarium.Core.Support;

namespace Voxelarium.Core.Voxels
{
	internal class EventSequencer
	{
		const int ZGAMEEVENTSEQUENCER_SLOTCOUNT = 128;

		long GameTime;

		internal class Event
		{
			public long StartTime;
			public long EndTime;
			public long InitialDelay;
			public long RepeatDelay;
			public long Duration;
			public uint SlotNum;
			public bool AutoRepeat;
		};

		internal struct Slot
		{
			internal bool Triggered;
			internal bool EventOn;
			internal LinkedListNode<Event> Item;
		};

		LinkedList<Event> EventList;
		Slot[] SlotTable = new Slot[ZGAMEEVENTSEQUENCER_SLOTCOUNT];

		internal EventSequencer()
		{
			uint i;

			GameTime = 0;
			for( i = 0; i < ZGAMEEVENTSEQUENCER_SLOTCOUNT; i++ ) { SlotTable[i].EventOn = false; SlotTable[i].Triggered = false; }
		}

		void SetGameTime( long GameTime ) { this.GameTime = GameTime; }

		LinkedListNode<Event> AddEvent( long InHowMuchTime, long Duration, uint SlotToSet, bool AutoRepeat = false, long RepeatDelay = 0 )
		{
			Event Evn;

			Evn = new Event();
			Evn.SlotNum = SlotToSet;
			Evn.StartTime = GameTime + InHowMuchTime;
			Evn.EndTime = Evn.StartTime + Duration;
			Evn.InitialDelay = InHowMuchTime;
			Evn.Duration = Duration;
			Evn.AutoRepeat = AutoRepeat;
			if( RepeatDelay != 0 ) Evn.RepeatDelay = RepeatDelay;
			else Evn.RepeatDelay = Duration;
			SlotTable[SlotToSet].EventOn = true;
			SlotTable[SlotToSet].Triggered = false;
			return ( SlotTable[SlotToSet].Item = EventList.AddLast( Evn ) );
		}

		bool ProcessEvents( long TotalElapsedGameTime )
		{
			LinkedListNode<Event> Item, NextItem;
			Event Evn;

			GameTime = TotalElapsedGameTime / 1000;
			// printf("GameTime : %ld\n",GameTime);
			Item = EventList.First;
			while( Item != null )
			{
				NextItem = Item.Next;
				Evn = Item.Value;
				if( GameTime > Evn.StartTime )
				{
					if( !SlotTable[Evn.SlotNum].Triggered ) Log.log( "Start Event : {0} at : {1}", Evn.SlotNum, GameTime);
					SlotTable[Evn.SlotNum].Triggered = true;
				}
				if( GameTime > Evn.EndTime )
				{
					Log.log( "End Event : {0} at : {1}", Evn.SlotNum, ( long )GameTime);
					SlotTable[Evn.SlotNum].Triggered = false;

					// If event is auto restartable, restart it. Else kill it.
					if( Evn.AutoRepeat )
					{
						Evn.StartTime = GameTime + Evn.RepeatDelay;
						Evn.EndTime = Evn.StartTime + Evn.Duration;
					}
					else
					{
						SlotTable[Evn.SlotNum].EventOn = false;
						EventList.Remove( Item );
					}
				}
				Item = NextItem;
			}
			return ( true );
		}


		internal bool SlotIsActivated( uint SlotNum ) { return ( SlotTable[SlotNum].Triggered ); }
		internal bool SlotIsEventAttached( uint SlotNum ) { return ( SlotTable[SlotNum].EventOn ); }


	}
}
