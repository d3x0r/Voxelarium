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

using OpenTK;
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Game.GameWindows
{
	internal class GameWindow_Advertising : IDisposable
	{

		internal class DisplayEntry
		{
			internal string Message;
			internal Visibility Visibility;
			internal uint SoundToPlay;
			internal double DisplayTime;
			internal double MinimumDisplayTime;
		};


		string ActualText;
		FontFrame Frame_Text = new FontFrame();

		VoxelGameEnvironment GameEnv;
		bool Flag_Shown;

		LinkedList<DisplayEntry> DisplayFile = new LinkedList<DisplayEntry>();
		DisplayEntry ActualyDisplayedEntry;

		internal enum Visibility
		{ VISIBILITY_HIGH, VISIBILITY_MEDIUM, VISIBILITY_MEDLOW, VISIBILITY_LOW, VISIBILITY_VERYLOW, VISIBILITY_VERYHARDTOREAD };


		internal GameWindow_Advertising()
		{
		}

		public void Dispose()
		{
			DisplayFile.Clear();
			ActualyDisplayedEntry = null;

		}

		~GameWindow_Advertising()
		{
			Dispose();
		}

		internal void SetGameEnv( VoxelGameEnvironment GameEnv ) { this.GameEnv = GameEnv; }

		internal void Advertise( string Text, Visibility Visibility, uint SoundToPlay, double DisplayTime, double MinimumDisplayTime )
		{
			DisplayEntry Entry;

			Entry = new DisplayEntry();
			Entry.Message = Text;
			Entry.Visibility = Visibility;
			Entry.SoundToPlay = SoundToPlay;
			Entry.DisplayTime = DisplayTime;
			Entry.MinimumDisplayTime = MinimumDisplayTime;

			DisplayFile.AddLast( Entry );
		}



		internal bool Is_Shown() { return ( Flag_Shown ); }
		internal void Show()
		{
			// Running position computing
			Flag_Shown = true;
		}

		internal void Hide()
		{
			Flag_Shown = false;
		}

		internal void Advertising_Actions( float FrameTime )
		{
			Vector2 Size;

			bool Stop;

			if( ActualyDisplayedEntry == null )
			{
				if( !Flag_Shown ) return;
				if( DisplayFile.First != null )
				{
					ActualyDisplayedEntry = DisplayFile.First.Value;
                    DisplayFile.RemoveFirst();

					if( ActualyDisplayedEntry.SoundToPlay != 0 )
						this.GameEnv.Sound.PlaySound( ActualyDisplayedEntry.SoundToPlay );
					switch( ActualyDisplayedEntry.Visibility )
					{

						default: break;
						case Visibility.VISIBILITY_HIGH:
							ActualText = ActualyDisplayedEntry.Message;
							Frame_Text.SetDisplayText( ActualText );
							Frame_Text.Font = GameEnv.default_font;
							Frame_Text.FontSize = 2.0f / 40;
							Frame_Text.GetTextDisplaySize( out Size );
							if( Size.X > 2.0f - 0.125f ) Size.X = 2.0f - 0.125f;
							Frame_Text.SetPosition( ( 2.0f - Size.X ) / 2.0f, ( 2.0f / 4.0f ) - ( Size.Y / 2.0f ) );
							// Frame_Text.SetPosition( (2.0f - Size.X) / 2.0f , (2.0f / 2.5f) - (Size.Y / 2.0f) );
							Frame_Text.SetSize( Size.X, Size.Y );
							GameEnv.GuiManager.AddFrame( Frame_Text );
							break;
						case Visibility.VISIBILITY_MEDIUM:
							ActualText = ActualyDisplayedEntry.Message;
							Frame_Text.SetDisplayText( ActualText );
							Frame_Text.Font = GameEnv.default_font;
							Frame_Text.FontSize = 2.0f / 20;
							Frame_Text.GetTextDisplaySize( out Size );
							if( Size.X > 2.0f - 0.125f ) Size.X = 2.0f - 0.125f;
							Frame_Text.SetPosition( ( 2.0f - Size.X ) / 2.0f, ( 2.0f / 4.0f ) - ( Size.Y / 2.0f ) );
							Frame_Text.SetSize( Size.X, Size.Y );
							GameEnv.GuiManager.AddFrame( Frame_Text );
							break;
						case Visibility.VISIBILITY_MEDLOW:
							ActualText = ActualyDisplayedEntry.Message;
							Frame_Text.SetDisplayText( ActualText );
							Frame_Text.Font = GameEnv.default_font;
							Frame_Text.FontSize = 2.0f / 20;
							Frame_Text.GetTextDisplaySize( out Size );
							if( Size.X > 2.0f - 0.125f ) Size.X = 2.0f - 0.125f;
							Frame_Text.SetPosition( ( 2.0f - Size.X ) / 2.0f, ( 2.0f * 0.75f ) - ( Size.Y / 2.0f ) );
							Frame_Text.SetSize( Size.X, Size.Y );
							GameEnv.GuiManager.AddFrame( Frame_Text );
							break;
						case Visibility.VISIBILITY_LOW:
							ActualText = ActualyDisplayedEntry.Message;
							Frame_Text.SetDisplayText( ActualText );
							Frame_Text.Font = GameEnv.default_font;
							Frame_Text.FontSize = 2.0f / 30;
							Frame_Text.GetTextDisplaySize( out Size );
							if( Size.X > 2.0f - 0.125f ) Size.X = 2.0f - 0.125f;
							Frame_Text.SetPosition( ( 2.0f - Size.X ) / 2.0f, ( 2.0f * 0.75f ) - ( Size.Y / 2.0f ) );
							Frame_Text.SetSize( Size.X, Size.Y );
							GameEnv.GuiManager.AddFrame( Frame_Text );
							break;
						case Visibility.VISIBILITY_VERYLOW:
							ActualText = ActualyDisplayedEntry.Message;
							Frame_Text.SetDisplayText( ActualText );
							Frame_Text.Font = GameEnv.default_font;
							Frame_Text.FontSize = 2.0f / 30;
							Frame_Text.GetTextDisplaySize( out Size );
							if( Size.X > 2.0f - 0.125f ) Size.X = 2.0f - 0.125f;
							Frame_Text.SetPosition( ( 2.0f - Size.X ) / 2.0f, ( 2.0f * 0.95f ) - ( Size.Y / 2.0f ) );
							Frame_Text.SetSize( Size.X, Size.Y );
							GameEnv.GuiManager.AddFrame( Frame_Text );
							break;
						case Visibility.VISIBILITY_VERYHARDTOREAD:
							ActualText = ActualyDisplayedEntry.Message;
							Frame_Text.SetDisplayText( ActualText );
							Frame_Text.Font = GameEnv.default_font;
							Frame_Text.FontSize = 2.0f / 80;
							Frame_Text.GetTextDisplaySize( out Size );
							if( Size.X > 2.0f - 0.125f ) Size.X = 2.0f - 0.125f;
							Frame_Text.SetPosition( ( 2.0f - Size.X ) / 2.0f, ( 2.0f * 0.95f ) - ( Size.Y / 2.0f ) );
							Frame_Text.SetSize( Size.X, Size.Y );
							GameEnv.GuiManager.AddFrame( Frame_Text );
							break;
					}
				}
			}
			else
			{
				ActualyDisplayedEntry.DisplayTime -= FrameTime;
				ActualyDisplayedEntry.MinimumDisplayTime -= FrameTime;

				Stop = false;
				if( !Flag_Shown ) Stop = true;
				else if( ActualyDisplayedEntry.DisplayTime < 0.0 ) Stop = true;
				else if( ( ActualyDisplayedEntry.MinimumDisplayTime < 0.0 ) && DisplayFile.Count > 2 ) Stop = true;

				if( Stop )
				{
					GameEnv.GuiManager.RemoveFrame( Frame_Text );
					ActualText = null;
					ActualyDisplayedEntry = null;
				}


			}
		}

		internal void Clear()
		{
			if( ActualyDisplayedEntry != null )
			{
				GameEnv.GuiManager.RemoveFrame( Frame_Text );
				ActualText = null;
				ActualyDisplayedEntry = null;
			}
			DisplayFile.Clear();
		}
	}
}
