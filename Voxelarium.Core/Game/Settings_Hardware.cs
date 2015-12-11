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
using OpenTK.Input;
using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Game
{
	internal class Settings_Hardware
	{
		// unique identifier for this client
		// determines Identity for network play.
		internal static Guid ClientID;
		internal static string MasterServerHostname;
		internal static int MasterServerPort;

		// Resolution and screen
		internal static uint Setting_Resolution_h;
		internal static uint Setting_Resolution_v;
		internal static uint Setting_ViewPort_Offset_x;
		internal static uint Setting_ViewPort_Offset_y;
		internal static uint Setting_ViewPort_Size_x;
		internal static uint Setting_ViewPort_Size_y;
		internal static bool Setting_FullScreen;
		// Sound
		internal static bool Setting_SoundEnabled;
		internal static float Setting_SoundVolume;

		// Mouse
		internal static float Setting_MouseFactor;

		// Keyboard
		internal static Key Setting_Key_MoveForward;
		internal static Key Setting_Key_MoveBackward;
		internal static Key Setting_Key_MoveLeft;
		internal static Key Setting_Key_MoveRight;
		internal static Key Setting_Key_MoveUp;
		internal static Key Setting_Key_MoveDown;
		internal static Key Setting_Key_Jump;
		internal static Key Setting_Key_Inventory;

		// Graphic quality;

		internal static uint RenderingDistance_Horizontal;
		internal static uint RenderingDistance_Vertical;
		internal static float Opt_SectCFactor; // Sector Rendering Culling Factor.
		internal static float PixelAspectRatio;

		// Options for special gaming modes

		internal static bool Experimental_LearningMode;

		static Settings_Hardware()
		{
			ClientID = new Guid();
			Setting_Resolution_h = 0;
			Setting_Resolution_v = 0;
			Setting_FullScreen = true;
			Setting_ViewPort_Offset_x = 0;
			Setting_ViewPort_Offset_y = 0;
			Setting_ViewPort_Size_x = 0;
			Setting_ViewPort_Size_y = 0;
			Setting_SoundEnabled = true;
			Setting_SoundVolume = 100.0f;
			Setting_MouseFactor = 2.5f;
			Setting_Key_MoveForward = Key.W;
			Setting_Key_MoveBackward = Key.S;
			Setting_Key_MoveLeft = Key.A;
			Setting_Key_MoveRight = Key.D;
			Setting_Key_MoveUp = Key.Space;
			Setting_Key_MoveDown = Key.AltLeft;
			Setting_Key_Jump = Key.Space;
			Setting_Key_Inventory = Key.I;

#if BUILD_ANDROID
			RenderingDistance_Horizontal = 3;
			RenderingDistance_Vertical = 2;
#else
			RenderingDistance_Horizontal = 5;
			RenderingDistance_Vertical = 3;
#endif
			Opt_SectCFactor = 1.0f;
			PixelAspectRatio = 1.0f;
			Experimental_LearningMode = false;
		}

		internal static bool Load()
		{
			ClientID = Settings.Read( "Client ID", ClientID );
			MasterServerHostname = Settings.Read( "Master Server Address", "d3x0r.org" );
			MasterServerPort = Settings.Read( "Master Server Port", 31732 );

			//Settings.Read( "Setting_Resolution_h" ) Setting_Resolution_h = Line.GetULong();
			//			Settings.Read( "Setting_Resolution_v" ) Setting_Resolution_v = Line.GetULong();
			Setting_FullScreen = Settings.Read( "Setting_FullScreen", false );

			//Setting_ViewPort_Offset_x = Settings.Read( "Setting_ViewPort_Offset_x" ) Line.GetULong();
			//Setting_ViewPort_Offset_y = Settings.Read( "Setting_ViewPort_Offset_y" ) Line.GetULong();
			//Setting_ViewPort_Size_x = Settings.Read( "Setting_ViewPort_Size_x" ) Line.GetULong();
			//Settings.Read( "Setting_ViewPort_Size_y" ) Setting_ViewPort_Size_y = Line.GetULong();
			Setting_SoundEnabled = Settings.Read( "Setting_Sound_Enabled", true );

			Setting_SoundVolume = Settings.Read( "Setting_Sound_Volume", 80.0f );
			if( Setting_SoundVolume < 0.0 || Setting_SoundVolume > 100.0 ) Setting_SoundVolume = 100.0f;
			Setting_MouseFactor = Settings.Read( "Setting_Mouse_Factor", Setting_MouseFactor );

			Setting_Key_MoveForward = Settings.Read( "Setting_Key_MoveForward", Setting_Key_MoveForward );
			Setting_Key_MoveBackward = Settings.Read( "Setting_Key_MoveBackward", Setting_Key_MoveBackward );
			Setting_Key_MoveLeft = Settings.Read( "Setting_Key_MoveLeft", Setting_Key_MoveLeft );
			Setting_Key_MoveRight = Settings.Read( "Setting_Key_MoveRight", Setting_Key_MoveRight );
			Setting_Key_MoveUp = Settings.Read( "Setting_Key_MoveUp", Setting_Key_MoveUp );
			Setting_Key_MoveDown = Settings.Read( "Setting_Key_MoveDown", Setting_Key_MoveDown );

			Setting_Key_Jump = Settings.Read( "Setting_Key_Jump", Setting_Key_Jump );
			Setting_Key_Inventory = Settings.Read( "Setting_Key_Inventory", Setting_Key_Inventory );

			RenderingDistance_Horizontal = (uint)Settings.Read( "RenderingDistance_Horizontal", RenderingDistance_Horizontal );
			{
				if( RenderingDistance_Horizontal < 1 ) RenderingDistance_Horizontal = 1;
				if( RenderingDistance_Horizontal > 65535 ) RenderingDistance_Horizontal = 65535;
			}
			RenderingDistance_Vertical = (uint)Settings.Read( "RenderingDistance_Vertical", RenderingDistance_Vertical );
			{
				if( RenderingDistance_Vertical < 1 ) RenderingDistance_Vertical = 1;
				if( RenderingDistance_Vertical > 65535 ) RenderingDistance_Vertical = 65535;
			}
			Opt_SectCFactor = Settings.Read( "Opt_SectCFactor", 0.0f );
			{
				if( Opt_SectCFactor < 0.5 ) Opt_SectCFactor = 0.5f;
				if( Opt_SectCFactor > 10.0 ) Opt_SectCFactor = 10.0f;
			}

			PixelAspectRatio = Settings.Read( "PixelAspectRatio", 0.0f );
			{
				if( PixelAspectRatio < 0.5 ) PixelAspectRatio = 0.5f;
				if( PixelAspectRatio > 3.0 ) PixelAspectRatio = 3.0f;
			}
			Experimental_LearningMode = Settings.Read( "Experimental_LearningMode", Experimental_LearningMode );

			return ( true );
		}

		internal bool Save()
		{
			Settings.Write( "Setting_Resolution_h", Setting_Resolution_h );
			Settings.Write( "Setting_Resolution_v", Setting_Resolution_v );
			Settings.Write( "Setting_FullScreen", Setting_FullScreen ? 1u : 0u );
			Settings.Write( "Setting_ViewPort_Offset_x", Setting_ViewPort_Offset_x );
			Settings.Write( "Setting_ViewPort_Offset_y", Setting_ViewPort_Offset_y );
			Settings.Write( "Setting_ViewPort_Size_x", Setting_ViewPort_Size_x );
			Settings.Write( "Setting_ViewPort_Size_y", Setting_ViewPort_Size_y );
			Settings.Write( "Setting_Sound_Enabled", Setting_SoundEnabled );
			Settings.Write( "Setting_Sound_Volume", Setting_SoundVolume );
			Settings.Write( "Setting_Mouse_Factor", Setting_MouseFactor );
			Settings.Write( "Setting_Key_MoveForward", Setting_Key_MoveForward );
			Settings.Write( "Setting_Key_MoveBackward", Setting_Key_MoveBackward );
			Settings.Write( "Setting_Key_MoveLeft", Setting_Key_MoveLeft );
			Settings.Write( "Setting_Key_MoveRight", Setting_Key_MoveRight );
			Settings.Write( "Setting_Key_MoveUp", Setting_Key_MoveUp );
			Settings.Write( "Setting_Key_MoveDown", Setting_Key_MoveDown );
			Settings.Write( "Setting_Key_Jump", Setting_Key_Jump );
			Settings.Write( "Setting_Key_Inventory", Setting_Key_Inventory );
			Settings.Write( "RenderingDistance_Horizontal", RenderingDistance_Horizontal );
			Settings.Write( "RenderingDistance_Vertical", RenderingDistance_Vertical );
			Settings.Write( "Opt_SectCFactor", Opt_SectCFactor );
			Settings.Write( "PixelAspectRatio", PixelAspectRatio );
			Settings.Write( "Experimental_LearningMode", Experimental_LearningMode );
			return true;
		}

	}
}
