/*
 * This file is part of Voxelarium.
 * Contains Compile time constance from Blackvoxel origianl sources.
 * Implements a common place for constants.
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
#define DEVELOPPEMENT_ON
using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	public class VoxelGlobalSettings
	{
		public static uint MulticharConst( char a, char b, char c, char d ) { return ( (uint)a ) | ( (uint)b << 9 ) | ( (uint)c << 16 ) | ( (uint)d << 24 ); }

		public static bool Exiting;

		public const string COMPILEOPTION_VERSIONSTRING = "git-????";
		public const bool COMPILEOPTION_USEHOMEDIRSTORAGE = false;
		public const string COMPILEOPTION_SAVEFOLDERNAME = "Voxelarium";
		public const string COMPILEOPTION_DATAFILESPATH = "./Content"; // "/usr/share/Voxelarium"

		public const bool COMPILEOPTION_ALLOWSTARTINGSTORAGE = false; // Allow starting storage to be present. Never introduced in the game.
		public const bool COMPILEOPTION_ALLOWJUMPANDBUILD = true; // Jump and build allow building when floating in the air (Like when jumping).

		public const bool COMPILEOPTION_ALLOWSAVE = true;
#if DEVELOPPEMENT_ON
		public const bool COMPILEOPTION_FABDATABASEOUTPUT = false;
		public const bool COMPILEOPTION_BOUNDCHECKING = true;
		public const bool COMPILEOPTION_BOUNDCHECKINGSLOW = true;
		public const bool COMPILEOPTION_FALLAREFATALS = false;
		public const bool COMPILEOPTION_FINETIMINGTRACKING = false;
		public const bool COMPILEOPTION_DEBUGFACILITY = true;
		public const bool COMPILEOPTION_NOMOUSECAPTURE = true;
		public const bool COMPILEOPTION_VOXELPROCESSOR = true;
		public const bool COMPILEOPTION_DONTEMPTYINVENTORYONDEATH = true;
		public const bool COMPILEOPTION_ALPHA_SOUNDS_1 = true;
		public const bool COMPILEOPTION_FNX_SOUNDS_1 = true;
		public const bool COMPILEOPTION_LOWRESTEXTURING = false;
		public const bool COMPILEOPTION_VERBOSELOGS = false;
		public const bool COMPILEOPTION_SQUIRRELUNSAFE = true;
		public const bool COMPILEOPTION_SAVEONLYMODIFIED = true;
#else
		public const bool COMPILEOPTION_FABDATABASEOUTPUT = false;
		public const bool COMPILEOPTION_BOUNDCHECKING = false;
		public const bool COMPILEOPTION_BOUNDCHECKINGSLOW = false;
		public const bool COMPILEOPTION_FALLAREFATALS = true;
		public const bool COMPILEOPTION_FINETIMINGTRACKING = false;
		public const bool COMPILEOPTION_DEBUGFACILITY = false;
		public const bool COMPILEOPTION_NOMOUSECAPTURE = false;
		public const bool COMPILEOPTION_VOXELPROCESSOR = true;
		public const bool COMPILEOPTION_DONTEMPTYINVENTORYONDEATH = false;
		public const bool COMPILEOPTION_ALPHA_SOUNDS_1 = true;
		public const bool COMPILEOPTION_FNX_SOUNDS_1 = true;
		public const bool COMPILEOPTION_LOWRESTEXTURING = false;
		public const bool COMPILEOPTION_VERBOSELOGS = false;
		public const bool COMPILEOPTION_SQUIRRELUNSAFE = true;
		public const bool COMPILEOPTION_SAVEONLYMODIFIED = true;
#endif


		public const int ZVOXEL_DRAWINFO_VOID = 0;
		public const int ZVOXEL_DRAWINFO_NOTVOID = 1;
		public const int ZVOXEL_DRAWINFO_DRAWFULLVOXELOPACITY = 2;
		public const int ZVOXEL_DRAWINFO_DRAWTRANSPARENTRENDERING = 4;
		public const int ZVOXEL_DRAWINFO_SPECIALRENDERING = 8;
		public const int ZVOXEL_DRAWINFO_SHADER = 16;
		public const int ZVOXEL_DRAWINFO_DECAL = 32;  // image is used over shader output
		public const int ZVOXEL_DRAWINFO_CULLINGBITS = ( ZVOXEL_DRAWINFO_NOTVOID | ZVOXEL_DRAWINFO_DRAWFULLVOXELOPACITY | ZVOXEL_DRAWINFO_DRAWTRANSPARENTRENDERING );
        public const int WorldVoxelBlockSizeBits = 5;
		public const int WorldVoxelBlockSize = 1 << WorldVoxelBlockSizeBits;

	}
}
