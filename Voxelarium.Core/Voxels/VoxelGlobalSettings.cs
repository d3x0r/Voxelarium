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
		public const int ZVOXEL_DRAWINFO_CULLINGBITS = ( ZVOXEL_DRAWINFO_NOTVOID | ZVOXEL_DRAWINFO_DRAWFULLVOXELOPACITY | ZVOXEL_DRAWINFO_DRAWTRANSPARENTRENDERING );

		public const int WorldVoxelBlockSizeBits = 5;
		public const int WorldVoxelBlockSize = 1 << WorldVoxelBlockSizeBits;

	}
}
