using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.UI
{
	internal abstract class Screen
	{
		protected VoxelGameEnvironment.Pages page_id;
		public delegate ScreenChoices Process( VoxelGameEnvironment Game );
		public enum ScreenChoices {
			SAME_SCREEN = -1,
			CHOICE_RETURN,
            MAIN_MENU,
			QUIT,
			OPTIONS,
			PLAYGAME,
			NONE
				, SlotChoice1
				, SlotChoice2
				, SlotChoice3
				, SlotChoice4
				, SlotChoice5
				, SlotChoice6
				, SlotChoice7
				, SlotChoice8
				, SlotChoice9
				, SlotChoice10
				, SlotChoice11
				, SlotChoice12
				, SlotChoice13
				, SlotChoice14
				, SlotChoice15
				, SlotChoice16
		};
		internal ScreenChoices ResultCode;
		internal Screen( VoxelGameEnvironment.Pages Page_id ) { page_id = Page_id; ResultCode = ScreenChoices.SAME_SCREEN; }
		internal abstract ScreenChoices ProcessScreen( VoxelGameEnvironment Game );

		// because coordindates are now -1 to 1 instead of 0 to X
		internal static float SclX( float x ) { return x / 1920; }
		internal static float SclY( float y ) { return y / 1080; }

	};

}
