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
