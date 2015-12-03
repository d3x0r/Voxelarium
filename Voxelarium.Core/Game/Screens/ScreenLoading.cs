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
using Voxelarium.Core.UI;

namespace Voxelarium.Core.Game.Screens
{
	internal class ScreenLoading : Screen
	{
		FontFrame Frame_Loading;
		ProgressBar LoadProgress;
		internal ScreenLoading( VoxelGameEnvironment.Pages page_id ) : base( page_id )
		{
		}

		internal override ScreenChoices ProcessScreen( VoxelGameEnvironment GameEnv )
		{
            if( GameEnv.page_up != page_id )
			{
				GameEnv.page_up = page_id;
				GameEnv.GuiManager.RemoveAllFrames();

				if( Frame_Loading == null )
				{
					Frame_Loading = new FontFrame();
					Frame_Loading.SetDisplayText( "LOADING..." );
					Frame_Loading.Font = GameEnv.default_font;// SetStyle( GameEnv.TileSetStyles.GetStyle( 1 ) );
					Frame_Loading.FontSize = ( 2.0f / 10 );
					Vector2 Loading_Size;
					Frame_Loading.GetTextDisplaySize( out Loading_Size );
					Frame_Loading.SetPosition(  1-Loading_Size.X / 2.0f, 1-Loading_Size.Y / 2.0f );
					Frame_Loading.SetSize( Loading_Size.X + 1.0f, Loading_Size.Y );
					Frame_Loading.TextureNum = TextureID.OldFont;

					LoadProgress = new ProgressBar();
					LoadProgress.SetPosition( 0.5f, 0.5f );
					LoadProgress.SetSize( 1.0f, 0.2f );
				}

				//TitleBackground.AddFrame(&Frame_PlayGame);
				GameEnv.GuiManager.AddFrame( Frame_Loading );
				GameEnv.GuiManager.AddFrame( LoadProgress );
			}
			LoadProgress.SetCompletion( GameEnv.start_percent );
			return ( ScreenChoices.NONE );
		}
	}
}
