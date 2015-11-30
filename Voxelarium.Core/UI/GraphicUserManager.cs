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
using OpenTK.Input;
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.Voxels;
using Voxelarium.Core.Voxels.Types;
using Voxelarium.Common;

namespace Voxelarium.Core.UI
{

	internal class GraphicUserManager : EventConsumer
	{
		internal Frame FirstFrame;  // this is the phsyical display (glass)
		internal TextureManager TextureManager;
		internal EventManager EventManager;


		// Mouse over imbrication stack tables

		internal Frame[] Actual_PointedStack;
		internal Frame[] Previous_PointedStack;
		internal Frame[] Actual_FocusStack;
		internal Frame[] Previous_FocusStack;

		internal uint Actual_StackSize;
		internal uint Previous_StackSize;
		internal uint Actual_FocusStackSize;
		internal uint Previous_FocusStackSize;

		// Draging support

		internal Frame DragingFrame;


		//#define SclX(a) (GameEnv.ScreenResolution.X * ((a)/1920.0))
		//#define SclY(a) (GameEnv.ScreenResolution.Y * ((a)/1080.0))




		internal GraphicUserManager()
		{
			TextureManager = null;
			FirstFrame = new Frame();
			FirstFrame.GuiManager = this;
			FirstFrame.ParentFrame = null;
			FirstFrame.Show_Frame( false );

			// Init MouseOver stack imbrication tables
			Actual_PointedStack = new Frame[1024];
			Previous_PointedStack = new Frame[1024];
			Actual_FocusStack = new Frame[1024];
			Previous_FocusStack = new Frame[1024];
			Actual_StackSize = 0;
			Previous_StackSize = 0;
			Actual_FocusStackSize = 0;
			Previous_FocusStackSize = 0;
			DragingFrame = null;
		}
		~GraphicUserManager()
		{
			Actual_PointedStack = null;
			Previous_PointedStack = null;
			Actual_FocusStack = null;
			Previous_FocusStack = null;
			Actual_StackSize = 0;
			Previous_StackSize = 0;
			Actual_FocusStackSize = 0;
			Previous_FocusStackSize = 0;
		}



		// Mouse over stack functions. These are not for internal use.

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal void MouseOverStack_AddToStack( Frame Frame ) { if( Actual_StackSize >= 1024 ) return; Actual_PointedStack[Actual_StackSize++] = Frame; }

		void MouseOverStack_SwapAndReset()
		{
			Frame[] tmp;
			tmp = Actual_PointedStack;
			Actual_PointedStack = Previous_PointedStack;
			Previous_PointedStack = tmp;
			Previous_StackSize = Actual_StackSize;
			Actual_StackSize = 0;
		}
		void FocusStack_SwapAndReset()
		{
			Frame[] tmp;
			tmp = Actual_FocusStack;
			Actual_FocusStack = Previous_FocusStack;
			Previous_FocusStack = tmp;
			Previous_FocusStackSize = Actual_FocusStackSize;
			Actual_FocusStackSize = 0;
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		internal void FocusStack_AddToStack( Frame Frame ) { if( Actual_FocusStackSize >= 1024 ) return; Actual_FocusStack[Actual_FocusStackSize++] = Frame; }


		internal void SetTextureManager( TextureManager TextureManager )
		{
			this.TextureManager = TextureManager;
		}
		internal void SetEventManager( EventManager EventManager )
		{
			this.EventManager = EventManager;
		}


		internal void SetScreenDimensions( float OriginX, float OriginY, float Width, float Height )
		{
			FirstFrame.SetPosition( OriginX, OriginY );
			FirstFrame.SetSize( Width, Height );
		}

		internal void AddFrame( Frame Frame )
		{
			FirstFrame.AddFrame( Frame );
		}

		internal void RemoveFrame( Frame Frame )
		{
			FirstFrame.RemoveFrame( Frame );
		}

		internal void RemoveAllFrames() // Clear Screen
		{
			FirstFrame.RemoveAllFrames();
			Actual_StackSize = 0;
			Previous_StackSize = 0;
			Actual_FocusStackSize = 0;
			Previous_FocusStackSize = 0;
		}

		internal void Render( Display render )
		{
			Box Dimensions;

			Dimensions.Position.X = -1;
			Dimensions.Position.Y = -1;
			Dimensions.Position.Z = 1;
			Dimensions.Size.X = 2;
			Dimensions.Size.Y = 2;
			Dimensions.Size.Z = 10;
			FirstFrame.Render( render, ref Dimensions );
		}


		// Event Input

		public bool KeyDown( Key KeySym )
		{
			uint i;

			for( i = 0; i < this.Actual_FocusStackSize; i++ )
			{
				if( Actual_FocusStack[i].Flag_Enable_Proc_KeyDown ) Actual_FocusStack[i].KeyDown( KeySym );
			}
			// keys are used alater...
			return ( false );
		}
		public bool KeyUp( Key KeySym )
		{
			uint i;

			for( i = 0; i < this.Actual_FocusStackSize; i++ )
			{
				if( Actual_FocusStack[i].Flag_Enable_Proc_KeyDown ) Actual_FocusStack[i].KeyUp( KeySym );
			}

			return ( false );
		}
		public bool MouseMove( float Relative_x, float Relative_y, float Absolute_x, float Absolute_y )
		{
			bool res;
			uint i;

			// Frame Draging

			if( DragingFrame != null )
			{
				Vector2 Dim;
				DragingFrame.GetSize( out Dim.X, out Dim.Y );
				DragingFrame.SetDragPosition( Absolute_x - Dim.X / 2.0f, Absolute_y - Dim.Y / 2.0f );
			}

			//

			MouseOverStack_SwapAndReset();
			res = FirstFrame.MouseMove( Relative_x, Relative_y, Absolute_x, Absolute_y );

			// MouseOut Event.

			//printf("As:"); for (z=0;z<Actual_StackSize;z++) printf(" %lx",(long unsigned int)Actual_PointedStack[z]); printf(" ");
			//printf("Pr:"); for (z=0;z<Previous_StackSize;z++) printf(" %lx",(long unsigned int)Previous_PointedStack[z]); printf("\n");

			for( i = 0; i < Previous_StackSize; i++ )
			{
				if( ( Actual_StackSize <= i ) || ( Actual_PointedStack[i] != Previous_PointedStack[i] ) )
				{
					Previous_PointedStack[i].Event_MouseIsActualyOver = false;
					Previous_PointedStack[i].Event_MouseWentOut = true;
					if( Previous_PointedStack[i].Flag_Enable_Proc_MouseOut ) Previous_PointedStack[i].MouseOut();

					//printf("ActualStack:"); for (z=0;z<Actual_StackSize;z++) printf(" %lx",(long unsigned int)Actual_PointedStack[z]); printf("\n");
					//printf("PrevioStack:");         for (z=0;z<Previous_StackSize;z++) printf(" %lx",(long unsigned int)Previous_PointedStack[z]); printf("\n");

				}
			}

			return ( res );
		}
		public bool MouseButtonClick( MouseButton nButton, float Absolute_x, float Absolute_y )
		{
			int i;

			FocusStack_SwapAndReset();
			FirstFrame.MouseButtonClick( nButton, Absolute_x, Absolute_y );

			for( i = 0; i < this.Previous_FocusStackSize; i++ )
			{
				if( ( Actual_FocusStackSize <= i ) || ( Actual_FocusStack[i] != Previous_FocusStack[i] ) )
				{
					Previous_FocusStack[i].Event_LostFocus = true;
					if( Previous_FocusStack[i].Flag_Enable_Proc_LostFocus ) Previous_FocusStack[i].LostFocus();

					//printf("ActualStack:"); for (z=0;z<Actual_StackSize;z++) printf(" %lx",(long unsigned int)Actual_PointedStack[z]); printf("\n");
					//printf("PrevioStack:");         for (z=0;z<Previous_StackSize;z++) printf(" %lx",(long unsigned int)Previous_PointedStack[z]); printf("\n");

				}
			}

			return ( true );
		}
		public bool MouseButtonRelease( MouseButton nButton, float Absolute_x, float Absolute_y )
		{

			FirstFrame.MouseButtonRelease( nButton, Absolute_x, Absolute_y );

			if( nButton == MouseButton.Button1 )
			{
				if( DragingFrame != null )
				{
					DragingFrame.Flag_Draging = false;
					DragingFrame = null;
				}
			}
			return true;
		}


	}
}
