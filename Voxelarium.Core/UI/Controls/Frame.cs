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
using System.Collections.Generic;
using Voxelarium.Core.Voxels;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.UI
{
	public class Frame
	{
		internal uint FrameType;
		internal LinkedList<Frame> SubFrameList = new LinkedList<Frame>();
		internal TextureID TextureNum;
		internal Vector4 DrawColor;
		internal GraphicUserManager GuiManager;
		internal Frame ParentFrame;
		internal bool Flag_Show_Frame;
		internal bool Flag_Show_Childs;
		internal bool Flag_Show_Master;
		internal bool Flag_Cap_Dragable;
		internal bool Flag_Draging;

		internal Box Dimensions;
		internal Box EffectivePosition;
		internal Vector2 DragAbsolutePosition;

		// Event Flags

		internal bool Event_MouseWentIn; // Mouse Went over.
		internal bool Event_MouseWentOut;
		internal bool Event_MouseIsActualyOver;  // Mouse is actualy over.
		internal bool Event_MouseClick;
		internal bool Event_MouseRelease;
		internal bool Event_GetFocus;
		internal bool Event_LostFocus;

		// Event Callback enable

		internal bool Flag_Enable_Proc_MouseIn;
		internal bool Flag_Enable_Proc_MouseOut;
		internal bool Flag_Enable_Proc_MouseOver;
		internal bool Flag_Enable_Proc_GetFocus;
		internal bool Flag_Enable_Proc_LostFocus;
		internal bool Flag_Enable_Proc_KeyDown;
		internal bool Flag_Enable_Proc_KeyUp;

		//bool  Flag
		// Event Query
		internal bool Is_MouseOver() { return ( Event_MouseIsActualyOver ); }
		internal bool Is_MouseIn( bool Reset = true ) { bool res; res = Event_MouseWentIn; if( Reset ) Event_MouseWentIn = false; return ( res ); }
		internal bool Is_MouseOut( bool Reset = true ) { bool res; res = Event_MouseWentOut; if( Reset ) Event_MouseWentOut = false; return ( res ); }
		internal bool Is_MouseClick( bool Reset = true ) { bool res; res = Event_MouseClick; if( Reset ) Event_MouseClick = false; return ( res ); }
		internal bool Is_MouseRelease( bool Reset = true ) { bool res; res = Event_MouseRelease; if( Reset ) Event_MouseRelease = false; return ( res ); }


		internal Frame()
		{
			FrameType = VoxelUtils.MulticharConst( 'B', 'F', 'R', 'M' ); // = BaseFRaMe
			Flag_Cap_Dragable = false;
			Flag_Draging = false;
			Dimensions.Position.X = 0.0f;
			Dimensions.Position.Y = 0.0f;
			Dimensions.Position.Z = 0.0f;
			Dimensions.Size.X = 0.0f;
			Dimensions.Size.Y = 0.0f;
			Dimensions.Size.Z = 0.0f;
			EffectivePosition.Position.X = 0.0f;
			EffectivePosition.Position.Y = 0.0f;
			EffectivePosition.Position.Z = 0.0f;
			EffectivePosition.Size.X = 0.0f;
			EffectivePosition.Size.Y = 0.0f;
			EffectivePosition.Size.Z = 0.0f;

			Flag_Show_Frame = true;
			Flag_Show_Childs = true;
			Flag_Show_Master = true;

			Event_MouseWentIn = false;
			Event_MouseWentOut = false;
			Event_MouseIsActualyOver = false;
			Event_MouseClick = false;
			Event_MouseRelease = false;
			Event_GetFocus = false;
			Event_LostFocus = false;

			Flag_Enable_Proc_MouseIn = false;
			Flag_Enable_Proc_MouseOut = false;
			Flag_Enable_Proc_MouseOver = false;
			Flag_Enable_Proc_GetFocus = false;
			Flag_Enable_Proc_LostFocus = false;
			Flag_Enable_Proc_KeyDown = false;
			Flag_Enable_Proc_KeyUp = false;

			DrawColor.X = 1.0f; DrawColor.Y = 1.0f; DrawColor.Z = 1.0f; DrawColor.W = 1.0f;

		}

		internal virtual void AddedToFrameCallback( Frame Frame ) { }
		internal virtual void AddFrame( Frame Frame )
		{
			lock ( SubFrameList )
			{
				SubFrameList.AddFirst( Frame );
			}
			Frame.GuiManager = GuiManager;
			Frame.ParentFrame = this;
			Frame.AddedToFrameCallback( this );
		}

		internal virtual void RemoveFrame( Frame Frame )
		{
			lock ( SubFrameList )
			{
				SubFrameList.Remove( Frame );
			}
			Frame.RemoveAllFrames();
		}

		internal virtual void RemoveAllFrames()
		{
			lock( SubFrameList )
			{
				foreach( Frame frame in SubFrameList )
					frame.RemoveAllFrames();

				SubFrameList.Clear();
			}
		}

		internal struct quad
		{
			internal Vector3 P1, P2, P3, P4;
		};

		float[] coords = new float[3 * 6];
		float[] tex_coords = new float[2 * 6];

		internal virtual void Render( Display render, ref Box ParentPosition )
		{
			quad P;

			// Frame Position Computing
			if( Flag_Show_Master )
			{
				EffectivePosition.Position = ParentPosition.Position + Dimensions.Position;
				EffectivePosition.Position.Z -= 0.1f;
				EffectivePosition.Size = Dimensions.Size;

				if( this.Flag_Draging ) { EffectivePosition.Position.X = DragAbsolutePosition.X; EffectivePosition.Position.Y = DragAbsolutePosition.Y; }

				// Render this frame

				if( Flag_Show_Frame )
				{
					P.P1.X = EffectivePosition.Position.X;                            P.P1.Y = EffectivePosition.Position.Y;                            P.P1.Z = EffectivePosition.Position.Z + EffectivePosition.Size.Z;
					P.P2.X = EffectivePosition.Position.X + EffectivePosition.Size.X; P.P2.Y = EffectivePosition.Position.Y;                            P.P2.Z = EffectivePosition.Position.Z + EffectivePosition.Size.Z;
					P.P3.X = EffectivePosition.Position.X + EffectivePosition.Size.X; P.P3.Y = EffectivePosition.Position.Y + EffectivePosition.Size.Y; P.P3.Z = EffectivePosition.Position.Z + EffectivePosition.Size.Z;
					P.P4.X = EffectivePosition.Position.X;                            P.P4.Y = EffectivePosition.Position.Y + EffectivePosition.Size.Y; P.P4.Z = EffectivePosition.Position.Z + EffectivePosition.Size.Z;

					int TextureRef = GuiManager.TextureManager.GetTextureEntry( this.TextureNum ).OpenGl_TextureRef;
					tex_coords[0] = 0.0f; tex_coords[1] = 1.0f; coords[0] = P.P1.X; coords[1] = P.P1.Y; coords[2] = P.P1.Z;
					tex_coords[2] = 1.0f; tex_coords[3] = 1.0f; coords[3] = P.P2.X; coords[4] = P.P2.Y; coords[5] = P.P2.Z;
					tex_coords[4] = 1.0f; tex_coords[5] = 0.0f; coords[6] = P.P3.X; coords[7] = P.P3.Y; coords[8] = P.P3.Z;
					tex_coords[6] = 1.0f; tex_coords[7] = 0.0f; coords[9] = P.P3.X; coords[10] = P.P3.Y; coords[11] = P.P3.Z;
					tex_coords[8] = 0.0f; tex_coords[9] = 0.0f; coords[12] = P.P4.X; coords[13] = P.P4.Y; coords[14] = P.P4.Z;
					tex_coords[10] = 0.0f; tex_coords[11] = 1.0f; coords[15] = P.P1.X; coords[16] = P.P1.Y; coords[17] = P.P1.Z;
					//lprintf( "%3d %g,%g,%g %g,%g,%g", TextureRef, P.P1.X, P.P1.Y, P.P1.Z, P.P3.X, P.P3.Y, P.P3.Z );

					render.simple_gui_texture.Activate();
					render.simple_gui_texture.SetUniforms( TextureRef, ref DrawColor );
					render.simple_gui_texture.DrawQuadTris( coords, tex_coords );
					render.simple_texture.Activate();
					render.simple_texture.SetUniforms( TextureRef, ref DrawColor );
					render.simple_texture.DrawQuadTris( coords, tex_coords );
				}
				// Render child frames
				if( Flag_Show_Childs )
				{
					lock ( SubFrameList )
					{
						foreach( Frame frame in SubFrameList )
							frame.Render( render, ref EffectivePosition );
					}
				}

			}

		}
		internal virtual void SetTexture( TextureID TextureNum ) { this.TextureNum = TextureNum; }
		internal virtual void SetPosition( float x, float y ) { Dimensions.Position.X = x; Dimensions.Position.Y = y; }
		internal virtual void SetDragPosition( float x, float y ) { DragAbsolutePosition.X = x; DragAbsolutePosition.Y = y; }
		//internal virtual void SetZPosition( float z ) { Dimensions.Position.Z = z; }
		internal virtual void SetSize( float Width, float Height ) { Dimensions.Size.X = Width; Dimensions.Size.Y = Height; }
		internal virtual void GetSize( out float Width, out float Height ) { Width = Dimensions.Size.X; Height = Dimensions.Size.Y; }
		internal virtual void SetSize( float Depth ) { Dimensions.Position.Z = Depth; }
		internal virtual void SetColor( float r, float v, float b ) { DrawColor.X = r; DrawColor.Y = v; DrawColor.Z = b; }

		internal virtual void Show( bool ShowState ) { Flag_Show_Master = ShowState; }
		internal virtual void Show_Frame( bool ShowState = true ) { Flag_Show_Frame = ShowState; }
		internal virtual void Show_Childs( bool ShowState = true ) { Flag_Show_Childs = ShowState; }

		public bool KeyDown( Key KeySym ) { return true; }
		public bool KeyUp( Key KeySym ) { return true; }
		public bool MouseMove( float Relative_x, float Relative_y, float Absolute_x, float Absolute_y )
		{
			LinkedListNode<Frame> Item;
			Frame Frame;

			GuiManager.MouseOverStack_AddToStack( this );
			if( ( Item = SubFrameList.First ) != null )
				do
				{
					Frame = Item.Value;
					if( Absolute_x >= Frame.EffectivePosition.Position.X )
						if( Absolute_y >= Frame.EffectivePosition.Position.Y )
							if( Absolute_x <= ( Frame.EffectivePosition.Position.X + Frame.EffectivePosition.Size.X ) )
								if( Absolute_y <= ( Frame.EffectivePosition.Position.Y + Frame.EffectivePosition.Size.Y ) )
								{
									if( !Frame.Event_MouseIsActualyOver )
									{
										if( Frame.Flag_Enable_Proc_MouseIn ) Frame.MouseIn();
										Frame.Event_MouseWentIn = true;
									}
									Frame.Event_MouseIsActualyOver = true;
									if( Frame.Flag_Enable_Proc_MouseOver ) Frame.MouseOver( (short)( Frame.EffectivePosition.Position.X - Absolute_x ), (short)( Frame.EffectivePosition.Position.Y - Absolute_y ) );

									Frame.MouseMove( Relative_x, Relative_y, Absolute_x, Absolute_y );
								}
				} while( ( Item = Item.Next ) != null );
			return true;
		}

		public bool MouseButtonClick( MouseButton nButton, float Absolute_x, float Absolute_y )
		{
			LinkedListNode<Frame> Item;
			Frame Frame;


			GuiManager.FocusStack_AddToStack( this );

			// Drag and drop

			if( Flag_Cap_Dragable && nButton == MouseButton.Button1 )
			{
				//ZVector2f Dim;
				//GetSize(Dim.X,Dim.Y);
				Flag_Draging = true;
				GuiManager.DragingFrame = this;
				DragAbsolutePosition.X = EffectivePosition.Position.X;
				DragAbsolutePosition.Y = EffectivePosition.Position.Y;
			}

			// Focus

			if( this.Flag_Enable_Proc_GetFocus ) GetFocus();

			// Subframe events propagation

			if( ( Item = SubFrameList.First ) != null )
				do
				{
					Frame = Item.Value;
					if( Absolute_x >= Frame.EffectivePosition.Position.X )
						if( Absolute_y >= Frame.EffectivePosition.Position.Y )
							if( Absolute_x <= ( Frame.EffectivePosition.Position.X + Frame.EffectivePosition.Size.X ) )
								if( Absolute_y <= ( Frame.EffectivePosition.Position.Y + Frame.EffectivePosition.Size.Y ) )
								{
									Frame.Event_MouseClick = true;
									Frame.MouseButtonClick( nButton, Absolute_x, Absolute_y );
								}

				} while( ( Item = Item.Next ) != null );
			return true;
		}

		public bool MouseButtonRelease( MouseButton nButton, float Absolute_x, float Absolute_y )
		{
			LinkedListNode<Frame> Item;
			Frame Frame;

			// Drag and drop

			if( GuiManager.DragingFrame != null )
			{
				if( GuiManager.DragingFrame == this ) { return ( true ); }
				DropItem( GuiManager.DragingFrame, nButton );
			}

			//

			if( ( Item = SubFrameList.First ) != null )
				do
				{
					Frame = (Frame)Item.Value;
					if( Absolute_x >= Frame.EffectivePosition.Position.X )
						if( Absolute_y >= Frame.EffectivePosition.Position.Y )
							if( Absolute_x <= ( Frame.EffectivePosition.Position.X + Frame.EffectivePosition.Size.X ) )
								if( Absolute_y <= ( Frame.EffectivePosition.Position.Y + Frame.EffectivePosition.Size.Y ) )
								{
									Frame.Event_MouseRelease = true;
									Frame.MouseButtonRelease( nButton, Absolute_x, Absolute_y );
								}

				} while( ( Item = Item.Next ) != null );
			return true;
		}


		internal virtual void GetFocus() { }
		internal virtual void LostFocus() { }

		internal virtual void MouseOut()
		{

		}

		internal virtual void MouseIn()
		{

		}

		internal virtual void MouseOver( short Inner_x, short Inner_y )
		{

		}
		internal virtual void DropItem( Frame Item, MouseButton nButton )
		{

		}

	};
}
