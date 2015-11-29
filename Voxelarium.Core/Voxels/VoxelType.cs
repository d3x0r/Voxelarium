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
#if USE_GLES2
using Android.Graphics;
#else
#endif
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Text;
using Voxelarium.Common;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.Voxels
{
	public class VoxelType
	{
		protected VoxelGameEnvironment GameEnv;
		internal VoxelTypeManager VoxelTypeManager;

		internal protected Bitmap MainTexture;
		//internal Box2D 
		internal Box2D TextureCoords;
		internal float[] TextureUVs;
        public VoxelProperties properties;

		public delegate void OnPropertiesSet();
		public event OnPropertiesSet PropertiesSet;

		~VoxelType()
		{
			//MainTexture = null;
			properties.FabInfo = null;
			properties = null;
		}

		public VoxelType()
		{
			VoxelTypeManager = null;
			MainTexture = null;
			//OpenGl_TextureRef = 0;
			GameEnv = null;
		}

		public void SetProperties( VoxelProperties props ) { properties = props; if( PropertiesSet != null ) PropertiesSet(); }
		public void SetGameEnv( VoxelGameEnvironment GameEnv ) { this.GameEnv = GameEnv; }
		internal void SetManager( VoxelTypeManager VoxelTypeManager ) { this.VoxelTypeManager = VoxelTypeManager; }
		public virtual void SpecialRender( float x, float y, float z ) { }

		bool loaded_texture;
		internal void LoadTextureToAtlas( TextureAtlas atlas )
		{
			if( MainTexture != null && !loaded_texture )
			{
				atlas.AddTexture( MainTexture, out TextureCoords, out TextureUVs );
				loaded_texture = true;
            }
		}

		internal virtual bool LoadTexture(  )
		{
			Bitmap image = null;
			//ZBitmapImage* Image;
			int attempt;
			// Get the right folder path
			//Image = new ZBitmapImage();
			for( attempt = 0; attempt < 2; attempt++ )
			{
				string FileSpec, FileName;
				if( properties.Type < 32768 )
				{
					FileName = "voxeltexture_" + properties.Type + ( attempt == 1 ? ".bmp" : ".png" );
					FileSpec = VoxelGlobalSettings.COMPILEOPTION_DATAFILESPATH + "/VoxelTypes/" + FileName;
				}
				else
				{
					FileName = "voxeltexture_" + ( properties.Type - 32767 ) + ( attempt == 1 ? ".bmp" : ".png" );
					FileSpec = VStreamFile.Get_Directory_UserData();
					FileSpec += "/" +  VoxelGlobalSettings.COMPILEOPTION_SAVEFOLDERNAME;
					FileSpec += "/UserTextures/" + FileName;
				}

				//  if (VoxelType<32768) sprintf(Buffer, "VoxelTypes/voxeltexture_%u.bmp", VoxelType);
				//  else                 sprintf(Buffer, "UserTextures/voxeltexture_%u.bmp", VoxelType-32767);
				if( File.Exists( FileSpec ) )
					image = Display.LoadBitmap( FileSpec );
				if( image != null )
					break;
			}
			if( attempt == 2 ) { return false; }

#if COMPILEOPTION_LOWRESTEXTURING
			if( Image->Width > 128 ) Image->ReduceSize();
#endif
			MainTexture = image;
#if !USE_GLES2
			if( image.PixelFormat != System.Drawing.Imaging.PixelFormat.Format32bppArgb
				&& image.PixelFormat != System.Drawing.Imaging.PixelFormat.Format32bppRgb
				)
			{
				if( properties.Type < 32768 ) Log.log( "Warning : Image {0} is not 32 bit, this can cause crashes.", properties.Type, false );
				else Log.log( "Warning : User defined image {0} is not 32 bit, this can cause crashes.", properties.Type - 32767, false );
			}
#endif
			return ( true );
		}

		// Voxel Extensions;
		public virtual VoxelExtension CreateVoxelExtension( bool IsLoadingPhase = false ) { return null; }
		public virtual void DeleteVoxelExtension( VoxelExtension VoxelExtension, bool IsUnloadingPhase = false )
		{
			if( properties.Is_HasAllocatedMemoryExtension && VoxelExtension != null )
			{
				VoxelExtension.Dispose();
				VoxelExtension = null;
			}
		}

		//
		public virtual void UserAction_Activate( uint VoxelInfo, int x, int y, int z ) { }

		public virtual bool Interface_StoreBlock_Store( ushort VoxelType, uint Count ) { return ( false ); }
		//internal virtual uint Interface_PushBlock_Push( VoxelLocation DestLocation, ushort VoxelType, uint Count ) { return ( 0 ); }
		//internal virtual uint Interface_PushBlock_PushTest( VoxelLocation DestLocation, ushort VoxelType, uint Count ) { return ( Count ); }
		//internal virtual uint Interface_PushBlock_Pull( VoxelLocation DestLocation, out ushort VoxelType, uint Count ) { VoxelType = 0; return ( 0 ); }
		//internal virtual uint Interface_PushBlock_PullTest( VoxelLocation DestLocation, out ushort VoxelType, uint Count ) { VoxelType = 0; return ( 0 ); }

		// Squirrel interface
		//public virtual bool Interface_GetInfo( VoxelLocation VLoc, uint InfoNum, ZVar Out ) { return ( false ); }
		//public virtual bool Interface_GetInfoDoc( uint InfoNum, uint DocType, ZVar Out ) { return ( false ); }
		//public virtual bool Interface_SetInfo( VoxelLocation VLoc, uint InfoNum, ZVar In ) { return ( false ); }
		//internal virtual void GetBlockInformations( VoxelLocation DestLocation, string Infos ) { return; }

		// When an active voxel should be processed. Note some voxels use "direct" faster way.
		//public virtual void ActiveProcess( ActiveVoxelInterface AvData ) { };

		public virtual bool React( ref VoxelRef self, double tick ) { return false; }

	}
}
