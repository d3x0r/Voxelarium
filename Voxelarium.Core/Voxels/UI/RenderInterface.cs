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
using Voxelarium.LinearMath;
using OpenTK;
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.Game;
using Voxelarium.Core.Support;
using Voxelarium.Core.UI;
using Voxelarium.Core.Voxels;

namespace Voxelarium.Core.Voxels.UI
{
	internal abstract class RenderInterface
	{
		int[] TextureName = new int[1024];
		internal static VoxelSector.FACEDRAW_Operations[][] IntFaceStateTable =
				{
				  new VoxelSector.FACEDRAW_Operations[]{ // State 0: Clear = no FullOpaque = no TranspRend = no
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 0 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 0
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 1 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 1
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 0 TranspRend = 1

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 1
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  },
				  new VoxelSector.FACEDRAW_Operations[]{ // State 1: Clear = yes FullOpaque = no TranspRend = no
					0   , // Clear = 0 FullOpaque = 0 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 0

					0   , // Clear = 0 FullOpaque = 1 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 0

					0   , // Clear = 0 FullOpaque = 0 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 1

					0   , // Clear = 0 FullOpaque = 1 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  },
				  new VoxelSector.FACEDRAW_Operations[]{ // State 2: Clear = no FullOpaque = yes TranspRend = no
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 0 TranspRend = 0

					0   , // Clear = 0 FullOpaque = 1 TranspRend = 0
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 1 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 1
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 0 TranspRend = 1

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 1
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  },
				  new VoxelSector.FACEDRAW_Operations[]{ // State 3 : Clear = yes FullOpaque = yes TranspRend = no
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 0 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 0
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 1 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 1
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 0 TranspRend = 1

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 1
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  },
				  new VoxelSector.FACEDRAW_Operations[]{ // State 4 : Clear = no FullOpaque = no TranspRend = yes
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 0 TranspRend = 0

					0   , // Clear = 0 FullOpaque = 1 TranspRend = 0
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 1 TranspRend = 0

					0   , // Clear = 0 FullOpaque = 0 TranspRend = 1
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 0 TranspRend = 1

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 1
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  },
				  new VoxelSector.FACEDRAW_Operations[]{ // State 5: Clear = yes FullOpaque = no TranspRend = yes
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 0 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 0
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 1 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 1
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 0 TranspRend = 1

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 1
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  },
				  new VoxelSector.FACEDRAW_Operations[]{ // State 6: Clear = no FullOpaque = yes TranspRend = yes
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 0 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 0
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 1 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 1
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 0 TranspRend = 1

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 1
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  },
				  new VoxelSector.FACEDRAW_Operations[]{ // State 7: Clear = yes FullOpaque = yes TranspRend = yes
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 0 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 0
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 1 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 1
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 0 TranspRend = 1

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 1
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  }
				};

		internal static VoxelSector.FACEDRAW_Operations[][] ExtFaceStateTable =
				{
				  new VoxelSector.FACEDRAW_Operations[8]{ // State 0: Clear = no FullOpaque = no TranspRend = no
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 1

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  },
				  new VoxelSector.FACEDRAW_Operations[8]{ // State 1: Clear = yes FullOpaque = no TranspRend = no
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend =  int Sector_x,Sector_y,Sector_z; 1
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 1

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  },
				  new VoxelSector.FACEDRAW_Operations[8]{ // State 2: Clear = no FullOpaque = yes TranspRend = no
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 0

					0   , // Clear = 0 FullOpaque = 1 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 0

					0   , // Clear = 0 FullOpaque = 0 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 1

					0   , // Clear = 0 FullOpaque = 1 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  },
				  new VoxelSector.FACEDRAW_Operations[8]{ // State 3 : Clear = yes FullOpaque = yes TranspRend = no
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 1

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  },
				  new VoxelSector.FACEDRAW_Operations[8] { // State 4 : Clear = no FullOpaque = no TranspRend = yes
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 0

					0   , // Clear = 0 FullOpaque = 0 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 1

					0   , // Clear = 0 FullOpaque = 1 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  },
				 new VoxelSector.FACEDRAW_Operations[8] { // State 5: Clear = yes FullOpaque = no TranspRend = yes
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 1

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  },
				  new VoxelSector.FACEDRAW_Operations[8]{ // State 6: Clear = no FullOpaque = yes TranspRend = yes
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 0

					0   , // Clear = 0 FullOpaque = 0 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 1

					0   , // Clear = 0 FullOpaque = 1 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  },
				  new VoxelSector.FACEDRAW_Operations[8]{ // State 7: Clear = yes FullOpaque = yes TranspRend = yes
					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 0
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 0

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 0 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 0 TranspRend = 1

					VoxelSector.FACEDRAW_Operations.ALL_BITS , // Clear = 0 FullOpaque = 1 TranspRend = 1
					0   , // Clear = 1 FullOpaque = 1 TranspRend = 1
				  }
				};


		protected VoxelTypeManager VoxelTypeManager;
		protected TextureManager TextureManager;

		//PTRSZVAL current_gl_camera;
		internal protected Camera Camera;
		//float* Aspect_Ratio;
		protected Actor Actor;  // where the camera came from really...
								//protected RayCast_out PointedVoxel;
		protected Radius_Zoning RadiusZones;

		uint hRenderRadius;
		uint vRenderRadius;

		protected int Stat_RenderDrawFaces;
		protected int Stat_FaceTop;
		protected int Stat_FaceBottom;
		protected int Stat_FaceLeft;
		protected int Stat_FaceRight;
		protected int Stat_FaceFront;
		protected int Stat_FaceBack;

		internal int Stat_RefreshWaitingSectorCount;
		internal int[] RefreshWaiters = new int[64];
		internal int[] RefreshToDo = new int[64];

		// Display Dimension and aspect ratio

		//Vector2 ViewportResolution;
		double VerticalFOV;
		float FocusDistance;
		double PixelAspectRatio;
		double Optimisation_FCullingFactor;

		// Computed by render()
		double Frustum_V;
		double Frustum_H;
		double Frustum_CullingLimit;

		internal VoxelGameEnvironment GameEnv;

		SectorSphere SectorSphere;

		int BvProp_CrossHairType;
		bool BvProp_DisplayCrossHair;
		bool BvProp_DisplayVoxelSelector;

		//Render_Sorter RenderSorter;

		//Render_Shader_Simple simple_shader;
		//Render_Shader_Gui_Texture gui_shader;
		//Render_Shader_Simple_Texture simple_texture_shader;

		internal RenderInterface()
		{
			RadiusZones = new Radius_Zoning();

			hRenderRadius = 1;  // 8
			vRenderRadius = 1;  // 3
			BvProp_CrossHairType = 1;
			BvProp_DisplayCrossHair = true;
			BvProp_DisplayVoxelSelector = true;
			//ViewportResolution.x = 1920;
			//ViewportResolution.y = 1200;
			//VerticalFOV = 63.597825649;
			FocusDistance = 50.0f;
			PixelAspectRatio = 1.0;
			Optimisation_FCullingFactor = 1.0;

			Frustum_V = 0.0;
			Frustum_H = 0.0;
			//Aspect_Ratio = 0.0;
			Frustum_CullingLimit = 90.0;
		}

		internal void SetPixelAspectRatio( float AspectRatio = 1.0f ) { PixelAspectRatio = AspectRatio; }
		internal void SetSectorCullingOptimisationFactor( float CullingOptimisationFactor = 1.0f )
		{ Optimisation_FCullingFactor = CullingOptimisationFactor; }
		void SetCamera( Camera Camera )
		{
			this.Camera = Camera;
		}
#if asdf
		void SetWorld( VoxelWorld World )
		{

		}
		void SetActor( ZActor* Actor );

		void SetVoxelTypeManager( ZVoxelTypeManager* Manager );
		void SetTextureManager( ZTextureManager* Manager ) { this->TextureManager = Manager; }
		void SetPointedVoxel( ZRayCast_out* Pvoxel ) { this->PointedVoxel = Pvoxel; }
		void SetViewportResolution( ZVector2f &Resolution) { ViewportResolution = Resolution; }
		void SetVerticalFOV( double VFov ) { VerticalFOV = VFov; }



		void Init();
		void Cleanup() { }

		void Render_DebugLine( ZVector3f & Start, ZVector3f & End);

		void Render_VoxelSelector( ZVoxelCoords* SelectedVoxel, float r, float g, float b );
		void EmitFaces( ZVoxelType** VoxelTypeTable, UShort &VoxelType, UShort &prevVoxelType, int info
								  , int x, int y, int z
								  , int Sector_Display_x, int Sector_Display_y, int Sector_Display_z );

		virtual void MakeSectorRenderingData( ZVoxelSector* Sector ) = 0;
    virtual void MakeSectorRenderingData_Sorted( ZVoxelSector* Sector ) = 0;
    virtual void FreeDisplayData( ZVoxelSector* Sector ) = 0;
    virtual void Render( bool use_external_matrix ) = 0;




		void SetGameEnv( ZGame* GameEnv ) { this->GameEnv = GameEnv; }


		// void RenderSector2(ZVoxelSector * Sector);


	void ComputeAndSetAspectRatio( double VerticalFOV, double PixelAspectRatio, ZVector2L & ViewportResolution);
	void DrawReticule( void );
	void DrawColorOverlay( void );
#endif
		internal abstract VoxelCuller GetCuller( );

		internal void SetRenderSectorRadius( uint Horizontal, uint Vertical )
		{
			hRenderRadius = Horizontal;
			vRenderRadius = Vertical;
			RadiusZones.SetSize( (int)hRenderRadius * 2 + 1, (int)vRenderRadius * 2 + 1, (int)hRenderRadius * 2 + 1 );
			// RadiusZones.DrawZones( 5.0, 3.5, 3.0, 2.0 );
			RadiusZones.DrawZones( 5.0, 1 );
			RadiusZones.DrawZones( 3.5, 2 );
			RadiusZones.DrawZones( 3.0, 3 );
			RadiusZones.DrawZones( 2.0, 4 );
			//RadiusZones.DebugOut();
			SectorSphere.Init( Horizontal, Vertical );
		}

		
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		// point fed to this is relative to the camera origin already.
		internal bool Is_PointVisible( ref btTransform TransformParam, ref btVector3 Point )
		{
			btVector3 Cv;
			btVector3 Cv2;
			bool Visible;
			Point.Sub( ref TransformParam.m_origin, out Cv2);
			TransformParam.m_basis.Apply( ref Cv2, out Cv );
			//Cv.Sub(

			// Projection
			if( Cv.z > 0 )
			{
				Cv.x = Cv.x / Cv.z * FocusDistance;  // Number replaced by FocusDistance was 50.0
				Cv.y = Cv.y / Cv.z * FocusDistance;

				// Visibility test

				Visible = ( ( Cv.x < Frustum_CullingLimit && Cv.x > -Frustum_CullingLimit ) // Number replaced by Frustum_CullingLimit was 50.0
						  && ( Cv.y < Frustum_CullingLimit && Cv.y > -Frustum_CullingLimit ) //
						);
				//Log.log( "visible: {0} {1} {2} {3}", Point.x, Point.y, Point.z, Visible );
				return ( Visible );
			}
			return false;
		}

		internal abstract void Render( Display display, VoxelWorld world );

		float[] sector_verts = new float[2 * 12 * 3];

		internal void Render_EmptySector( Display display, VoxelWorld world, int x, int y, int z, float r, float g, float b )
		{
			Vector4 c;
			int p = 0;
			int sx = (int)VoxelSector.ZVOXELBLOCSIZE_X * world.VoxelBlockSize;
			int sy = (int)VoxelSector.ZVOXELBLOCSIZE_Y * world.VoxelBlockSize;
			int sz = (int)VoxelSector.ZVOXELBLOCSIZE_Z * world.VoxelBlockSize;

			c.X = r;
			c.Y = g;
			c.Z = b;
			c.W = 0.5f;
			unsafe
			{
				fixed ( float* _v = sector_verts )
				{
					float* v = _v;
					*(v++) = x * sx;
					*(v++) = y * sy;
					*(v++) = z * sz;
					*(v++) = x * sx;
					*(v++) = y * sy;
					*(v++) = z * sz + sz;

					*(v++) = x * sx;
					*(v++) = y * sy;
					*(v++) = z * sz + sz;
					*(v++) = x * sx + sx;
					*(v++) = y * sy;
					*(v++) = z * sz + sz;

					*(v++) = x * sx + sx;
					*(v++) = y * sy;
					*(v++) = z * sz + sz;
					*(v++) = x * sx + sx;
					*(v++) = y * sy;
					*(v++) = z * sz;


					*(v++) = x * sx + sx;
					*(v++) = y * sy;
					*(v++) = z * sz;
					*(v++) = x * sx;
					*(v++) = y * sy;
					*(v++) = z * sz;

					*(v++) = x * sx;
					*(v++) = y * sy + sy;
					*(v++) = z * sz;
					*(v++) = x * sx;
					*(v++) = y * sy + sy;
					*(v++) = z * sz + sz;

					*(v++) = x * sx;
					*(v++) = y * sy + sy;
					*(v++) = z * sz + sz;
					*(v++) = x * sx + sx;
					*(v++) = y * sy + sy;
					*(v++) = z * sz + sz;

					*(v++) = x * sx + sx;
					*(v++) = y * sy + sy;
					*(v++) = z * sz + sz;
					*(v++) = x * sx + sx;
					*(v++) = y * sy + sy;
					*(v++) = z * sz;

					*(v++) = x * sx + sx;
					*(v++) = y * sy + sy;
					*(v++) = z * sz;
					*(v++) = x * sx;
					*(v++) = y * sy + sy;
					*(v++) = z * sz;

					*(v++) = x * sx;
					*(v++) = y * sy;
					*(v++) = z * sz;
					*(v++) = x * sx;
					*(v++) = y * sy + sy;
					*(v++) = z * sz;

					*(v++) = x * sx;
					*(v++) = y * sy;
					*(v++) = z * sz + sz;
					*(v++) = x * sx;
					*(v++) = y * sy + sy;
					*(v++) = z * sz + sz;

					*(v++) = x * sx + sx;
					*(v++) = y * sy;
					*(v++) = z * sz + sz;
					*(v++) = x * sx + sx;
					*(v++) = y * sy + sy;
					*(v++) = z * sz + sz;

					*(v++) = x * sx + sx;
					*(v++) = y * sy;
					*(v++) = z * sz;
					*(v++) = x * sx + sx;
					*(v++) = y * sy + sy;
					*(v++) = z * sz;
					if( display.simple.Activate() )
						display.simple.DrawBox( sector_verts, ref c );
				}
			}
#if false
	CheckErr();

              glDisable(GL_TEXTURE_2D);
              glColor3f(r,g,b);
              glEnable(GL_LINE_SMOOTH);
	CheckErr();

              glEnable (GL_LINE_SMOOTH);
              glEnable (GL_BLEND);
	CheckErr();
              glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
              glHint (GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
	CheckErr();
              glLineWidth (3.5);
    //glLineWidth(0.001f);
    //glPointSize(0.001f);
			  	CheckErr();
                glBegin(GL_LINES);
                  glVertex3f(P1.x,P1.y,P1.z);glVertex3f(P2.x,P2.y,P2.z);
                  glVertex3f(P2.x,P2.y,P2.z);glVertex3f(P3.x,P3.y,P3.z);
                  glVertex3f(P3.x,P3.y,P3.z);glVertex3f(P4.x,P4.y,P4.z);
                  glVertex3f(P4.x,P4.y,P4.z);glVertex3f(P1.x,P1.y,P1.z);

                  glVertex3f(P5.x,P5.y,P5.z);glVertex3f(P6.x,P6.y,P6.z);
                  glVertex3f(P6.x,P6.y,P6.z);glVertex3f(P7.x,P7.y,P7.z);
                  glVertex3f(P7.x,P7.y,P7.z);glVertex3f(P8.x,P8.y,P8.z);
                  glVertex3f(P8.x,P8.y,P8.z);glVertex3f(P5.x,P5.y,P5.z);

                  glVertex3f(P1.x,P1.y,P1.z);glVertex3f(P5.x,P5.y,P5.z);
                  glVertex3f(P2.x,P2.y,P2.z);glVertex3f(P6.x,P6.y,P6.z);
                  glVertex3f(P3.x,P3.y,P3.z);glVertex3f(P7.x,P7.y,P7.z);
                  glVertex3f(P4.x,P4.y,P4.z);glVertex3f(P8.x,P8.y,P8.z);
                glEnd();
              glColor3f(1.0,1.0,1.0);
              glEnable(GL_TEXTURE_2D);
#endif

		}

	}
}


