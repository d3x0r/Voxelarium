using Bullet.LinearMath;
using System;
using System.Collections.Generic;
using System.Text;
using Voxelarium.Core.Game;
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

		class Render_Interface_displaydata
		{
			internal int DisplayList_Regular;
			internal int DisplayList_Transparent;

			internal Render_Interface_displaydata()
			{
				for( int i = 0; i < 6; i++ )
				{
					DisplayList_Regular = 0;
					DisplayList_Transparent = 0;
				}

			}
			~Render_Interface_displaydata()
			{
				for( int i = 0; i < 6; i++ )
				{
					//if( DisplayList_Regular ) glDeleteLists( DisplayList_Regular, 1 );
					DisplayList_Regular = 0;
					//if( DisplayList_Transparent ) glDeleteLists( DisplayList_Transparent, 1 );
					DisplayList_Transparent = 0;
				}
			}
		}

		protected VoxelWorld World;
		protected VoxelTypeManager VoxelTypeManager;
		protected TextureManager TextureManager;

		//PTRSZVAL current_gl_camera;
		internal protected Camera Camera;
		//float* Aspect_Ratio;
		protected Actor Actor;  // where the camera came from really...
								//protected RayCast_out PointedVoxel;
		protected Radius_Zoning RadiusZones;

		int hRenderRadius;
		int vRenderRadius;

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
			Frustum_CullingLimit = 50.0;
		}

#if asdf
		void SetWorld( VoxelWorld World );
		void SetCamera( ZCamera* Camera );
		void SetActor( ZActor* Actor );

		void SetVoxelTypeManager( ZVoxelTypeManager* Manager );
		void SetTextureManager( ZTextureManager* Manager ) { this->TextureManager = Manager; }
		void SetPointedVoxel( ZRayCast_out* Pvoxel ) { this->PointedVoxel = Pvoxel; }
		void SetViewportResolution( ZVector2f &Resolution) { ViewportResolution = Resolution; }
		void SetVerticalFOV( double VFov ) { VerticalFOV = VFov; }
		void SetPixelAspectRatio( double AspectRatio = 1.0 ) { PixelAspectRatio = AspectRatio; }
		void SetSectorCullingOptimisationFactor( double CullingOptimisationFactor = 1.0 ) { Optimisation_FCullingFactor = CullingOptimisationFactor; }



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
		internal abstract VoxelCuller GetCuller();

		void SetRenderSectorRadius( int Horizontal, int Vertical )
		{
			hRenderRadius = Horizontal;
			vRenderRadius = Vertical;
			RadiusZones.SetSize( hRenderRadius * 2 + 1, vRenderRadius * 2 + 1, hRenderRadius * 2 + 1 );
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
		internal bool Is_PointVisible( ref btMatrix3x3 TransformParam, ref btVector3 Point )
		{
			btVector3 Cv;
			bool Visible;
			TransformParam.ApplyRotation( ref Point, out Cv );

			// Projection

			Cv.x = Cv.x / Cv.z * FocusDistance;  // Number replaced by FocusDistance was 50.0
			Cv.y = Cv.y / Cv.z * FocusDistance;

			// Visibility test

			Visible = (
						 ( Cv.z > 0.0 )
					  && ( Cv.x < Frustum_CullingLimit && Cv.x > -Frustum_CullingLimit ) // Number replaced by Frustum_CullingLimit was 50.0
					  && ( Cv.y < Frustum_CullingLimit && Cv.y > -Frustum_CullingLimit ) //
					);

			return ( Visible );
		}

		internal void Render_EmptySector( int x, int y, int z, float r, float g, float b )
		{
#if asdfasdf
			btVector3 P1, P2, P3, P4, P5, P6, P7, P8;
			btVector3 c;
			c.r = r;
			c.g = g;
			c.b = b;
			c.a = 0.5f;
			P1.x = x * ZVOXELBLOCSIZE_X * GlobalSettings.VoxelBlockSize + 0.0f; P1.y = y * ZVOXELBLOCSIZE_Y * GlobalSettings.VoxelBlockSize + 0.0f; P1.z = z * ZVOXELBLOCSIZE_Z * GlobalSettings.VoxelBlockSize + 0.0f;
			P2.x = x * ZVOXELBLOCSIZE_X * GlobalSettings.VoxelBlockSize + 0.0f; P2.y = y * ZVOXELBLOCSIZE_Y * GlobalSettings.VoxelBlockSize + 0.0f; P2.z = z * ZVOXELBLOCSIZE_Z * GlobalSettings.VoxelBlockSize + ZVOXELBLOCSIZE_Z * GlobalSettings.VoxelBlockSize;
			P3.x = x * ZVOXELBLOCSIZE_X * GlobalSettings.VoxelBlockSize + ZVOXELBLOCSIZE_X * GlobalSettings.VoxelBlockSize; P3.y = y * ZVOXELBLOCSIZE_Y * GlobalSettings.VoxelBlockSize + 0.0f; P3.z = z * ZVOXELBLOCSIZE_Z * GlobalSettings.VoxelBlockSize + ZVOXELBLOCSIZE_Z * GlobalSettings.VoxelBlockSize;
			P4.x = x * ZVOXELBLOCSIZE_X * GlobalSettings.VoxelBlockSize + ZVOXELBLOCSIZE_X * GlobalSettings.VoxelBlockSize; P4.y = y * ZVOXELBLOCSIZE_Y * GlobalSettings.VoxelBlockSize + 0.0f; P4.z = z * ZVOXELBLOCSIZE_Z * GlobalSettings.VoxelBlockSize + 0.0f;
			P5.x = x * ZVOXELBLOCSIZE_X * GlobalSettings.VoxelBlockSize + 0.0f; P5.y = y * ZVOXELBLOCSIZE_Y * GlobalSettings.VoxelBlockSize + ZVOXELBLOCSIZE_Y * GlobalSettings.VoxelBlockSize; P5.z = z * ZVOXELBLOCSIZE_Z * GlobalSettings.VoxelBlockSize + 0.0f;
			P6.x = x * ZVOXELBLOCSIZE_X * GlobalSettings.VoxelBlockSize + 0.0f; P6.y = y * ZVOXELBLOCSIZE_Y * GlobalSettings.VoxelBlockSize + ZVOXELBLOCSIZE_Y * GlobalSettings.VoxelBlockSize; P6.z = z * ZVOXELBLOCSIZE_Z * GlobalSettings.VoxelBlockSize + ZVOXELBLOCSIZE_Z * GlobalSettings.VoxelBlockSize;
			P7.x = x * ZVOXELBLOCSIZE_X * GlobalSettings.VoxelBlockSize + ZVOXELBLOCSIZE_X * GlobalSettings.VoxelBlockSize; P7.y = y * ZVOXELBLOCSIZE_Y * GlobalSettings.VoxelBlockSize + ZVOXELBLOCSIZE_Y * GlobalSettings.VoxelBlockSize; P7.z = z * ZVOXELBLOCSIZE_Z * GlobalSettings.VoxelBlockSize + ZVOXELBLOCSIZE_Z * GlobalSettings.VoxelBlockSize;
			P8.x = x * ZVOXELBLOCSIZE_X * GlobalSettings.VoxelBlockSize + ZVOXELBLOCSIZE_X * GlobalSettings.VoxelBlockSize; P8.y = y * ZVOXELBLOCSIZE_Y * GlobalSettings.VoxelBlockSize + ZVOXELBLOCSIZE_Y * GlobalSettings.VoxelBlockSize; P8.z = z * ZVOXELBLOCSIZE_Z * GlobalSettings.VoxelBlockSize + 0.0f;

			simple_shader->DrawBox( &P1, &P2, &P3, &P4
				, &P5, &P6, &P7, &P8
				, &c );
#endif
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


