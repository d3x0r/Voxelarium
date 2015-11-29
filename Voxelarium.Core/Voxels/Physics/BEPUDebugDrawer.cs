#if !USE_GLES2
using OpenTK.Graphics.OpenGL;
#else
using OpenTK.Graphics.ES20;
#endif
using BEPUutilities;
using System;
using System.Collections.Generic;

using System.Text;
using BEPUphysics;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using Voxelarium.Core.Support;

namespace Voxelarium.Core.Voxels.Physics
{
	public class BulletDebugDrawer 
	{
		public BulletDebugDrawer()
		{
		}

		public  void draw3dText( ref Vector3 location, string textString )
		{
			Console.WriteLine( "Draw GL Text : " + textString );
		}

		public  void drawContactPoint( ref Vector3 PointOnB, ref Vector3 normalOnB, double distance, int lifeTime, ref Vector3 color )
		{
			Vector3 tmpD;
			PointOnB.Add( ref normalOnB, out tmpD );
			drawLine( ref PointOnB, ref tmpD, ref color, ref color );

		}

		public static void drawLine( ref Vector3 from, ref Vector3 to, ref Vector3 fromColor, ref Vector3 toColor )
		{
#if !USE_GLES2
			GL.DrawArrays( PrimitiveType.TriangleStrip, 0, 4 );
			GL.Begin( PrimitiveType.Lines );
			GL.Color3( fromColor.ToFloat3() );
			GL.Vertex3( from.ToFloat3() );
			GL.Color3( toColor.ToFloat3() );
			GL.Vertex3( to.ToFloat3() );
			GL.End();
#endif
		}

		public  void reportErrorWarning( string warningString )
		{
			Console.WriteLine( warningString );
		}

		static void DrawBoundingBox( BoundingBox bb )
		{
			///Box box = e as Box;
			// = e.CollisionInformation.BoundingBox;
			//Vector3 corner = Matrix.TransformNormal( Vector3.One, e.WorldTransform );
			Vector3[] corners = new Vector3[8];
			//Vector3 half_size = new Vector3( box.HalfWidth, box.HalfHeight, box.HalfLength );
			corners[0] = bb.Min;
			corners[6] = bb.Max;
			corners[1] = corners[0];
			corners[1].X = corners[6].X;
			corners[2] = corners[0];
			corners[2].X = corners[6].X;
			corners[2].Y = corners[6].Y;
			corners[3] = corners[0];
			corners[3].Y = corners[6].Y;

			corners[4] = corners[0];
			corners[4].Z = corners[6].Z;

			corners[5] = corners[0];
			corners[5].X = corners[6].X;
			corners[5].Z = corners[6].Z;
			corners[7] = corners[0];
			corners[7].Y = corners[6].Y;
			corners[7].Z = corners[6].Z;
			Vector3 white = Vector3.One;
			white.Y = 0;
			white.Z = 0;
			drawLine( ref corners[0], ref corners[1], ref white, ref white );
			drawLine( ref corners[1], ref corners[2], ref white, ref white );
			drawLine( ref corners[2], ref corners[3], ref white, ref white );
			drawLine( ref corners[3], ref corners[0], ref white, ref white );
			drawLine( ref corners[0], ref corners[4], ref white, ref white );
			drawLine( ref corners[1], ref corners[5], ref white, ref white );
			drawLine( ref corners[2], ref corners[6], ref white, ref white );
			drawLine( ref corners[3], ref corners[7], ref white, ref white );
			drawLine( ref corners[4], ref corners[5], ref white, ref white );
			drawLine( ref corners[5], ref corners[6], ref white, ref white );
			drawLine( ref corners[6], ref corners[7], ref white, ref white );
			drawLine( ref corners[7], ref corners[4], ref white, ref white );
		}

		static void DrawBox( Box box )
		{
			{
				Vector3 o = box.WorldTransform.Translation;
				Vector3 xdir; Matrix3x3.Transform( ref Vector3.UnitX, ref box.orientationMatrix, out xdir );
				Vector3 ydir; Matrix3x3.Transform( ref Vector3.UnitY, ref box.orientationMatrix, out ydir );
				Vector3 zdir; Matrix3x3.Transform( ref Vector3.UnitZ, ref box.orientationMatrix, out zdir );

				Vector3 corner; Matrix3x3.Transform( ref Vector3.One, ref box.orientationMatrix, out corner );
				Vector3[] corners = new Vector3[8];
				Vector3 size = new Vector3( box.Width, box.Height, box.Length );
				corners[0] = o + corner * ( size * 0.5f);
				corners[1] = corners[0] - ( xdir * size.X );
				corners[2] = corners[0] - ( xdir * size.X ) - ( ydir * size.Y );
				corners[3] = corners[0] - ( ydir * size.Y );

				corners[4] = corners[0] - ( zdir * size.Z );
				corners[5] = corners[0] - ( zdir * size.Z ) - ( xdir * size.X );
				corners[6] = corners[0] - ( zdir * size.Z ) - ( xdir * size.X ) - ( ydir * size.Y );
				corners[7] = corners[0] - ( zdir * size.Z ) - ( ydir * size.Y );

				Vector3 white = Vector3.One;
				drawLine( ref corners[0], ref corners[1], ref white, ref white );
				drawLine( ref corners[1], ref corners[2], ref white, ref white );
				drawLine( ref corners[2], ref corners[3], ref white, ref white );
				drawLine( ref corners[3], ref corners[0], ref white, ref white );
				drawLine( ref corners[0], ref corners[4], ref white, ref white );
				drawLine( ref corners[1], ref corners[5], ref white, ref white );
				drawLine( ref corners[2], ref corners[6], ref white, ref white );
				drawLine( ref corners[3], ref corners[7], ref white, ref white );
				drawLine( ref corners[4], ref corners[5], ref white, ref white );
				drawLine( ref corners[5], ref corners[6], ref white, ref white );
				drawLine( ref corners[6], ref corners[7], ref white, ref white );
				drawLine( ref corners[7], ref corners[4], ref white, ref white );
			}
		}

		public static void DrawSpace( PhysicsEngine engine )
		{
			if( engine != null )
			{
				foreach( Entity e in engine.test_entitites )
				{
					Box box = e as Box;
					DrawBox( box );
				}
				lock( engine.active_sectors )
				{
					/*
					foreach( PhysicsEngine.Sector s in engine.active_sectors )
					{
						int n = 0;
						foreach( VoxelShape shp in s.content )
						{
							if( shp != VoxelShape.Empty )
							{
								int x = (int)(( n / s.sector.Size_y ) % s.sector.Size_x);
								int y = (int)( n % s.sector.Size_y);
								int z = (int)( ( n / ( s.sector.Size_x * s.sector.Size_y ) ) % s.sector.Size_z );
								BoundingBox bb;
								//Log.log( "found non empty at sector {0} {1} {2} offset {3}"
								//	, s.sector.Pos_x, s.sector.Pos_y, s.sector.Pos_z 
								//	, n );
								bb.Min.X = s.sector.Pos_x * 32 + x;
								bb.Min.Y = s.sector.Pos_y * 32 + y;
								bb.Min.Z = s.sector.Pos_z * 32 + z;
								bb.Max.X = bb.Min.X + 1;
								bb.Max.Y = bb.Min.Y + 1;
								bb.Max.Z = bb.Min.Z + 1;
								DrawBoundingBox( bb );
							}
							n++;
						}
						DrawBoundingBox( s.grid.BoundingBox );
					}
					*/
				}
			}
		}
	}
}
