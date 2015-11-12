using Bullet.LinearMath;
using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;
using System.Text;

namespace Bullet.Debug.OpenGL2
{
	public class BulletDebugDrawer : btIDebugDraw
	{
		public BulletDebugDrawer()
		{
		}

		public override void draw3dText( ref btVector3 location, string textString )
		{
			Console.WriteLine( "Draw GL Text : " + textString );
		}

		public override void drawContactPoint( ref btVector3 PointOnB, ref btVector3 normalOnB, double distance, int lifeTime, ref btVector3 color )
		{
			btVector3 tmpD;
			PointOnB.Add( ref normalOnB, out tmpD );
			drawLine( ref normalOnB, ref tmpD, ref color, ref color );

		}

		public override void drawLine( ref btVector3 from, ref btVector3 to, ref btVector3 fromColor, ref btVector3 toColor )
		{

			GL.Begin( PrimitiveType.Lines );
			GL.Color4( fromColor.ToFloat4() );
			GL.Vertex3( from.ToFloat4() );
			GL.Color4( toColor.ToFloat4() );
			GL.Vertex3( to.ToFloat4() );
			GL.End();
		}

		public override void reportErrorWarning( string warningString )
		{
			Console.WriteLine( warningString );
		}

		DebugDrawModes mode = DebugDrawModes.DBG_DrawWireframe 
			| DebugDrawModes.DBG_DrawNormals
			| DebugDrawModes.DBG_DrawConstraints
			| DebugDrawModes.DBG_DrawContactPoints
			| DebugDrawModes.DBG_DrawAabb;
		public override void setDebugMode( DebugDrawModes debugMode )
		{
			mode = debugMode;
		}

		public override DebugDrawModes getDebugMode()
		{
			return mode;
			
		}
	}
}
