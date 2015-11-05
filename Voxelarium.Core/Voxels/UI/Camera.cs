//#define COLUMN_MAJOR_EXPECTED

using Voxelarium.LinearMath;

namespace Voxelarium.Core.Voxels.UI
{
	internal class Camera
	{
		internal btTransform location;
		internal struct VisionColor
	    {
			internal bool Activate;
			internal byte Red, Green, Blue, Opacity;
		}
		internal VisionColor ColoredVision;

		internal Camera()
		{
			location = btTransform.Identity;
		}
		internal void MoveTo( float x, float y, float z )
		{
			location.Translate( x, y, z );
		}
		internal void RotateYaw( float yaw )
		{
			location.m_basis.Rotate( 1, yaw );
		}
		internal void RotatePitch( float pitch )
		{
			location.m_basis.Rotate( 0, pitch );
		}
		internal void RotateRoll( float roll )
		{
			location.m_basis.Rotate( 2, roll );
		}
		internal void MoveForward( float delta )
		{
			btVector3 dir;
#if COLUMN_MAJOR_EXPECTED
			location.m_basis.getColumn( 2 ).Mult( delta, out dir );
#else
			location.m_basis.getRow( 2 ).Mult( delta, out dir );
#endif
			location.Move( dir.x, dir.y, dir.z );
		}
		internal void MoveRight( float delta )
		{
			btVector3 dir;
#if COLUMN_MAJOR_EXPECTED
			location.m_basis.getColumn( 0 ).Mult( delta, out dir );
#else
			location.m_basis.getRow( 0 ).Mult( delta, out dir );
#endif
			location.Move( dir.x, dir.y, dir.z );
		}
		internal void MoveUp( float delta )
		{
			btVector3 dir;
#if COLUMN_MAJOR_EXPECTED
			location.m_basis.getColumn( 1 ).Mult( delta, out dir );
#else
			location.m_basis.getRow( 1 ).Mult( delta, out dir );
#endif
			location.Move( dir.x, dir.y, dir.z );
		}
	}
}
