using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	public struct VoxelRef
	{
		public VoxelSector Sector;
		public int Offset;
		public int x, y, z;
		public int wx, wy, wz;
		public ushort Type;
		public VoxelWorld World;
		public VoxelTypeManager VoxelTypeManager;
		public VoxelExtension VoxelExtension;

		/*
		VoxelRef( ZVoxelWorld *world, ZVoxelTypeManager *vtm, long x = 0, long y = 0, long z = 0, ZVoxelSector *Sector=NULL, UShort VoxelType = 0, int offset = 0 )
		{
			this.x = x;
			this.y = y;
			this.z = z;
			this.Sector = Sector;
			this.Offset = offset;
			this.World = world;
			this.VoxelType = VoxelType;
			VoxelTypeManager = vtm;
		}
		*/
		public delegate int ForEachCallback( ref VoxelRef voxelRef );
		static int ForEachVoxel( VoxelWorld World, ref VoxelRef v1, ref VoxelRef v2, ForEachCallback f, bool not_zero )
		{
			if( v1.Sector == null || v2.Sector == null )
				return not_zero ? 1 : 0;
			int v1x = v1.x + ( v1.Sector.Pos_x << VoxelSector.ZVOXELBLOCSHIFT_X );
			int v1y = v1.y + ( v1.Sector.Pos_y << VoxelSector.ZVOXELBLOCSHIFT_Y );
			int v1z = v1.z + ( v1.Sector.Pos_z << VoxelSector.ZVOXELBLOCSHIFT_Z );
			int v2x = v2.x + ( v2.Sector.Pos_x << VoxelSector.ZVOXELBLOCSHIFT_X );
			int v2y = v2.y + ( v2.Sector.Pos_y << VoxelSector.ZVOXELBLOCSHIFT_Y );
			int v2z = v2.z + ( v2.Sector.Pos_z << VoxelSector.ZVOXELBLOCSHIFT_Z );
			int del_x = v2x - v1x;
			int del_y = v2y - v1y;
			int del_z = v2z - v1z;
			int abs_x = del_x < 0 ? -del_x : del_x;
			int abs_y = del_y < 0 ? -del_y : del_y;
			int abs_z = del_z < 0 ? -del_z : del_z;
			// cannot use iterate if either end is undefined.
			if( del_x != 0 )
			{
				if( del_y != 0 )
				{
					if( del_z != 0 )
					{
						if( abs_x > abs_y || ( abs_z > abs_y ) )
						{
							if( abs_z > abs_x )
							{
								// z is longest path
								int erry = -abs_z / 2;
								int errx = -abs_z / 2;
								int incy = del_y < 0 ? -1 : 1;
								int incx = del_x < 0 ? -1 : 1;
								int incz = del_z < 0 ? -1 : 1;
								{
									int x = v1x;
									int y = v1y;
									for( int z = v1z + incz; z != v2z; z += incz )
									{
										errx += abs_x;
										if( errx > 0 )
										{
											errx -= abs_z;
											x += incx;
										}
										erry += abs_y;
										if( erry > 0 )
										{
											erry -= abs_z;
											y += incy;
										}
										{
											int val;
											VoxelRef v;
											if( World.GetVoxelRef( out v, x, y, z ) )
											{
												val = f( ref v );
												if( ( !not_zero && val != 0 ) || ( not_zero && val == 0 ) )
													return val;
											}
										}
									}
								}
							}
							else
							{
								// x is longest.
								int erry = -abs_x / 2;
								int errz = -abs_x / 2;
								int incy = del_y < 0 ? -1 : 1;
								int incx = del_x < 0 ? -1 : 1;
								int incz = del_z < 0 ? -1 : 1;
								{
									int y = v1y;
									int z = v1z;
									for( int x = v1x + incx; x != v2x; x += incx )
									{
										errz += abs_z;
										if( errz > 0 )
										{
											errz -= abs_x;
											z += incx;
										}
										erry += abs_y;
										if( erry > 0 )
										{
											erry -= abs_x;
											y += incy;
										}
										{
											int val;
											VoxelRef v;
											if( World.GetVoxelRef( out v, x, y, z ) )
											{
												val = f( ref v );
												if( ( !not_zero && val != 0 ) || ( not_zero && val == 0 ) )
													return val;
											}
										}
									}
								}
							}
						}
						else
						{
							// y is longest.
							int errx = -abs_y / 2;
							int errz = -abs_y / 2;
							int incy = del_y < 0 ? -1 : 1;
							int incx = del_x < 0 ? -1 : 1;
							int incz = del_z < 0 ? -1 : 1;
							{
								int x = v1x;
								int z = v1x;
								for( int y = v1y + incy; y != v2y; y += incy )
								{
									errx += abs_x;
									if( errx > 0 )
									{
										errx -= abs_y;
										x += incx;
									}
									errz += abs_z;
									if( errz > 0 )
									{
										errz -= abs_y;
										z += incz;
									}
									{
										int val;
										VoxelRef v;
										if( World.GetVoxelRef( out v, x, y, z ) )
										{
											val = f( ref v );
											if( ( !not_zero && val != 0 ) || ( not_zero && val == 0 ) )
												return val;
										}
									}
								}
							}
						}
					}
					else
					{
						// z is constant
						if( abs_x > abs_y )
						{
							// x is longest
							int erry = -abs_x / 2;
							int incy = del_y < 0 ? -1 : 1;
							int incx = del_x < 0 ? -1 : 1;
							{
								int y = v1y;
								int z = v1z;
								for( int x = v1x + incx; x != v2x; x += incx )
								{
									erry += abs_y;
									if( erry > 0 )
									{
										erry -= abs_x;
										y += incy;
									}
									{
										int val;
										VoxelRef v;
										if( World.GetVoxelRef( out v, x, y, z ) )
										{
											val = f( ref v );
											if( ( !not_zero && val != 0 ) || ( not_zero && val == 0 ) )
												return val;
										}
									}
								}
							}
						}
						else
						{
							// y is longest.
							int errx = -abs_y / 2;
							int incy = del_y < 0 ? -1 : 1;
							int incx = del_x < 0 ? -1 : 1;
							// z is longest path
							{
								int x = v1x;
								int z = v1x;
								for( int y = v1y + incy; y != v2y; y += incy )
								{
									errx += abs_x;
									if( errx > 0 )
									{
										errx -= abs_y;
										x += incx;
									}
									{
										int val;
										VoxelRef v;
										if( World.GetVoxelRef( out v, x, y, z ) )
										{
											val = f( ref v );
											if( ( !not_zero && val != 0 ) || ( not_zero && val == 0 ) )
												return val;
										}
									}
								}
							}
						}
					}
				}
				else
				{
					if( del_z != 0 )
					{
						if( abs_x > abs_z )
						{
							// x is longest.
							int errz = -abs_x / 2;
							int incx = del_x < 0 ? -1 : 1;
							int incz = del_z < 0 ? -1 : 1;
							{
								int y = v1y;
								int z = v1z;
								for( int x = v1x + incx; x != v2x; x += incx )
								{
									errz += abs_z;
									if( errz > 0 )
									{
										errz -= abs_x;
										z += incx;
									}
									{
										int val;
										VoxelRef v;
										if( World.GetVoxelRef( out v, x, y, z ) )
										{
											val = f( ref v );
											if( ( !not_zero && val != 0 ) || ( not_zero && val == 0 ) )
												return val;
										}
									}
								}
							}
						}
						else
						{
							// z is longest path
							int errx = -abs_z / 2;
							int incx = del_x < 0 ? -1 : 1;
							int incz = del_z < 0 ? -1 : 1;
							{
								int x = v1x;
								int y = v1y;
								for( int z = v1z + incz; z != v2z; z += incz )
								{
									errx += abs_x;
									if( errx > 0 )
									{
										errx -= abs_z;
										x += incx;
									}
									{
										int val;
										VoxelRef v;
										if( World.GetVoxelRef( out v, x, y, z ) )
										{
											val = f( ref v );
											if( ( !not_zero && val != 0 ) || ( not_zero && val == 0 ) )
												return val;
										}
									}
								}
							}
						}
					}
					else
					{
						// x is only changing.
						int incx = del_x < 0 ? -1 : 1;
						for( int x = v1x + incx; x != v2x; x += incx )
						{
							int val;
							VoxelRef v;
							if( World.GetVoxelRef( out v, x, v1y, v1z ) )
							{
								val = f( ref v );
								if( ( !not_zero && val != 0 ) || ( not_zero && val == 0 ) )
									return val;
							}
						}
					}
				}
			}
			else
			{
				if( del_y != 0 )
				{
					if( del_z != 0 )
					{
						if( abs_y > abs_z )
						{
							// y is longest.
							int errz = -abs_y / 2;
							int incy = del_y < 0 ? -1 : 1;
							int incz = del_z < 0 ? -1 : 1;
							{
								int x = v1x;
								int z = v1x;
								for( int y = v1y + incy; y != v2y; y += incy )
								{
									errz += abs_z;
									if( errz > 0 )
									{
										errz -= abs_y;
										z += incz;
									}
									{
										int val;
										VoxelRef v;
										if( World.GetVoxelRef( out v, x, y, z ) )
										{
											val = f( ref v );
											if( ( !not_zero && val != 0 ) || ( not_zero && val == 0 ) )
												return val;
										}
									}
								}
							}
						}
						else
						{
							// z is longest path
							int erry = -abs_z / 2;
							int incy = del_y < 0 ? -1 : 1;
							int incz = del_z < 0 ? -1 : 1;
							{
								int x = v1x;
								int y = v1y;
								for( int z = v1z + incz; z != v2z; z += incz )
								{
									erry += abs_y;
									if( erry > 0 )
									{
										erry -= abs_z;
										y += incy;
									}
									{
										int val;
										VoxelRef v;
										if( World.GetVoxelRef( out v, x, y, z ) )
										{
											val = f( ref v );
											if( ( !not_zero && val != 0 ) || ( not_zero && val == 0 ) )
												return val;
										}
									}
								}
							}
						}
					}
					else
					{
						// no del_x, no del_z
						// y is only changing.
						int incy = del_y < 0 ? -1 : 1;
						for( int y = v1y + incy; y != v2y; y += incy )
						{
							int val;
							VoxelRef v;
							if( World.GetVoxelRef( out v, v1x, y, v1z ) )
							{
								val = f( ref v );
								if( ( !not_zero && val != 0 ) || ( not_zero && val == 0 ) )
									return val;
							}
						}
					}
				}
				else
				{
					// no del_x, no del_y...
					if( del_z != 0 )
					{
						if( del_z > 0 )
							for( int z = v1z + 1; z < v2z; z++ )
							{
								int val;
								VoxelRef v;
								if( World.GetVoxelRef( out v, v1x, v1y, z ) )
								{
									val = f( ref v );
									if( ( !not_zero && val != 0 ) || ( not_zero && val == 0 ) )
										return val;
								}
							}
						else
							for( int z = v2z + 1; z < v1z; z++ )
							{
								int val;
								VoxelRef v;
								if( World.GetVoxelRef( out v, v1x, v1y, z ) )
								{
									val = f( ref v );
									if( ( !not_zero && val != 0 ) || ( not_zero && val == 0 ) )
										return val;
								}
							}

					}
					else
					{
						// no delta diff, nothing to do.
					}
				}
			}
			return not_zero ? 1 : 0;
		}
	}
}
