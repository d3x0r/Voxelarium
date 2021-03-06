﻿/*
 * This file is part of Voxelarium.
 *
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
#define ALLOW_INLINE
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace Voxelarium.Core.Voxels
{
	public struct NearVoxelRef
	{
		public VoxelSector Sector;
		public VoxelType Type;
		public uint Offset;
		public VoxelExtension VoxelExtension;
	}

	public struct VoxelRef
	{
		public VoxelSector Sector;
		public uint Offset;
		public byte x, y, z;
		public int wx, wy, wz;
		public VoxelType Type;
		public VoxelWorld World;
		internal VoxelTypeManager VoxelTypeManager;
		public VoxelExtension VoxelExtension;

		internal VoxelRef( VoxelWorld world, VoxelTypeManager vtm, byte x = 0, byte y = 0, byte z = 0, VoxelSector Sector = null, ushort VoxelType = 0 )
		{
			this.x = x;
			this.y = y;
			this.z = z;
			this.wx = ( Sector.Pos_x << VoxelSector.ZVOXELBLOCSHIFT_X )+ x;
			this.wy = ( Sector.Pos_y << VoxelSector.ZVOXELBLOCSHIFT_Y )+y;
			this.wz = ( Sector.Pos_z << VoxelSector.ZVOXELBLOCSHIFT_Z )+z;
			this.Sector = Sector;
			this.Offset = ( (uint)x << VoxelSector.ZVOXELBLOCSHIFT_Y )  + y + ((uint)z << ( VoxelSector.ZVOXELBLOCSHIFT_X+VoxelSector.ZVOXELBLOCSHIFT_Y));
			this.World = world;
			this.Type = vtm[VoxelType];
			VoxelTypeManager = vtm;
			VoxelExtension = null;
		}

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

		internal void GetVoxelRefs( out VoxelSector[] ResultSectors, out uint[] ResultOffsets, bool nearOnly = false )
		{
			ResultSectors = new VoxelSector[nearOnly ? 7 : 19];
			ResultOffsets = new uint[nearOnly ? 7 : 19];
			GetVoxelRefs( ResultSectors, ResultOffsets, nearOnly );
		}

		public static void GetNearVoxelRef( out VoxelRef that, ref VoxelRef self, VoxelSector.RelativeVoxelOrds direction )
		{
			that = self;
			switch( direction )
			{
			default:
				throw new NotImplementedException( "Creating voxel ref " + direction + " is not implemented " );
				break;
			case VoxelSector.RelativeVoxelOrds.LEFT:
				that.wx--;
				if( that.x > 0 )
				{
					that.x--;
					that.Offset -= that.Sector.Size_y;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
					{
						that.x = (byte)( that.Sector.Size_x - 1 );
						that.Offset += that.Sector.Size_y * ( that.Sector.Size_x - 2 );
					}
				}
				break;
			case VoxelSector.RelativeVoxelOrds.RIGHT:
				that.wx++;
				if( that.x < (that.Sector.Size_x-1 ) )
				{
					that.x++;
					that.Offset += that.Sector.Size_y;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
					{
						that.x = 0;
						that.Offset -= that.Sector.Size_y * ( that.Sector.Size_x - 2 );
					}
				}
				break;
			case VoxelSector.RelativeVoxelOrds.BEHIND:
				that.wz--;
				if( that.z > 0 )
				{
					that.z--;
					that.Offset -= that.Sector.Size_y*that.Sector.Size_x;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
					{
						that.z = (byte)( that.Sector.Size_z - 1 );
						that.Offset += ( that.Sector.Size_x * that.Sector.Size_y * ( that.Sector.Size_z - 2 ) );
					}
				}
				break;
			case VoxelSector.RelativeVoxelOrds.AHEAD:
				that.wz++;
				if( that.z < ( that.Sector.Size_z - 1 ) )
				{
					that.z++;
					that.Offset += that.Sector.Size_y*that.Sector.Size_x;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
					{
						that.z = 0;
						that.Offset -= ( that.Sector.Size_x * that.Sector.Size_y * ( that.Sector.Size_z - 2 ) );
					}
				}
				break;
			case VoxelSector.RelativeVoxelOrds.BELOW:
				that.wy--;
				if( that.y > 0 )
				{
					that.y--;
					that.Offset--;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
					{
						that.y = (byte)( that.Sector.Size_y - 1 );
						that.Offset += ( that.Sector.Size_y - 2 );
					}
				}
				break;
			case VoxelSector.RelativeVoxelOrds.ABOVE:
				that.wy++;
				if( that.y < ( that.Sector.Size_y - 1 ) )
				{
					that.y++;
					that.Offset++;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
					{
						that.y = 0;
						that.Offset -= ( that.Sector.Size_y - 2 );
					}
				}
				break;
			}
			if( that.Sector != null )
			{
				that.Type = that.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearVoxelRef( out NearVoxelRef that, ref VoxelRef self, VoxelSector.RelativeVoxelOrds direction )
		{
			that.Sector = self.Sector;
			that.Offset = self.Offset;
			switch( direction )
			{
			default:
				throw new NotImplementedException( "Creating voxel ref " + direction + " is not implemented " );
				break;
			case VoxelSector.RelativeVoxelOrds.LEFT:
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y ) ) != 0 )
					that.Offset -= that.Sector.Size_y;
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
						that.Offset += VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_X - 2 );
				}
				break;
			case VoxelSector.RelativeVoxelOrds.RIGHT:
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y ) ) != VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y )
					that.Offset += VoxelSector.ZVOXELBLOCSIZE_Y;
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
						that.Offset -= VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_X - 2 );
				}
				break;
			case VoxelSector.RelativeVoxelOrds.BEHIND:
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) ) != 0 )
					that.Offset -= VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
						that.Offset += ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_Z - 2 ) );
				}
				break;
			case VoxelSector.RelativeVoxelOrds.AHEAD:
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) ) != ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) )
					that.Offset += VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
						that.Offset -= ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_Z - 2 ) );
				}
				break;
			case VoxelSector.RelativeVoxelOrds.BELOW:
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Y ) ) != 0 )
					that.Offset--;
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
						that.Offset += ( VoxelSector.ZVOXELBLOCSIZE_Y - 2 );
				}
				break;
			case VoxelSector.RelativeVoxelOrds.ABOVE:
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Y ) ) != VoxelSector.ZVOXELBLOCMASK_Y )
					that.Offset++;
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
						that.Offset -= ( VoxelSector.ZVOXELBLOCSIZE_Y - 2 );
				}
				break;
			}
			if( that.Sector != null )
			{
				that.Type = self.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}


#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearLeftVoxelRef( out NearVoxelRef that, ref VoxelRef self )
		{
			that.Sector = self.Sector;
			that.Offset = self.Offset;
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y ) ) != 0 )
					that.Offset -= that.Sector.Size_y;
				else
				{
					that.Sector = self.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
					if( that.Sector != null )
						that.Offset += VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_X - 2 );
				}
			if( that.Sector != null )
			{
				that.Type = self.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearRightVoxelRef( out NearVoxelRef that, ref VoxelRef self )
		{
			that.Sector = self.Sector;
			that.Offset = self.Offset;
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y ) ) != VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y )
					that.Offset += VoxelSector.ZVOXELBLOCSIZE_Y;
				else
				{
					that.Sector = self.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
					if( that.Sector != null )
						that.Offset -= VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_X - 2 );
				}
			if( that.Sector != null )
			{
				that.Type = self.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearAheadVoxelRef( out NearVoxelRef that, ref VoxelRef self )
		{
			that.Sector = self.Sector;
			that.Offset = self.Offset;
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) ) != ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) )
					that.Offset += VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
				else
				{
					that.Sector = self.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1];
					if( that.Sector != null )
						that.Offset -= ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_Z - 2 ) );
				}
			if( that.Sector != null )
			{
				that.Type = self.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearBehindVoxelRef( out NearVoxelRef that, ref VoxelRef self )
		{
			that.Sector = self.Sector;
			that.Offset = self.Offset;
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) ) != 0 )
					that.Offset -= VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
				else
				{
					that.Sector = self.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1];
					if( that.Sector != null )
						that.Offset += ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_Z - 2 ) );
				}
			if( that.Sector != null )
			{
				that.Type = self.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearAboveVoxelRef( out NearVoxelRef that, ref VoxelRef self )
		{
			that.Sector = self.Sector;
			that.Offset = self.Offset;
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Y ) ) != VoxelSector.ZVOXELBLOCMASK_Y )
					that.Offset++;
				else
				{
					that.Sector = self.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1];
					if( that.Sector != null )
						that.Offset -= ( VoxelSector.ZVOXELBLOCSIZE_Y - 2 );
				}
			if( that.Sector != null )
			{
				that.Type = self.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearBelowVoxelRef( out NearVoxelRef that, ref VoxelRef self )
		{
			that.Sector = self.Sector;
			that.Offset = self.Offset;
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Y ) ) != 0 )
					that.Offset--;
				else
				{
					that.Sector = self.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1];
					if( that.Sector != null )
						that.Offset += ( VoxelSector.ZVOXELBLOCSIZE_Y - 2 );
				}
			if( that.Sector != null )
			{
				that.Type = self.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}


#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearVoxelRef( out NearVoxelRef that, ref NearVoxelRef self, VoxelSector.RelativeVoxelOrds direction )
		{
			that.Sector = self.Sector;
			that.Offset = self.Offset;
			switch( direction )
			{
			default:
				throw new NotImplementedException( "Creating voxel ref " + direction + " is not implemented " );
				break;
			case VoxelSector.RelativeVoxelOrds.LEFT:
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y ) ) != 0 )
				{
					that.Offset -= VoxelSector.ZVOXELBLOCSIZE_Y;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
					{
						that.Offset += VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_X - 2 );
					}
				}
				break;
			case VoxelSector.RelativeVoxelOrds.RIGHT:
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y ) ) != VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y )
				{
					that.Offset += VoxelSector.ZVOXELBLOCSIZE_Y;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
					{
						that.Offset -= VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_X - 2 );
					}
				}
				break;
			case VoxelSector.RelativeVoxelOrds.BEHIND:
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) ) != 0 )
				{
					that.Offset -= VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
					{
						that.Offset += ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_Z - 2 ) );
					}
				}
				break;
			case VoxelSector.RelativeVoxelOrds.AHEAD:
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) ) != ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) )
				{
					that.Offset += VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
					{
						that.Offset -= ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_Z - 2 ) );
					}
				}
				break;
			case VoxelSector.RelativeVoxelOrds.BELOW:
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Y ) ) != 0 )
				{
					that.Offset--;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
					{
						that.Offset += ( VoxelSector.ZVOXELBLOCSIZE_Y - 2 );
					}
				}
				break;
			case VoxelSector.RelativeVoxelOrds.ABOVE:
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Y ) ) != VoxelSector.ZVOXELBLOCMASK_Y )
				{
					that.Offset++;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)direction - 1];
					if( that.Sector != null )
					{
						that.Offset -= ( VoxelSector.ZVOXELBLOCSIZE_Y - 2 );
					}
				}
				break;
			}
			if( that.Sector != null )
			{
				that.Type = self.Sector.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearLeftVoxelRef( out NearVoxelRef that, ref NearVoxelRef self )
		{
			that.Sector = self.Sector;
			that.Offset = self.Offset;
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y ) ) != 0 )
				{
					that.Offset -= VoxelSector.ZVOXELBLOCSIZE_Y;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
					if( that.Sector != null )
					{
						that.Offset += VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_X - 2 );
					}
				}
			if( that.Sector != null )
			{
				that.Type = self.Sector.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearRightVoxelRef( out NearVoxelRef that, ref NearVoxelRef self )
		{
			that.Sector = self.Sector;
			that.Offset = self.Offset;
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y ) ) != VoxelSector.ZVOXELBLOCMASK_X << VoxelSector.ZVOXELBLOCSHIFT_Y )
				{
					that.Offset += VoxelSector.ZVOXELBLOCSIZE_Y;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
					if( that.Sector != null )
					{
						that.Offset -= VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_X - 2 );
					}
				}
			if( that.Sector != null )
			{
				that.Type = self.Sector.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearAheadVoxelRef( out NearVoxelRef that, ref NearVoxelRef self )
		{
			that.Sector = self.Sector;
			that.Offset = self.Offset;
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) ) != ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) )
				{
					that.Offset += VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1];
					if( that.Sector != null )
					{
						that.Offset -= ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_Z - 2 ) );
					}
				}
			if( that.Sector != null )
			{
				that.Type = self.Sector.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearBehindVoxelRef( out NearVoxelRef that, ref NearVoxelRef self )
		{
			that.Sector = self.Sector;
			that.Offset = self.Offset;
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) ) ) != 0 )
				{
					that.Offset -= VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1];
					if( that.Sector != null )
					{
						that.Offset += ( VoxelSector.ZVOXELBLOCSIZE_X * VoxelSector.ZVOXELBLOCSIZE_Y * ( VoxelSector.ZVOXELBLOCSIZE_Z - 2 ) );
					}
				}
			if( that.Sector != null )
			{
				that.Type = self.Sector.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearAboveVoxelRef( out NearVoxelRef that, ref NearVoxelRef self )
		{
			that.Sector = self.Sector;
			that.Offset = self.Offset;
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Y ) ) != VoxelSector.ZVOXELBLOCMASK_Y )
				{
					that.Offset++;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1];
					if( that.Sector != null )
					{
						that.Offset -= ( VoxelSector.ZVOXELBLOCSIZE_Y - 2 );
					}
				}
			if( that.Sector != null )
			{
				that.Type = self.Sector.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public static void GetNearBelowVoxelRef( out NearVoxelRef that, ref NearVoxelRef self )
		{
			that.Sector = self.Sector;
			that.Offset = self.Offset;
				if( ( that.Offset & ( VoxelSector.ZVOXELBLOCMASK_Y ) ) != 0 )
				{
					that.Offset--;
				}
				else
				{
					that.Sector = self.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1];
					if( that.Sector != null )
					{
						that.Offset += ( VoxelSector.ZVOXELBLOCSIZE_Y - 2 );
					}
				}
			if( that.Sector != null )
			{
				that.Type = self.Sector.VoxelTypeManager.VoxelTable[that.Sector.Data.Data[that.Offset]];
				that.VoxelExtension = that.Sector.Data.OtherInfos[that.Offset];
			}
			else
			{
				that.Type = null;
				that.VoxelExtension = null;
			}
		}

		internal void GetVoxelRefs( out VoxelRef[] result, bool nearOnly = true )
		{
			if( nearOnly )
			{
				result = new VoxelRef[7];
				//result[0] = this;
				GetNearVoxelRef( out result[(int)VoxelSector.RelativeVoxelOrds.LEFT]   ,ref this, VoxelSector.RelativeVoxelOrds.LEFT );
				GetNearVoxelRef( out result[(int)VoxelSector.RelativeVoxelOrds.RIGHT]  ,ref this, VoxelSector.RelativeVoxelOrds.RIGHT );
				GetNearVoxelRef( out result[(int)VoxelSector.RelativeVoxelOrds.AHEAD]  ,ref this, VoxelSector.RelativeVoxelOrds.AHEAD );
				GetNearVoxelRef( out result[(int)VoxelSector.RelativeVoxelOrds.BEHIND] ,ref this, VoxelSector.RelativeVoxelOrds.BEHIND );
				GetNearVoxelRef( out result[(int)VoxelSector.RelativeVoxelOrds.ABOVE]  ,ref this, VoxelSector.RelativeVoxelOrds.ABOVE );
				GetNearVoxelRef( out result[(int)VoxelSector.RelativeVoxelOrds.BELOW]  ,ref this, VoxelSector.RelativeVoxelOrds.BELOW );
			}
			else
			{
				result = new VoxelRef[27];
			}
			
		}

		internal void GetVoxelRefs( VoxelSector[] ResultSectors, uint[] ResultOffsets, bool nearOnly = false )
		{
			//ResultSectors = new VoxelSector[nearOnly ? 7 : 19];
			//ResultOffsets = new uint[nearOnly ? 7 : 19];

			ResultSectors[(int)VoxelSector.RelativeVoxelOrds.INCENTER] = this.Sector;
			uint origin = this.Offset;//( this.x <<VoxelSector.ZVOXELBLOCSHIFT_Y ) + this.y + ( this.z << (VoxelSector.ZVOXELBLOCSHIFT_X +VoxelSector.ZVOXELBLOCSHIFT_Y ) );
			{
				uint[] input = VoxelSector.RelativeVoxelOffsets_Unwrapped;
				int n;
				int idx = 0;
				ResultOffsets[idx] = origin + input[idx]; idx++;
				ResultOffsets[idx] = origin + input[idx]; idx++; //1
				ResultOffsets[idx] = origin + input[idx]; idx++; //2
				ResultOffsets[idx] = origin + input[idx]; idx++; //3
				ResultOffsets[idx] = origin + input[idx]; idx++; //4
				ResultOffsets[idx] = origin + input[idx]; idx++; //5
				ResultOffsets[idx] = origin + input[idx]; idx++; //6
				if( !nearOnly ) for( n = 0; n < 20; n++ ) { ResultOffsets[idx] = origin + input[idx]; idx++; }

				if( this.x == 0 )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT]
							= this.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1]; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT] = this.Sector;
					ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1, 0]] += ( VoxelSector.ZVOXELBLOCSIZE_X ) * VoxelSector.ZVOXELBLOCSIZE_Y;
					if( !nearOnly ) for( n = 1; n < 9; n++ ) ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1, n]] += ( VoxelSector.ZVOXELBLOCSIZE_X ) * VoxelSector.ZVOXELBLOCSIZE_Y;
				}
				else if( this.x == ( VoxelSector.ZVOXELBLOCSIZE_X - 1 ) )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT] = this.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
					ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1, 0]] -= ( VoxelSector.ZVOXELBLOCSIZE_X ) * VoxelSector.ZVOXELBLOCSIZE_Y;
					if( !nearOnly ) for( n = 1; n < 9; n++ ) ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1, n]] -= ( VoxelSector.ZVOXELBLOCSIZE_X ) * VoxelSector.ZVOXELBLOCSIZE_Y;
				}
				else
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT] = this.Sector;
				}
				if( this.y == 0 )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW] = this.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1];
					ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1, 0]] += ( VoxelSector.ZVOXELBLOCSIZE_Y );
					if( !nearOnly ) for( n = 1; n < 9; n++ ) ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1, n]] += ( VoxelSector.ZVOXELBLOCSIZE_Y );
				}
				else if( this.y == ( VoxelSector.ZVOXELBLOCSIZE_Y - 1 ) )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = this.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1]; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW] = this.Sector;
					ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1, 0]] -= ( VoxelSector.ZVOXELBLOCSIZE_Y );
					if( !nearOnly ) for( n = 1; n < 9; n++ ) ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1, n]] -= ( VoxelSector.ZVOXELBLOCSIZE_Y );
				}
				else
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW] = this.Sector;
				}

				if( this.z == 0 )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND] = this.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1];
					ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1, 0]] += ( VoxelSector.ZVOXELBLOCSIZE_Z ) * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
					if( !nearOnly ) for( n = 1; n < 9; n++ ) ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1, n]] += ( VoxelSector.ZVOXELBLOCSIZE_Z ) * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
				}
				else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD] = this.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1]; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND] = this.Sector;
					ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1, 0]] -= ( VoxelSector.ZVOXELBLOCSIZE_Z ) * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
					if( !nearOnly ) for( n = 1; n < 9; n++ ) ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1, n]] -= ( VoxelSector.ZVOXELBLOCSIZE_Z ) * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
				}
				else
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND] = this.Sector;
				}

				// test to make sure resulting offsets are within range.
				//for( n = 0; n < 27; n++ ) if( ResultOffsets[n] & 0xFFFF8000 ) DebugBreak();
			}
			if( nearOnly )
				return;

			if( this.x == 0 )
			{
				if( this.y == 0 )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
				}
				else if( this.y == ( VoxelSector.ZVOXELBLOCSIZE_Y - 1 ) )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT].near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT].near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else //----------------------------------------------
					{
						// left bound, top bound, front nobound
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD] = this.Sector;
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND] = this.Sector;

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
				}
				else //----------------------------------------------
				{
					// left bound, above/below unbound
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = this.Sector;
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW] = this.Sector;
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD] != null ? ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1] : null;
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else
					{
						// left bound, y unbound z unbound
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
				}
			}
			else if( this.x == ( VoxelSector.ZVOXELBLOCSIZE_X - 1 ) )
			{
				if( this.y == 0 )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT].near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW].near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
				}
				else if( this.y == ( VoxelSector.ZVOXELBLOCSIZE_Y - 1 ) )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = this.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW] = this.Sector;
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
				}
				else
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
				}
			}
			else //---------------------------------------------------------
			{
				// left/right unbound... left and right should never be terms of equality
				if( this.y == 0 )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW].near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1];
					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW].near_sectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					}
					else
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					}
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND];
				}
				else if( this.y == ( VoxelSector.ZVOXELBLOCSIZE_Y - 1 ) )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE].near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE].near_sectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					}
					else
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					}
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND];
				}
				else  //----------------------------------------------
				{
					// x not on bound, y not on bound.
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
				}
			}
		}

		// result set is only 9 (3x3 face) 
		internal void GetVoxelRefs( out VoxelSector[] ResultSectors, out uint[] ResultOffsets, VoxelSector.RelativeVoxelOrds faceOnly )
		{
			ResultSectors = new VoxelSector[27];
			ResultOffsets = new uint[27];

			ResultSectors[(int)VoxelSector.RelativeVoxelOrds.INCENTER] = this.Sector;
			int origin = (int)( this.x << VoxelSector.ZVOXELBLOCSHIFT_Y ) + this.y + ( this.z << ( VoxelSector.ZVOXELBLOCSHIFT_X + VoxelSector.ZVOXELBLOCSHIFT_Y ) );
			{
				uint[] input = VoxelSector.RelativeVoxelOffsets_Unwrapped;
				int n;
				for( n = 0; n < 9; n++ )
					ResultOffsets[n] = (uint)(origin + (int)VoxelSector.RelativeVoxelOffsets_Unwrapped[(int)VoxelSector.VoxelFaceGroups[(int)faceOnly, n]]);

				switch( faceOnly )
				{
					case VoxelSector.RelativeVoxelOrds.LEFT:
						if( this.x == 0 )
						{
							for( n = 0; n < 9; n++ )
							{
								ResultSectors[n] = this.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1]; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT] = this.Sector;
								ResultOffsets[n] += ( VoxelSector.ZVOXELBLOCSIZE_X ) * VoxelSector.ZVOXELBLOCSIZE_Y;
							}
						}
						else
						{
							ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT] = this.Sector;
						}
						break;
					case VoxelSector.RelativeVoxelOrds.RIGHT:
						if( this.x == ( VoxelSector.ZVOXELBLOCSIZE_X - 1 ) )
						{
							ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT] = this.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
							for( n = 0; n < 9; n++ ) ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1, n]] -= ( VoxelSector.ZVOXELBLOCSIZE_X ) * VoxelSector.ZVOXELBLOCSIZE_Y;
						}
						else
						{
							ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT] = this.Sector;
						}
						break;
					case VoxelSector.RelativeVoxelOrds.ABOVE:
						if( this.y == 0 )
						{
							ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW] = this.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1];
							ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1, 0]] += ( VoxelSector.ZVOXELBLOCSIZE_Y );
							for( n = 0; n < 9; n++ ) ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1, n]] += ( VoxelSector.ZVOXELBLOCSIZE_Y );
						}
						else
						{
							ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW] = this.Sector;
						}
						break;
					case VoxelSector.RelativeVoxelOrds.BELOW:
						if( this.y == ( VoxelSector.ZVOXELBLOCSIZE_Y - 1 ) )
						{
							ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = this.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1]; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW] = this.Sector;
							ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1, 0]] -= ( VoxelSector.ZVOXELBLOCSIZE_Y );
							for( n = 0; n < 9; n++ ) ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1, n]] -= ( VoxelSector.ZVOXELBLOCSIZE_Y );
						}
						else
						{
							ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW] = this.Sector;
						}
						break;
					case VoxelSector.RelativeVoxelOrds.BEHIND:
						if( this.z == 0 )
						{
							ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND] = this.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1];
							ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1, 0]] += ( VoxelSector.ZVOXELBLOCSIZE_Z ) * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
							for( n = 1; n < 9; n++ ) ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1, n]] += ( VoxelSector.ZVOXELBLOCSIZE_Z ) * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
						}
						else
						{
							ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND] = this.Sector;
						}
						break;
					case VoxelSector.RelativeVoxelOrds.AHEAD:
						if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
						{
							ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD] = this.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1]; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND] = this.Sector;
							ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1, 0]] -= ( VoxelSector.ZVOXELBLOCSIZE_Z ) * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
							for( n = 1; n < 9; n++ ) ResultOffsets[(int)VoxelSector.VoxelFaceGroups[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1, n]] -= ( VoxelSector.ZVOXELBLOCSIZE_Z ) * VoxelSector.ZVOXELBLOCSIZE_Y * VoxelSector.ZVOXELBLOCSIZE_X;
						}
						else
						{
							ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD] = this.Sector; ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND] = this.Sector;
						}
						break;
				}
				// test to make sure resulting offsets are within range.
				//for( n = 0; n < 27; n++ ) if( ResultOffsets[n] & 0xFFFF8000 ) DebugBreak();
			}


			if( this.x == 0 )
			{
				if( this.y == 0 )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
				}
				else if( this.y == ( VoxelSector.ZVOXELBLOCSIZE_Y - 1 ) )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT].near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT].near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else //----------------------------------------------
					{
						// left bound, top bound, front nobound
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD] = this.Sector;
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND] = this.Sector;

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
				}
				else //----------------------------------------------
				{
					// left bound, above/below unbound
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = this.Sector;
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW] = this.Sector;
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.LEFT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else
					{
						// left bound, y unbound z unbound
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
				}
			}
			else if( this.x == ( VoxelSector.ZVOXELBLOCSIZE_X - 1 ) )
			{
				if( this.y == 0 )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT].near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW].near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.BELOW - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
				}
				else if( this.y == ( VoxelSector.ZVOXELBLOCSIZE_Y - 1 ) )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE] = this.Sector.near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW] = this.Sector;
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
				}
				else
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD].near_sectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
					else
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW];
					}
				}
			}
			else //---------------------------------------------------------
			{
				// left/right unbound... left and right should never be terms of equality
				if( this.y == 0 )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW].near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1];
					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW].near_sectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					}
					else
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					}
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND];
				}
				else if( this.y == ( VoxelSector.ZVOXELBLOCSIZE_Y - 1 ) )
				{
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					if( this.z == 0 )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE].near_sectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];

					}
					else if( this.z == ( VoxelSector.ZVOXELBLOCSIZE_Z - 1 ) )
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE].near_sectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD - 1];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					}
					else
					{
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
						ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW];
					}
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND];
				}
				else  //----------------------------------------------
				{
					// x not on bound, y not on bound.
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT];

					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_LEFT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND_RIGHT] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];

					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_AHEAD] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.AHEAD];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.LEFT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_ABOVE_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
					ResultSectors[(int)VoxelSector.RelativeVoxelOrds.RIGHT_BELOW_BEHIND] = ResultSectors[(int)VoxelSector.RelativeVoxelOrds.BEHIND];
				}
			}
		}
	}
}
