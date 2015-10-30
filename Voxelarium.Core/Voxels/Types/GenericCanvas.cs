using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Text;

namespace Voxelarium.Core.Voxels.Types
{
	public abstract class GenericCanvas<T> 
	{
		public class MinMax { public float Min, Max; };
		internal enum CLIP { IN = 0, LEFT = 1, RIGHT = 2, TOP = 4, BOTTOM = 8 };

		uint ElementCount;
		T[] Canvas;
		MinMax[] MinMax_H;
		MinMax[] MinMax_V;
		public int Width, Height;

		public GenericCanvas( int Width, int Height )
		{
			this.Width = Width; this.Height = Height;
			ElementCount = (uint)( Width * Height );
			Canvas = new T[ElementCount];
			MinMax_H = null;
			MinMax_V = null;
		}

		public GenericCanvas()
		{
			Width = 0;
			Height = 0;
			ElementCount = 0;
			Canvas = null;
			MinMax_H = null;
			MinMax_V = null;
		}

		void SetSize( int Width, int Height )
		{
			this.Width = Width; this.Height = Height;
			ElementCount = (uint)( Width * Height );
			Canvas = new T[ElementCount];
		}

		~GenericCanvas()
		{
			Canvas = null;
			ElementCount = 0;
			Width = Height = 0;
			MinMax_H = null;
			MinMax_V = null;
		}

		public void Clear( T ClearData = default( T ) )
		{
			uint i;

			for( i = 0; i < ElementCount; i++ ) Canvas[i] = ClearData;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public T GetPoint_Fast( int x, int y )
		{
			return ( Canvas[x + y * Width] );

		}
#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public void SetPoint_Fast( int x, int y, T Color )
		{
			Canvas[x + y * Width] = Color;
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public T GetPoint_Secure( int x, int y )
		{
			if( x < 0 || y < 0 || x >= Width || y > Height ) return ( default( T ) );
			return ( Canvas[x + y * Width] );
		}

#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		public void SetPoint_Secure( int x, int y, T Color )
		{
			if( x < 0 || y < 0 || x >= Width || y >= Height ) return;
			Canvas[x + y * Width] = Color;
		}

		public void DrawBox( int Sx, int Sy, int Ex, int Ey, T Color )
		{
			int StartX, StartY, EndX, EndY, x, y;

			// Clipping the box if it goes outside of the canva

			if( Sx < 0 ) Sx = 0;
			else if( Sx >= Width ) Sx = Width - 1;
			if( Sy < 0 ) Sy = 0;
			else if( Sy >= Height ) Sy = Height - 1;
			if( Ex < 0 ) Ex = 0;
			else if( Ex >= Width ) Ex = Width - 1;
			if( Ey < 0 ) Ey = 0;
			else if( Ey >= Height ) Ey = Height - 1;

			StartX = Sx;
			EndX = Ex;
			StartY = Sy * Width;
			EndY = Ey * Width;

			for( y = StartY; y <= EndY; y += Height )
				for( x = StartX; x <= EndX; x++ )
				{
					//if ( (x+y)>= CanvaMemSize) *(char *)0=1;
					Canvas[x + y] = Color;
				}
		}


#if ALLOW_INLINE
		[MethodImpl( MethodImplOptions.AggressiveInlining )]
#endif
		 CLIP EvalPoint( float x, float y )
		{
			CLIP Code;
			Code = CLIP.IN;
			if( x < 0 ) Code |= CLIP.LEFT;
			else if( x >= Width ) Code |= CLIP.RIGHT;
			if( y < 0 ) Code |= CLIP.TOP;
			else if( y >= Height ) Code |= CLIP.BOTTOM;
			return Code;
		}

		public bool ClipCoords( ref ZLineCoords LineCoords )
		{
			CLIP Start_ClipCode, End_ClipCode;
			float sx, sy, ex, ey, dwidth, dheight;


			sx = LineCoords.Start.x;
			sy = LineCoords.Start.y;
			ex = LineCoords.End.x;
			ey = LineCoords.End.y;
			dwidth = Width - 1;
			dheight = Height - 1;

			Start_ClipCode = EvalPoint( sx, sy );
			End_ClipCode = EvalPoint( ex, ey );

			while( true )
			{

				if( ( Start_ClipCode | End_ClipCode ) == 0 )
				{
					LineCoords.Start.x = sx; LineCoords.Start.y = sy; LineCoords.End.x = ex; LineCoords.End.y = ey;
					return ( true );  // Line is visible and in the frame, so draw it.
				}
				if( ( Start_ClipCode & End_ClipCode ) != 0 )
				{
					LineCoords.Start.x = sx; LineCoords.Start.y = sy; LineCoords.End.x = ex; LineCoords.End.y = ey;
					return ( false ); // Line is out of the frame, so don't draw it.
				}

				if( ( Start_ClipCode ) == 0 )
				{
					if( ( Start_ClipCode & CLIP.LEFT ) != 0 ) { sy = sy + ( ey - sy ) / ( ex - sx ) * ( 0 - sx ); sx = 0; }
					else if( ( Start_ClipCode & CLIP.RIGHT ) != 0 ) { sy = sy + ( ey - sy ) / ( ex - sx ) * ( dwidth - sx ); sx = dwidth; }
					else if( ( Start_ClipCode & CLIP.TOP ) != 0 ) { sx = sx + ( ex - sx ) / ( ey - sy ) * ( 0 - sy ); sy = 0; }
					else if( ( Start_ClipCode & CLIP.BOTTOM ) != 0 ) { sx = sx + ( ex - sx ) / ( ey - sy ) * ( dheight - sy ); sy = dheight; }
					Start_ClipCode = EvalPoint( sx, sy );
				}
				else
				{
					if( ( End_ClipCode & CLIP.LEFT ) != 0 ) { ey = sy + ( ey - sy ) / ( ex - sx ) * ( 0 - sx ); ex = 0; }
					else if( ( End_ClipCode & CLIP.RIGHT ) != 0 ) { ey = sy + ( ey - sy ) / ( ex - sx ) * ( dwidth - sx ); ex = dwidth; }
					else if( ( End_ClipCode & CLIP.TOP ) != 0 ) { ex = sx + ( ex - sx ) / ( ey - sy ) * ( 0 - sy ); ey = 0; }
					else if( ( End_ClipCode & CLIP.BOTTOM ) != 0 ) { ex = sx + ( ex - sx ) / ( ey - sy ) * ( dheight - sy ); ey = dheight; }
					End_ClipCode = EvalPoint( ex, ey );
				}
			}
			return ( false );
		}

		public void Polygon_Start()
		{
			int i;

			if( MinMax_H == null ) MinMax_H = new MinMax[Width];
			if( MinMax_V == null ) MinMax_V = new MinMax[Height];

			for( i = 0; i < Width; i++ ) { MinMax_H[i].Min = Width; MinMax_H[i].Max = 0; }
			for( i = 0; i < Height; i++ ) { MinMax_V[i].Min = Height; MinMax_V[i].Max = 0; }
		}

		public void Polygon_Segment( ref ZLineCoords LineCoords )
		{
			float dx, dy, Divider, x, y, dwidth, dheight;
			int xint, yint;
			int Steps;

			dwidth = Width;
			dheight = Height;
			x = LineCoords.Start.x;
			y = LineCoords.Start.y;
			dx = LineCoords.End.x - LineCoords.Start.x;
			dy = LineCoords.End.y - LineCoords.Start.y;

			Divider = ( Math.Abs( dy ) > Math.Abs( dx ) ) ? Math.Abs( dy ) : Math.Abs( dx );

			dx /= Divider;
			dy /= Divider;

			for( Steps = (int)Divider; Steps >= 0; Steps-- )
			{

				if( x >= 0.0 && x < dwidth )
				{
					xint = (int)x;
					if( y > MinMax_H[xint].Max ) MinMax_H[xint].Max = y;
					if( y < MinMax_H[xint].Min ) MinMax_H[xint].Min = y;
				}
				if( y >= 0.0 && y < dheight )
				{
					yint = (int)y;
					if( x > MinMax_V[yint].Max ) MinMax_V[yint].Max = x;
					if( x < MinMax_V[yint].Min ) MinMax_V[yint].Min = x;
				}
				x += dx;
				y += dy;
			}
		}

		public void Polygon_Render( T RenderColor )
		{
			int y, x;
			int Min, Max;
			int Dp;

			for( y = 0; y < Height; y++ )
			{
				Min = (int)( MinMax_V[y].Min + 0.5 ); Max = (int)( MinMax_V[y].Max + 0.5 );
				if( Max >= Width ) Max = Width - 1;
				if( Max < 0 ) Max = 0;
				if( Min < 0 ) Min = 0;
				if( Min >= Width ) Min = Width - 1;
				if( Max >= Min )
				{
					unsafe
					{
						Dp = ( y * Width ) + Min;
						for( x = Min; x <= Max; x++, Dp++ )
						{
							this.Canvas[Dp] = RenderColor;
						}
					}
				}
			}

			for( x = 0; x < Width; x++ )
			{
				Min = (int)MinMax_H[x].Min; Max = (int)MinMax_H[x].Max;
				if( Max >= Height ) Max = Height - 1;
				if( Max < 0 ) Max = 0;
				if( Min < 0 ) Min = 0;
				if( Min >= Height ) Min = Height - 1;
				if( Max >= Min )
				{
					Dp = x + Min * Width;
					for( y = Min; y <= Max; y++, Dp += Width )
					{
						this.Canvas[Dp] = RenderColor;
					}
				}
			}

		}

		public void DrawCircleFilled( float x, float y, float radius, T Color, float Precision )
		{
			float SqRadius, px, py;
			radius = Math.Abs( radius );
			// IntRadius = ceil(radius);


			SqRadius = radius * radius;
			// printf("Circle : [x=%fl][y=%fl][radius=%fl][Precision=%fl]\n",x,y,radius,Precision);
			for( py = -radius; py <= radius; py += Precision )
			{
				for( px = -radius; px <= radius; px += Precision )
				{
					// printf("[%fl %fl %fl =%fl,%fl %d] ", px,py, px*px + py*py,px+x,py+y, (int)(px*px + py*py) <= radius );
					if( ( px * px + py * py ) <= SqRadius ) SetPoint_Secure( (int)( px + x ), (int)( py + y ), Color );
				}
				// printf("\n");
			}
		}

		public abstract void GetFromBitmap( Bitmap Image, bool InitSizeFromImage = true, int Channel = 0 );

		public void GetFromByteTable( T[][] Table, int Width, int Height )
		{
			int x, y;

			SetSize( Width, Height );
			for( y = 0; y < Height; y++ )
			{
				for( x = 0; x < Width; x++ )
				{
					SetPoint_Secure( x, y, ( Table[y] )[x] );
				}
			}
		}

		// To optimize

		// To optimize

		public void Blit( GenericCanvas<T> Source, int xs, int ys, int xSize, int ySize, int xd, int yd )
		{
			int x, y;
			bool Reloop;


			do
			{
				Reloop = false;
				if( xSize > ( Source.Width - xs ) ) xSize = Source.Width - xs;
				if( ySize > ( Source.Height - ys ) ) ySize = Source.Height - ys;
				if( Width < ( xd + xSize ) ) xSize += Width - ( xd + xSize );
				if( Height < ( yd + ySize ) ) ySize += Height - ( yd + ySize );
				if( xd < 0 ) { xSize += xd; xs -= xd; xd = 0; Reloop = true; }
				if( yd < 0 ) { ySize += yd; ys -= yd; yd = 0; Reloop = true; }
			} while( Reloop );

			if( xSize <= 0 || ySize <= 0 ) return;

			for( y = 0; y < ySize; y++ )
				for( x = 0; x < xSize; x++ )
				{
					SetPoint_Secure( x + xd, y + yd, Source.GetPoint_Secure( x + xs, y + ys ) );
				}
		}

		public void Line( ref ZLineCoords LineCoords, T Color )
		{
			float dx, dy, x, y;
			uint i, Steps;

			// Line Clipping algorithm : Clip lines into canva and don't draw if completely out of the drawing area.

			if( !ClipCoords( ref LineCoords ) ) return;

			// Compute line delta.

			x = LineCoords.Start.x;
			y = LineCoords.Start.y;
			dx = LineCoords.End.x - LineCoords.Start.x;
			dy = LineCoords.End.y - LineCoords.Start.y;
			Steps = (uint)( ( Math.Abs( dx ) > Math.Abs( dy ) ) ? Math.Abs( dx ) : Math.Abs( dy ) );
			dx /= Steps;
			dy /= Steps;

			for( i = 0; i < Steps; i++ )
			{
				SetPoint_Secure( (int)x, (int)y, Color );
				x += dx; y += dy;
			}
			SetPoint_Secure( (int)LineCoords.End.x, (int)LineCoords.End.y, Color );

		}

		public void SearchAndReplace( T SearchData, T NewData )
		{
			int x, y;

			for( y = 0; y < Height; y++ )
				for( x = 0; x < Width; x++ )
				{
					if( SearchData.Equals( GetPoint_Fast( x, y ) ) ) SetPoint_Fast( x, y, NewData );
				}
		}

		public abstract void DumpAscii( out string Out, string TableDecl );

		public int Convert_AscToNum( char c )
		{
			if( c >= '0' && c <= '9' ) return ( c - '0' );
			if( c >= 'A' && c <= 'Z' ) return ( c - 'A' + 10 );
			if( c >= 'a' && c <= 'z' ) return ( c - 'a' + 36 );
			return ( 255 );
		}

		char Convert_NumToAsc( int Num )
		{
			if( Num >= 0 && Num <= 9 ) return ( (char)(Num + '0') );
			if( Num >= 10 && Num <= 35 ) return ( (char)(Num + 'A' - 10) );
			if( Num >= 36 && Num <= 61 ) return ( (char)(Num + 'a' - 36) );
			return ( '*' );
		}

	}
}
