using OpenTK;
using Voxelarium.Core.Voxels;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.UI
{
	class FontFrame : Frame
	{
		string TextToDisplay;
		//TileSet.TileStyle TileStyle;
		FontRenderer font;
		float line_height;
		internal FontFrame()
		{
			FrameType = VoxelGlobalSettings.MulticharConst( 'T', 'E', 'X', 'T' );
			TextToDisplay = "";
		}
		internal virtual void SetDisplayText( string TextToDisplay ) { this.TextToDisplay = TextToDisplay; }
		//internal virtual void Style( TileSet.TileStyle TileStyle ) { this.TileStyle = TileStyle; }
		internal virtual FontRenderer Font { set { font = value; } }
		internal virtual float FontSize { set { line_height = value; } get { return line_height; } }
		internal virtual void GetTextDisplaySize( out Vector2 OutSize )
		{
			if( font != null )
			{
				Vector2 target_size;
				font.GetFontRenderSize( TextToDisplay, out target_size );
				OutSize.X = ( target_size.X * line_height ) / target_size.Y;
				OutSize.Y = ( line_height );
			}
			else
				OutSize = new Vector2( 0 );
		}

		internal override void Render( Display render, ref Box ParentPosition )
		{
			if( Flag_Show_Master )
			{
				EffectivePosition.Position = ParentPosition.Position + Dimensions.Position;
				EffectivePosition.Position.Z -= 0.1f;
                EffectivePosition.Size = Dimensions.Size;
				if( Flag_Show_Frame )
				{
					if( font != null )
						font.RenderFont( render, ref EffectivePosition, line_height
                            , this.TextToDisplay, ref DrawColor );
				}

				// Render child frames
				if( Flag_Show_Childs )
				{
					//SubFrameList.Dump();
					foreach( Frame frame in SubFrameList )
					{
						frame.Render( render, ref EffectivePosition );
					}
				}

			}

		}
	}
}
