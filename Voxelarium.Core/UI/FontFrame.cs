using OpenTK;
using Voxelarium.Core.Voxels;
using Voxelarium.Core.Voxels.Types;

namespace Voxelarium.Core.UI
{
	class FontFrame : Frame
	{
		string TextToDisplay;
		TileSet.TileStyle TileStyle;

		internal FontFrame()
		{
			FrameType = VoxelGlobalSettings.MulticharConst( 'T', 'E', 'X', 'T' );
			TileStyle = null;
			TextToDisplay = "";
		}
		internal virtual void SetDisplayText( string TextToDisplay ) { this.TextToDisplay = TextToDisplay; }
		internal virtual void SetStyle( TileSet.TileStyle TileStyle ) { this.TileStyle = TileStyle; }

		internal virtual void GetTextDisplaySize( out Vector2 OutSize )
		{
			if( TileStyle!=null )
			{
				TileStyle.TileSet.GetFontRenderSize( TileStyle, TextToDisplay, out OutSize );
			}
			else
				OutSize = new Vector2( 0 );
		}

		internal override void Render( Display render, ref Box ParentPosition )
		{
			if( Flag_Show_Master )
			{
				EffectivePosition.Position = ParentPosition.Position + Dimensions.Position;
				EffectivePosition.Position.Z += 0.1f;
                EffectivePosition.Size = Dimensions.Size;
				if( Flag_Show_Frame )
				{
					if( TileStyle != null )
						if( TileStyle.TileSet != null )
							TileStyle.TileSet.RenderFont( render, TileStyle, ref EffectivePosition, this.TextToDisplay, ref DrawColor );
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
