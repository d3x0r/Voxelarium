using OpenTK;
using OpenTK.Graphics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Voxelarium.Common;
using Voxelarium.Core;
using Voxelarium.Core.UI;

namespace Voxelarium
{
	internal class GameDisplay : GameWindow
	{
		Display display;
		internal GameDisplay( VoxelGameEnvironment game ):base(
				  Settings.Read( "GL.Width", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Width )
				, Settings.Read( "GL.Height", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Height )
				, new OpenTK.Graphics.GraphicsMode( 32, 24, 24, 4, ColorFormat.Empty, 2, true )
				, "Voxelarium", GameWindowFlags.Default )
		{
			display = new Display( game );

			X = Settings.Read( "GL.Display.X", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.X );
			Y = Settings.Read( "GL.Display.Y", System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Y );

			this.WindowBorder = OpenTK.WindowBorder.Hidden;
			display.Mouse = Mouse;
			display.Keyboard = Keyboard;
			UpdateFrame += display.Display_UpdateFrame;
			RenderFrame += display.Display_RenderFrame;
			RenderFrame += GameDisplay_RenderFrame;
			display.Width = Width;
			display.Height = Height;
			display.X = X;
			display.Y = Y;

			KeyDown += display.Display_KeyDown;
			KeyUp += display.Display_KeyUp;
			MouseWheel += display.Display_MouseWheel;
			MouseMove += display.Display_MouseMove;
			MouseDown += display.Display_MouseDown;
			MouseUp += display.Display_MouseUp;

			Load += display.Display_Load;
			Resize += GameDisplay_Resize;
			Move += GameDisplay_Move;

			display.Exit = Exit;
			//display.SwapBuffers = SwapBuffers;
		}

		private void GameDisplay_RenderFrame( object sender, FrameEventArgs e )
		{
			SwapBuffers();
		}

		private void GameDisplay_Move( object sender, EventArgs e )
		{
			display.X = X;
			display.Y = Y;

			display.Display_Move( sender, e );
		}

		private void GameDisplay_Resize( object sender, EventArgs e )
		{
			display.Width = Width;
			display.Height = Height;
			display.Display_Resize( sender, e );
		}
	}
}
