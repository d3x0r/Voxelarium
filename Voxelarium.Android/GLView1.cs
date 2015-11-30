using System;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.ES20;
using OpenTK.Platform;
using OpenTK.Platform.Android;
using Android.Views;
using Android.Content;
using Android.Util;
using System.Reflection;
using System.IO;
using System.Data;
using System.Net.Sockets;
using OpenTK.Input;

namespace Voxelarium.Android
{
	public delegate void UpdateMethod( object unused, FrameEventArgs e );
	public delegate void SetDisplayParams( int x, int y, int width, int height );
	public delegate void SetExit( object invokeOn, MethodInfo mi );
	public delegate void KeyEvent(object sender, KeyboardKeyEventArgs e);
	public delegate bool SetDevices(KeyboardDevice keyboard, MouseDevice mouse);
	public delegate bool TouchEvent(MotionEvent e);
	//public delegate void KeyPress (object sender, KeyEventArgs e);

	public class DisplayInterface
	{
		public SetExit setExit;
		public SetDevices setDevices;
		public UpdateMethod updateMethod;
		public UpdateMethod displayMethod;
		public UpdateMethod invalidateContext;
		public KeyEvent onKeyDown;
		public KeyEvent onKeyUp;
		public TouchEvent onTouchEvent;
		public SetDisplayParams setDisplayParams;
	}

	class GLView1 : AndroidGameView
	{
		static DisplayInterface displayInterface;
		static object looderObject;

		static StringWriter sw;
		static Mono.CSharp.ReportPrinter rp ;
		static Mono.CSharp.CompilerContext cc;
		static Mono.CSharp.Report r;
		static Mono.CSharp.Evaluator e;

		void LoadGame( Context context )
		{
			if( displayInterface != null )
				return;
			//System.Reflection.Emit.AssemblyBuilder
			//Mono.Runtime.
			DataTable dt = new DataTable();
			Socket sock = null;
			Environment.CurrentDirectory = Environment.GetFolderPath( Environment.SpecialFolder.UserProfile );
			try
			{
				sock = new Socket( SocketType.Dgram, ProtocolType.IPv6 );
			}
			catch {
				try { sock = new Socket( SocketType.Dgram, ProtocolType.IP ); }
				catch{
				}
			}
			string save_path = System.Environment.GetFolderPath( System.Environment.SpecialFolder.Personal );
			//Application.Context.Assets.
			string[] files = { "BEPUphysics.dll"
				, "protobuf-net.dll"
				, "TrueTypeSharp.dll"
				, "Voxelarium.Common.dll"
				, "Voxelarium.Core.dll"
			};
			string[] slist = context.Assets.List( "" );
			if (!File.Exists ( save_path + "/assets3.exported" ) ) {
				foreach (string file in files) {
					Stream input = context.Assets.Open (file);
					Stream output = File.Create (save_path + "/" + file);
					int size = 0;// = (int)input.Length;
					byte[] data = new byte[4096];
					while( ( size = input.Read( data, 0, 4096 ) ) > 0 ) {
						output.Write( data, 0, size );
					}
					output.Dispose();
					input.Dispose();
				}
				//Stream output2 = File.Create( save_path + "/assets3.exported" );
				//output2.Dispose();

			}

			//Assembly a = Assembly.LoadFile( save_path + "/Voxelarium.Core.dll" );

			{
				if( sw == null )
					sw = new StringWriter();
				if( rp == null )
					rp = new Mono.CSharp.StreamReportPrinter( sw );
				if( cc == null )
					cc = new Mono.CSharp.CompilerContext( new Mono.CSharp.CompilerSettings(), rp );
				if( r == null )
					r = new Mono.CSharp.Report( cc, rp );
				if( e == null )
				{
					e = new Mono.CSharp.Evaluator( cc );
					e.ReferenceAssembly( typeof(GLView1).Assembly );
				}
				Mono.CSharp.CompiledMethod m;
				long now;
				now = DateTime.Now.Ticks;
				//Log.log( "About to compile" + (DateTime.Now.Ticks - now ));
				String s = e.Compile( @"
				using System;
				using System.Reflection;
				using Voxelarium.Android;
				namespace MyLoader {
				public class Loader{ 
					static Loader() { } 
					public Loader( string save_path, DisplayInterface ii, object dataTableRef, object socketRef ) { 
						Assembly a = Assembly.LoadFile( save_path + ""/Voxelarium.Core.dll"" );
						//Console.WriteLine( ""a is "" + a );
						Type gameType = null;
						Type displayType = null;
						Type[] types = a.GetTypes();
						foreach( Type t in types )
						{
							if( String.Compare( t.ToString(), ""Voxelarium.Core.UI.Display"" ) == 0 )
								displayType = t;
							if( String.Compare( t.ToString(), ""Voxelarium.Core.VoxelGameEnvironment"" ) == 0 )
								gameType = t;
						}
						object gameObject = Activator.CreateInstance( gameType );
						object displayObject = Activator.CreateInstance( displayType, new object[] { gameObject }, null );

						MethodInfo mi = displayType.GetMethod( ""Display_UpdateFrame"" );
						ii.updateMethod = (UpdateMethod)mi.CreateDelegate( typeof(UpdateMethod), displayObject );

						mi = displayType.GetMethod( ""Display_RenderFrame"" );
						ii.displayMethod = (UpdateMethod)mi.CreateDelegate( typeof(UpdateMethod), displayObject );

						mi = displayType.GetMethod( ""Display_InvalidateContext"" );
						ii.invalidateContext = (UpdateMethod)mi.CreateDelegate( typeof(UpdateMethod), displayObject );

						mi = displayType.GetMethod( ""Display_SetExit"" );
						ii.setExit = (SetExit)mi.CreateDelegate( typeof(SetExit), displayObject );

						mi = displayType.GetMethod( ""Display_KeyDown"" );
						ii.onKeyDown = (KeyEvent)mi.CreateDelegate( typeof(KeyEvent), displayObject );

						mi = displayType.GetMethod( ""Display_KeyUp"" );
						ii.onKeyUp = (KeyEvent)mi.CreateDelegate( typeof(KeyEvent), displayObject );

						mi = displayType.GetMethod( ""Display_OnTouchEvent"" );
						ii.onTouchEvent = (TouchEvent)mi.CreateDelegate( typeof(TouchEvent), displayObject );

						mi = displayType.GetMethod( ""Display_SetDisplayParams"" );
						ii.setDisplayParams = (SetDisplayParams)mi.CreateDelegate( typeof(SetDisplayParams), displayObject );					
						
						//Console.WriteLine( ""Success?"" );
					} 
				}}", out m );


				//Log.log( "did compile    " + (DateTime.Now.Ticks - now ) );
				s = sw.ToString();
				if( !s.Contains( "error" ) )
				{
					object val = e.Evaluate( "typeof( MyLoader.Loader );" );
					if( val != null )
					{
						displayInterface = new DisplayInterface();
						looderObject = Activator.CreateInstance( (Type)val, new object[] { save_path, displayInterface, dt, sock }, null );

						displayInterface.setExit( this, typeof( GLView1 ).GetMethod( "Exit" ) );

						//displayInterface.setDevices( Keyboard, Mouse );

					}
				}

				//else
				//	Log.log( "Compile Failure: " + s );
			}
			string command = Environment.CommandLine;
				
		}

		public void Exit()
		{
			System.Environment.Exit( 0 );
		}

		//Voxelarium.Core.UI.Display 
		public GLView1( Context context ) : base( context )
		{
			LoadGame( context );			
			// this happens before the context gets created... can't use this.
			// and during CreateFrameBuffer
			//this.ContextSet += GLView1_ContextSet;;
			this.ContextRenderingApi = GLVersion.ES2;
			this.KeyPress += GLView1_KeyPress;
			this.Resize += GLView1_Resize;
			this.Touch += GLView1_Touch;
		}

		void GLView1_Touch (object sender, TouchEventArgs e)
		{
			displayInterface.onTouchEvent( e.Event );
		}

		void GLView1_Resize (object sender, EventArgs e)
		{
			displayInterface.setDisplayParams( 0, 0, Width, Height );
		}

		void GLView1_KeyPress (object sender, KeyEventArgs e)
		{
			//e.KeyCode
			//displayInterface.keyPress( sender, e );
		}
		/*
		public override bool OnKeyDown(Keycode keyCode, KeyEvent e)
		{
			KeyboardKeyEventArgs args = new KeyboardKeyEventArgs();
			args.Key = keyCode;
			displayInterface.onKeyDown( null, args );
			return true;
		}
		public override bool OnKeyUp(Keycode keyCode, KeyEvent e)
		{
			KeyboardKeyEventArgs args = new KeyboardKeyEventArgs();
			args.Key = keyCode;
			displayInterface.onKeyUp( null, args );
			return true;
		}
*/
		public override bool OnTouchEvent(MotionEvent e)
		{
			Console.WriteLine( "last/first touch?" );
			if( e.PointerCount == 0 )
				displayInterface.onTouchEvent( e );
			return base.OnTouchEvent(e);
		}


		// This gets called when the drawing surface is ready
		protected override void OnLoad( EventArgs e )
		{
			base.OnLoad( e );
			// Run the render loop
			Run();
		}

		// This method is called everytime the context needs
		// to be recreated. Use it to set any egl-specific settings
		// prior to context creation
		//
		// In this particular case, we demonstrate how to set
		// the graphics mode and fallback in case the device doesn't
		// support the defaults
		protected override void CreateFrameBuffer()
		{
			// the default GraphicsMode that is set consists of (16, 16, 0, 0, 2, false)
			try
			{
				// if you don't call this, the context won't be created
				base.CreateFrameBuffer();
				displayInterface.invalidateContext( null, null );
				return;
			}
			catch( Exception ex )
			{
				Log.Verbose( "GLCube", "{0}", ex );
			}

			// this is a graphics setting that sets everything to the lowest mode possible so
			// the device returns a reliable graphics setting.
			try
			{
				//Log.Verbose( "GLCube", "Loading with custom Android settings (low mode)" );
				GraphicsMode = new AndroidGraphicsMode( 0, 0, 0, 0, 0, false );

				// if you don't call this, the context won't be created
				base.CreateFrameBuffer();
				return;
			}
			catch( Exception ex )
			{
				Log.Verbose( "GLCube", "{0}", ex );
			}
			throw new Exception( "Can't load egl, aborting" );
		}

		protected override void OnUpdateFrame(FrameEventArgs e)
		{
			base.OnUpdateFrame(e);
			displayInterface.updateMethod( this, e );
		}

		// This gets called on each frame render
		protected override void OnRenderFrame( FrameEventArgs e )
		{
			base.OnRenderFrame( e );
			displayInterface.displayMethod( null, e );

			SwapBuffers();
		}

	}
}
