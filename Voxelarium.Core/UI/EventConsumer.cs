using OpenTK.Input;
using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.UI
{
	public interface EventConsumer
	{
		bool KeyDown( Key KeySym )  ;
		bool KeyUp( Key KeySym )    ;
		bool MouseMove( float Relative_x, float Relative_y, float Absolute_x, float Absolute_y );
		bool MouseButtonClick( MouseButton nButton, float Absolute_x, float Absolute_y );
		bool MouseButtonRelease( MouseButton nButton, float Absolute_x, float Absolute_y );
	}
}
