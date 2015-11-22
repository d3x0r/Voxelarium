/*
 * Before porting, this header appeared inmost sources.  Of course
 * the change from C++ to C# required significant changes an no part
 * is entirely original.
 * 
 * This file is part of Blackvoxel. (Now Voxelarium)
 *
 * Copyright 2010-2014 Laurent Thiebaut & Olivia Merle
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
