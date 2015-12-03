/*
 * 
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
 *
 * Created : 2015/12/02 James Buckeyne
 *
 */

/* Information for frame buffer output - http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-14-render-to-texture/ 
 */

//#define USE_GLES2
#if !USE_GLES2
using OpenTK.Graphics.OpenGL;
#else
using OpenTK.Graphics.ES20;
#endif
using System;
using System.Collections.Generic;
using System.Text;

namespace Voxelarium.Core.UI
{
	class FrameBufferObject : IDisposable
	{
		int glFBOHandle;
		int glFBOTexture; // actual target

		FrameBufferObject()
		{
			glFBOHandle = -1;
			Display.OnInvalidate += Display_OnInvalidate;
		}

		public void Dispose()
		{
			GL.DeleteFramebuffer( glFBOHandle );

		}

		private void Display_OnInvalidate()
		{
			glFBOHandle = -1;
			glFBOTexture = -1;
		}

		void GenerateBuffer()
		{
			GL.GenFramebuffers( 1, out glFBOHandle );
			GL.GenTextures( 1, out glFBOTexture );
		}

		bool SetupBuffer()
		{
			GL.BindFramebuffer( FramebufferTarget.Framebuffer, glFBOHandle );

			GL.BindTexture( TextureTarget.Texture2D, glFBOTexture );
			GL.TexImage2D(TextureTarget.Texture2D, 0, PixelInternalFormat.Rgb8, 1024, 768, 0
								, PixelFormat.Rgba, PixelType.UnsignedByte, (IntPtr)0 );
			GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Nearest );
			GL.TexParameter( TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMinFilter.Nearest );
			/* depth buffer additon
			 *
			 * GLuint depthrenderbuffer;
				 glGenRenderbuffers(1, &depthrenderbuffer);
				glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);
				glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, 1024, 768);
				glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthrenderbuffer);
			*/


			// Set "renderedTexture" as our colour attachement #0
			GL.FramebufferTexture(FramebufferTarget.Framebuffer , FramebufferAttachment.ColorAttachment0, glFBOTexture, 0 );
			// Set the list of draw buffers.
			DrawBuffersEnum[] DrawBuffers = { DrawBuffersEnum.ColorAttachment0 };
			GL.DrawBuffers( 1, DrawBuffers ); // "1" is the size of DrawBuffers


			// Always check that our framebuffer is ok
			if( GL.CheckFramebufferStatus(FramebufferTarget.Framebuffer ) != FramebufferErrorCode.FramebufferComplete )
				return false;

			return true;
		}


		void Activate()
		{
			GL.BindFramebuffer(FramebufferTarget.Framebuffer, glFBOTexture );
			GL.Viewport( 0, 0, 1024, 768 ); // Render on the whole framebuffer, complete from the lower left corner to the upper right

			// fragment shader modification     layout(location = 0) out vec3 color;
			// Note : there is no layout(location=i) in OpenGL < 3.3, but you use glFragData[i] = mvvalue anyway.
		}



		/*
		 * using the texture as an output

		 // The fullscreen quad's FBO
 2 GLuint quad_VertexArrayID;
 3 glGenVertexArrays(1, &quad_VertexArrayID);
 4 glBindVertexArray(quad_VertexArrayID);
 5 
 6 static const GLfloat g_quad_vertex_buffer_data[] = {
 7     -1.0f, -1.0f, 0.0f,
 8     1.0f, -1.0f, 0.0f,
 9     -1.0f,  1.0f, 0.0f,
10     -1.0f,  1.0f, 0.0f,
11     1.0f, -1.0f, 0.0f,
12     1.0f,  1.0f, 0.0f,
13 };
14 
15 GLuint quad_vertexbuffer;
16 glGenBuffers(1, &quad_vertexbuffer);
17 glBindBuffer(GL_ARRAY_BUFFER, quad_vertexbuffer);
18 glBufferData(GL_ARRAY_BUFFER, sizeof(g_quad_vertex_buffer_data), g_quad_vertex_buffer_data, GL_STATIC_DRAW);
19 
20 // Create and compile our GLSL program from the shaders
21 GLuint quad_programID = LoadShaders( "Passthrough.vertexshader", "SimpleTexture.fragmentshader" );
22 GLuint texID = glGetUniformLocation(quad_programID, "renderedTexture");
23 GLuint timeID = glGetUniformLocation(quad_programID, "time");
		*/
	}
}
