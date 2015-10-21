/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

using Bullet.Types;

namespace Bullet.LinearMath
{

	/* used by btPolyhedralConvexShape only */
	internal static class btGeometryUtil
	{

		public static bool isPointInsidePlanes( btList<btVector3> planeEquations
							, ref btVector3 point, double margin )
		{
			btVector3[] pe = planeEquations.InternalArray;
			int numbrushes = planeEquations.Count;
			for( int i = 0; i < numbrushes; i++ )
			{
				double dist = ( pe[i].dot( ref point ) ) + (double)( pe[i][3] ) - margin;
				if( dist > btScalar.BT_ZERO )
				{
					return false;
				}
			}
			return true;
		}


		public static bool areVerticesBehindPlane( ref btVector3 planeNormal, btList<btVector3> vertices, double margin )
		{
			btVector3[] va = vertices.InternalArray;
			int numvertices = vertices.Count;
			for( int i = 0; i < numvertices; i++ )
			{
				//ref btVector3 N1 = vertices[i];
				double dist = (double)( planeNormal.dot( ref va[i] ) ) + (double)( planeNormal[3] ) - margin;
				if( dist > btScalar.BT_ZERO )
				{
					return false;
				}
			}
			return true;
		}


		static bool notExist( ref btVector3 planeEquation, btList<btVector3> planeEquations )
		{
			int numbrushes = planeEquations.Count;
			for( int i = 0; i < numbrushes; i++ )
			{
				btVector3 N1 = planeEquations[i];
				if( planeEquation.dot( ref N1 ) > (double)( 0.999 ) )
				{
					return false;
				}
			}
			return true;
		}

		public static void getPlaneEquationsFromVertices( btList<btVector3> vertices, btList<btVector3> planeEquationsOut )
		{
			int numvertices = vertices.Count;
			// brute force:
			for( int i = 0; i < numvertices; i++ )
			{
				btVector3 N1 = vertices[i];


				for( int j = i + 1; j < numvertices; j++ )
				{
					btVector3 N2 = vertices[j];

					for( int k = j + 1; k < numvertices; k++ )
					{

						btVector3 N3 = vertices[k];

						btVector3 planeEquation, edge0, edge1;
						N2.Sub( ref N1, out edge0 );
						N3.Sub( ref N1, out edge1 );
						double normalSign = (double)( 1.0 );
						for( int ww = 0; ww < 2; ww++ )
						{
							btVector3 tmp;
							edge0.cross( ref edge1, out tmp );
							tmp.Mult( normalSign, out planeEquation );
							if( planeEquation.length2() > (double)( 0.0001 ) )
							{
								planeEquation.normalize();
								if( notExist( ref planeEquation, planeEquationsOut ) )
								{
									planeEquation[3] = -planeEquation.dot( ref N1 );

									//check if inside, and replace supportingVertexOut if needed
									if( areVerticesBehindPlane( ref planeEquation, vertices, (double)( 0.01 ) ) )
									{
										planeEquationsOut.Add( planeEquation );
									}
								}
							}
							normalSign = (double)( -1.0 );
						}

					}
				}
			}

		}

		public static void getVerticesFromPlaneEquations( btList<btVector3> planeEquations, btList<btVector3> verticesOut )
		{
			int numbrushes = planeEquations.Count;
			// brute force:
			for( int i = 0; i < numbrushes; i++ )
			{
				btVector3 N1 = planeEquations[i];


				for( int j = i + 1; j < numbrushes; j++ )
				{
					btVector3 N2 = planeEquations[j];

					for( int k = j + 1; k < numbrushes; k++ )
					{

						btVector3 N3 = planeEquations[k];

						btVector3 n2n3;
						N2.cross( ref N3, out n2n3 );
						btVector3 n3n1;
						N3.cross( ref N1, out n3n1 );
						btVector3 n1n2;
						N1.cross( ref N2, out n1n2 );

						if( ( n2n3.length2() > (double)( 0.0001 ) ) &&
							 ( n3n1.length2() > (double)( 0.0001 ) ) &&
							 ( n1n2.length2() > (double)( 0.0001 ) ) )
						{
							//point P out of 3 plane equations:

							//	d1 ( N2 * N3 ) + d2 ( N3 * N1 ) + d3 ( N1 * N2 )  
							//P =  -------------------------------------------------------------------------  
							//   N1 . ( N2 * N3 )  


							double quotient = ( N1.dot( ref n2n3 ) );
							if( btScalar.btFabs( quotient ) > (double)( 0.000001 ) )
							{
								quotient = (double)( -1.0 ) / quotient;

								n2n3.Mult( N1[3], out n2n3 );
								n3n1.Mult( N2[3], out n3n1 );
								n1n2.Mult( N3[3], out n1n2 );
								btVector3 potentialVertex = n2n3;
								potentialVertex.Add( ref n3n1, out potentialVertex );
								potentialVertex.Add( ref n1n2, out potentialVertex );
								potentialVertex.Mult( quotient, out potentialVertex );

								//check if inside, and replace supportingVertexOut if needed
								if( isPointInsidePlanes( planeEquations, ref potentialVertex, (double)( 0.01 ) ) )
								{
									verticesOut.Add( potentialVertex );
								}
							}
						}
					}
				}
			}
		}

	}
}