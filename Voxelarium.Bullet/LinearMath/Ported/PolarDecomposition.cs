
namespace Bullet.LinearMath
{

	class btPolarDecomposition
	{
		public const double DEFAULT_TOLERANCE = 0.0001;
		public const uint DEFAULT_MAX_ITERATIONS = 16;

		double m_tolerance;
		uint m_maxIterations;

		public double abs_column_sum( ref btMatrix3x3 a, int i )
		{
			return btScalar.btFabs( a[0, i] ) + btScalar.btFabs( a[1, i] ) + btScalar.btFabs( a[2, i] );
		}

		public double abs_row_sum( btMatrix3x3 a, int i )
		{
			return btScalar.btFabs( a[i, 0] ) + btScalar.btFabs( a[i, 1] ) + btScalar.btFabs( a[i, 2] );
		}

		public double p1_norm( btMatrix3x3 a )
		{
			double sum0 = abs_column_sum( ref a, 0 );
			double sum1 = abs_column_sum( ref a, 1 );
			double sum2 = abs_column_sum( ref a, 2 );
			return btScalar.btMax( btScalar.btMax( sum0, sum1 ), sum2 );
		}

		double pinf_norm( btMatrix3x3 a )
		{
			double sum0 = abs_row_sum( a, 0 );
			double sum1 = abs_row_sum( a, 1 );
			double sum2 = abs_row_sum( a, 2 );
			return btScalar.btMax( btScalar.btMax( sum0, sum1 ), sum2 );
		}

		/*
		 * Creates an instance with optional parameters.
		 *
		 * @param tolerance     - the tolerance used to determine convergence of the
		 *                        algorithm
		 * @param maxIterations - the maximum number of iterations used to achieve
		 *                        convergence
		 */
		public btPolarDecomposition( double tolerance = DEFAULT_TOLERANCE, uint maxIterations = DEFAULT_MAX_ITERATIONS )
		{
			m_tolerance = ( tolerance );
			m_maxIterations = ( maxIterations );
		}

		/*
		 * Decomposes a matrix into orthogonal and symmetric, positive-definite
		 * parts. If the number of iterations returned by this function is equal to
		 * the maximum number of iterations, the algorithm has failed to converge.
		 *
		 * @param a - the original matrix
		 * @param u - the resulting orthogonal matrix
		 * @param h - the resulting symmetric matrix
		 *
		 * @return the number of iterations performed by the algorithm.
		 */
		public uint decompose( ref btMatrix3x3 a, out btMatrix3x3 u, out btMatrix3x3 h )
		{
			// Use the 'u' and 'h' matrices for intermediate calculations
			u = a;
			a.inverse( out h );
			btMatrix3x3 tmp;
			btMatrix3x3 tmp2;
			btMatrix3x3 tmp3;

			for( uint i = 0; i < m_maxIterations; ++i )
			{
				double h_1 = p1_norm( h );
				double h_inf = pinf_norm( h );
				double u_1 = p1_norm( u );
				double u_inf = pinf_norm( u );

				double h_norm = h_1 * h_inf;
				double u_norm = u_1 * u_inf;

				// The matrix is effectively singular so we cannot invert it
				if( btScalar.btFuzzyZero( h_norm ) || btScalar.btFuzzyZero( u_norm ) )
					break;

				double gamma = btScalar.btPow( h_norm / u_norm, 0.25f );
				double inv_gamma = (double)( 1.0 ) / gamma;

				// Determine the delta to 'u'
				u.Mult( gamma - (double)( 2.0 ), out tmp );
				h.transpose( out tmp2 );
				tmp2.Mult( inv_gamma, out tmp3 );
				tmp.Add( ref tmp3, out tmp2 );
				btMatrix3x3 delta;
				tmp2.Mult( 0.5, out delta );
				//const btMatrix3x3 delta = ( u * ( gamma - btScalar( 2.0 ) ) + h.transpose() * inv_gamma ) * btScalar( 0.5 );

				// Update the matrices
				u.Add( ref delta, out u );
				//u += delta;
				u.inverse( out h );

				// Check for convergence
				if( p1_norm( delta ) <= m_tolerance * u_1 )
				{
					u.transpose( out tmp );
					tmp.Mult( ref a, out h );
					//h = u.transpose() * a;
					h.transpose( out tmp );
					h.Add( ref tmp, out tmp2 );
					tmp2.Mult( 0.5, out h );
					//h = ( h + h.transpose() ) * 0.5;
					return i;
				}
			}

			// The algorithm has failed to converge to the specified tolerance, but we
			// want to make sure that the matrices returned are in the right form.
			u.transpose( out tmp );
			tmp.Mult( ref a, out h );
			//h = u.transpose() * a;
			h.transpose( out tmp );
			h.Add( ref tmp, out tmp2 );
			tmp2.Mult( 0.5, out h );
			//h = ( h + h.transpose() ) * 0.5;

			return m_maxIterations;
		}

		/*
		 * Returns the maximum number of iterations that this algorithm will perform
		 * to achieve convergence.
		 *
		 * @return maximum number of iterations
		 */
		public uint maxIterations()
		{
			return m_maxIterations;
		}

		public uint polarDecompose( ref btMatrix3x3 a, out btMatrix3x3 u, out btMatrix3x3 h )
		{
			return decompose( ref a, out u, out h );
		}
	}
}
