/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///original version written by Erwin Coumans, October 2013

#if ! BT_DANTZIG_SOLVER_H
#define BT_DANTZIG_SOLVER_H

#include "btMLCPSolverInterface.h"
#include "btDantzigLCP.h"


class btDantzigSolver : btMLCPSolverInterface
{
protected:

	double m_acceptableUpperLimitSolution;

	List<char>	m_tempBuffer;

	List<double> m_A;
	List<double> m_b;
	List<double> m_x;
	List<double> m_lo;
	List<double> m_hi;
	List<int>	m_dependencies;
	btDantzigScratchMemory m_scratchMemory;
public:

	btDantzigSolver()
		:m_acceptableUpperLimitSolution((double)(1000))
	{
	}

	virtual bool solveMLCP(btMatrixXu & A, btVectorXu & b, btVectorXu& x, btVectorXu & lo,btVectorXu & hi,btAlignedObjectArray<int>& limitDependency, int numIterations, bool useSparsity = true)
	{
		bool result = true;
		int n = b.rows();
		if (n)
		{
			int nub = 0;
			List<double> ww;
			ww.resize(n);
	

			double* Aptr = A.getBufferPointer();
			m_A.resize(n*n);
			for (int i=0;i<n*n;i++)
			{
				m_A[i] = Aptr[i];

			}

			m_b.resize(n);
			m_x.resize(n);
			m_lo.resize(n);
			m_hi.resize(n);
			m_dependencies.resize(n);
			for (int i=0;i<n;i++)
			{
				m_lo[i] = lo[i];
				m_hi[i] = hi[i];
				m_b[i] = b[i];
				m_x[i] = x[i];
				m_dependencies[i] = limitDependency[i];
			}


			result = btSolveDantzigLCP (n,m_A,m_x,m_b,ww,nub,m_lo,m_hi,m_dependencies,m_scratchMemory);
			if (!result)
				return result;

//			Console.WriteLine("numAllocas = %d\n",numAllocas);
			for (int i=0;i<n;i++)
			{
				volatile double xx = m_x[i];
				if (xx != m_x[i])
					return false;
				if (x[i] >= m_acceptableUpperLimitSolution)
				{
					return false;
				}

				if (x[i] <= -m_acceptableUpperLimitSolution)
				{
					return false;
				}
			}

			for (int i=0;i<n;i++)
			{
				x[i] = m_x[i];
			}
			
		}

		return result;
	}
};

#endif //BT_DANTZIG_SOLVER_H
