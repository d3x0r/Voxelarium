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

#include "btMLCPSolver.h"
#include "LinearMath/btMatrixX.h"
#include "LinearMath/btQuickprof.h"
#include "btSolveProjectedGaussSeidel.h"


btMLCPSolver::btMLCPSolver(	 btMLCPSolverInterface* solver)
:m_solver(solver),
m_fallback(0),
m_cfm(0.000001)//0.0000001
{
}

btMLCPSolver::~btMLCPSolver()
{
}

bool gUseMatrixMultiply = false;
bool interleaveContactAndFriction = false;

double btMLCPSolver::solveGroupCacheFriendlySetup(btCollisionObject bodies, int numBodiesUnUsed, btPersistentManifold[] manifoldPtr, int numManifolds,btTypedConstraint[] constraints,int numConstraints,btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer)
{
	btSequentialImpulseConstraintSolver::solveGroupCacheFriendlySetup( bodies, numBodiesUnUsed, manifoldPtr, numManifolds,constraints,numConstraints,infoGlobal,debugDrawer);

	{
		CProfileSample sample = new CProfileSample("gather constraint data");

		int numFrictionPerContact = m_tmpSolverContactConstraintPool.Count==m_tmpSolverContactFrictionConstraintPool.Count? 1 : 2;


	//	int numBodies = m_tmpSolverBodyPool.Count;
		m_allConstraintPtrArray.resize(0);
		m_limitDependencies.resize(m_tmpSolverNonContactConstraintPool.Count+m_tmpSolverContactConstraintPool.Count+m_tmpSolverContactFrictionConstraintPool.Count);
		Debug.Assert(m_limitDependencies.Count == m_tmpSolverNonContactConstraintPool.Count+m_tmpSolverContactConstraintPool.Count+m_tmpSolverContactFrictionConstraintPool.Count);
	//	Console.WriteLine("m_limitDependencies.Count = %d\n",m_limitDependencies.Count);

		int dindex = 0;
		for (int i=0;i<m_tmpSolverNonContactConstraintPool.Count;i++)
		{
			m_allConstraintPtrArray.Add(&m_tmpSolverNonContactConstraintPool[i]);
			m_limitDependencies[dindex++] = -1;
		}
 
		///The btSequentialImpulseConstraintSolver moves all friction constraints at the very end, we can also interleave them instead
		
		int firstContactConstraintOffset=dindex;

		if (interleaveContactAndFriction)
		{
			for (int i=0;i<m_tmpSolverContactConstraintPool.Count;i++)
			{
				m_allConstraintPtrArray.Add(&m_tmpSolverContactConstraintPool[i]);
				m_limitDependencies[dindex++] = -1;
				m_allConstraintPtrArray.Add(&m_tmpSolverContactFrictionConstraintPool[i*numFrictionPerContact]);
				int findex = (m_tmpSolverContactFrictionConstraintPool[i*numFrictionPerContact].m_frictionIndex*(1+numFrictionPerContact));
				m_limitDependencies[dindex++] = findex +firstContactConstraintOffset;
				if (numFrictionPerContact==2)
				{
					m_allConstraintPtrArray.Add(&m_tmpSolverContactFrictionConstraintPool[i*numFrictionPerContact+1]);
					m_limitDependencies[dindex++] = findex+firstContactConstraintOffset;
				}
			}
		} else
		{
			for (int i=0;i<m_tmpSolverContactConstraintPool.Count;i++)
			{
				m_allConstraintPtrArray.Add(&m_tmpSolverContactConstraintPool[i]);
				m_limitDependencies[dindex++] = -1;
			}
			for (int i=0;i<m_tmpSolverContactFrictionConstraintPool.Count;i++)
			{
				m_allConstraintPtrArray.Add(&m_tmpSolverContactFrictionConstraintPool[i]);
				m_limitDependencies[dindex++] = m_tmpSolverContactFrictionConstraintPool[i].m_frictionIndex+firstContactConstraintOffset;
			}
			
		}


		if (!m_allConstraintPtrArray.Count)
		{
			m_A.resize(0,0);
			m_b.resize(0);
			m_x.resize(0);
			m_lo.resize(0);
			m_hi.resize(0);
			return 0;
		}
	}

	
	if (gUseMatrixMultiply)
	{
		CProfileSample sample = new CProfileSample("createMLCP");
		createMLCP(infoGlobal);
	}
	else
	{
		CProfileSample sample = new CProfileSample("createMLCPFast");
		createMLCPFast(infoGlobal);
	}

	return 0;
}

bool btMLCPSolver::solveMLCP(btContactSolverInfo& infoGlobal)
{
	bool result = true;

	if (m_A.rows()==0)
		return true;

	//if using split impulse, we solve 2 separate (M)LCPs
	if (infoGlobal.m_splitImpulse)
	{
		btMatrixXu Acopy = m_A;
		List<int> limitDependenciesCopy = m_limitDependencies;
//		Console.WriteLine("solve first LCP\n");
		result = m_solver.solveMLCP(m_A, m_b, m_x, m_lo,m_hi, m_limitDependencies,infoGlobal.m_numIterations );
		if (result)
			result = m_solver.solveMLCP(Acopy, m_bSplit, m_xSplit, m_lo,m_hi, limitDependenciesCopy,infoGlobal.m_numIterations );

	} else
	{
		result = m_solver.solveMLCP(m_A, m_b, m_x, m_lo,m_hi, m_limitDependencies,infoGlobal.m_numIterations );
	}
	return result;
}

struct btJointNode
{
	int jointIndex;     // pointer to enclosing dxJoint object
	int otherBodyIndex;       // *other* body this joint is connected to
	int nextJointNodeIndex;//-1 for null
	int constraintRowIndex;
};



void btMLCPSolver::createMLCPFast(btContactSolverInfo& infoGlobal)
{
	int numContactRows = interleaveContactAndFriction ? 3 : 1;

	int numConstraintRows = m_allConstraintPtrArray.Count;
	int n = numConstraintRows;
	{
		CProfileSample sample = new CProfileSample("init b (rhs)");
		m_b.resize(numConstraintRows);
		m_bSplit.resize(numConstraintRows);
		m_b.setZero();
		m_bSplit.setZero();
		for (int i=0;i<numConstraintRows ;i++)
		{
			double jacDiag = m_allConstraintPtrArray[i].m_jacDiagABInv;
			if (!btFuzzyZero(jacDiag))
			{
				double rhs = m_allConstraintPtrArray[i].m_rhs;
				double rhsPenetration = m_allConstraintPtrArray[i].m_rhsPenetration;
				m_b[i]=rhs/jacDiag;
				m_bSplit[i] = rhsPenetration/jacDiag;
			}

		}
	}

//	double* w = 0;
//	int nub = 0;

	m_lo.resize(numConstraintRows);
	m_hi.resize(numConstraintRows);
 
	{
		CProfileSample sample = new CProfileSample("init lo/ho");

		for (int i=0;i<numConstraintRows;i++)
		{
			if (0)//m_limitDependencies[i]>=0)
			{
				m_lo[i] = -BT_INFINITY;
				m_hi[i] = BT_INFINITY;
			} else
			{
				m_lo[i] = m_allConstraintPtrArray[i].m_lowerLimit;
				m_hi[i] = m_allConstraintPtrArray[i].m_upperLimit;
			}
		}
	}

	//
	int m=m_allConstraintPtrArray.Count;

	int numBodies = m_tmpSolverBodyPool.Count;
	List<int> bodyJointNodeArray;
	{
		CProfileSample sample = new CProfileSample("bodyJointNodeArray.resize");
		bodyJointNodeArray.resize(numBodies,-1);
	}
	List<btJointNode> jointNodeArray;
	{
		CProfileSample sample = new CProfileSample("jointNodeArray.reserve");
		jointNodeArray.reserve(2*m_allConstraintPtrArray.Count);
	}

	static btMatrixXu J3;
	{
		CProfileSample sample = new CProfileSample("J3.resize");
		J3.resize(2*m,8);
	}
	static btMatrixXu JinvM3;
	{
		CProfileSample sample = new CProfileSample("JinvM3.resize/setZero");

		JinvM3.resize(2*m,8);
		JinvM3.setZero();
		J3.setZero();
	}
	int cur=0;
	int rowOffset = 0;
	static btList<int> ofs;
	{
		CProfileSample sample = new CProfileSample("ofs resize");
		ofs.resize(0);
		ofs.resizeNoInitialize(m_allConstraintPtrArray.Count);
	}				
	{
		CProfileSample sample = new CProfileSample("Compute J and JinvM");
		int c=0;

		int numRows = 0;

		for (int i=0;i<m_allConstraintPtrArray.Count;i+=numRows,c++)
		{
			ofs[c] = rowOffset;
			int sbA = m_allConstraintPtrArray[i].m_solverBodyIdA;
			int sbB = m_allConstraintPtrArray[i].m_solverBodyIdB;
			btRigidBody orgBodyA = m_tmpSolverBodyPool[sbA].m_originalBody;
			btRigidBody orgBodyB = m_tmpSolverBodyPool[sbB].m_originalBody;

			numRows = i<m_tmpSolverNonContactConstraintPool.Count ? m_tmpConstraintSizesPool[c].m_numConstraintRows : numContactRows ;
			if (orgBodyA)
			{
				{
					int slotA=-1;
					//find free jointNode slot for sbA
					slotA =jointNodeArray.Count;
					jointNodeArray.expand();//NonInitializing();
					int prevSlot = bodyJointNodeArray[sbA];
					bodyJointNodeArray[sbA] = slotA;
					jointNodeArray[slotA].nextJointNodeIndex = prevSlot;
					jointNodeArray[slotA].jointIndex = c;
					jointNodeArray[slotA].constraintRowIndex = i;
					jointNodeArray[slotA].otherBodyIndex = orgBodyB ? sbB : -1;
				}
				for (int row=0;row<numRows;row++,cur++)
				{
					btVector3 normalInvMass =				m_allConstraintPtrArray[i+row].m_contactNormal1 *		orgBodyA.getInvMass();
					btVector3 relPosCrossNormalInvInertia = m_allConstraintPtrArray[i+row].m_relpos1CrossNormal *	orgBodyA.getInvInertiaTensorWorld();

					for (int r=0;r<3;r++)
					{
						J3.setElem(cur,r,m_allConstraintPtrArray[i+row].m_contactNormal1[r]);
						J3.setElem(cur,r+4,m_allConstraintPtrArray[i+row].m_relpos1CrossNormal[r]);
						JinvM3.setElem(cur,r,normalInvMass[r]);
						JinvM3.setElem(cur,r+4,relPosCrossNormalInvInertia[r]);
					}
					J3.setElem(cur,3,0);
					JinvM3.setElem(cur,3,0);
					J3.setElem(cur,7,0);
					JinvM3.setElem(cur,7,0);
				}
			} else
			{
				cur += numRows;
			}
			if (orgBodyB)
			{

				{
					int slotB=-1;
					//find free jointNode slot for sbA
					slotB =jointNodeArray.Count;
					jointNodeArray.expand();//NonInitializing();
					int prevSlot = bodyJointNodeArray[sbB];
					bodyJointNodeArray[sbB] = slotB;
					jointNodeArray[slotB].nextJointNodeIndex = prevSlot;
					jointNodeArray[slotB].jointIndex = c;
					jointNodeArray[slotB].otherBodyIndex = orgBodyA ? sbA : -1;
					jointNodeArray[slotB].constraintRowIndex = i;
				}

				for (int row=0;row<numRows;row++,cur++)
				{
					btVector3 normalInvMassB = m_allConstraintPtrArray[i+row].m_contactNormal2*orgBodyB.getInvMass();
					btVector3 relPosInvInertiaB = m_allConstraintPtrArray[i+row].m_relpos2CrossNormal * orgBodyB.getInvInertiaTensorWorld();

					for (int r=0;r<3;r++)
					{
						J3.setElem(cur,r,m_allConstraintPtrArray[i+row].m_contactNormal2[r]);
						J3.setElem(cur,r+4,m_allConstraintPtrArray[i+row].m_relpos2CrossNormal[r]);
						JinvM3.setElem(cur,r,normalInvMassB[r]);
						JinvM3.setElem(cur,r+4,relPosInvInertiaB[r]);
					}
					J3.setElem(cur,3,0);
					JinvM3.setElem(cur,3,0);
					J3.setElem(cur,7,0);
					JinvM3.setElem(cur,7,0);
				}
			}
			else
			{
				cur += numRows;
			}
			rowOffset+=numRows;

		}
		
	}


	//compute JinvM = J*invM.
	double* JinvM = JinvM3.getBufferPointer();

	double* Jptr = J3.getBufferPointer();
	{
		CProfileSample sample = new CProfileSample("m_A.resize");
		m_A.resize(n,n);
	}
	
	{
		CProfileSample sample = new CProfileSample("m_A.setZero");
		m_A.setZero();
	}
	int c=0;
	{
		int numRows = 0;
		CProfileSample sample = new CProfileSample("Compute A");
		for (int i=0;i<m_allConstraintPtrArray.Count;i+= numRows,c++)
		{
			int row__ = ofs[c];
			int sbA = m_allConstraintPtrArray[i].m_solverBodyIdA;
			int sbB = m_allConstraintPtrArray[i].m_solverBodyIdB;
		//	btRigidBody orgBodyA = m_tmpSolverBodyPool[sbA].m_originalBody;
		//	btRigidBody orgBodyB = m_tmpSolverBodyPool[sbB].m_originalBody;

			numRows = i<m_tmpSolverNonContactConstraintPool.Count ? m_tmpConstraintSizesPool[c].m_numConstraintRows : numContactRows ;
					
			double *JinvMrow = JinvM + 2*8*(size_t)row__;

			{
				int startJointNodeA = bodyJointNodeArray[sbA];
				while (startJointNodeA>=0)
				{
					int j0 = jointNodeArray[startJointNodeA].jointIndex;
					int cr0 = jointNodeArray[startJointNodeA].constraintRowIndex;
					if (j0<c)
					{
								 
						int numRowsOther = cr0 < m_tmpSolverNonContactConstraintPool.Count ? m_tmpConstraintSizesPool[j0].m_numConstraintRows : numContactRows;
						size_t ofsother = (m_allConstraintPtrArray[cr0].m_solverBodyIdB == sbA) ? 8*numRowsOther  : 0;
						//Console.WriteLine("%d joint i %d and j0: %d: ",count++,i,j0);
						m_A.multiplyAdd2_p8r ( JinvMrow, 
						Jptr + 2*8*(size_t)ofs[j0] + ofsother, numRows, numRowsOther,  row__,ofs[j0]);
					}
					startJointNodeA = jointNodeArray[startJointNodeA].nextJointNodeIndex;
				}
			}

			{
				int startJointNodeB = bodyJointNodeArray[sbB];
				while (startJointNodeB>=0)
				{
					int j1 = jointNodeArray[startJointNodeB].jointIndex;
					int cj1 = jointNodeArray[startJointNodeB].constraintRowIndex;

					if (j1<c)
					{
						int numRowsOther =  cj1 < m_tmpSolverNonContactConstraintPool.Count ? m_tmpConstraintSizesPool[j1].m_numConstraintRows : numContactRows;
						size_t ofsother = (m_allConstraintPtrArray[cj1].m_solverBodyIdB == sbB) ? 8*numRowsOther  : 0;
						m_A.multiplyAdd2_p8r ( JinvMrow + 8*(size_t)numRows, 
						Jptr + 2*8*(size_t)ofs[j1] + ofsother, numRows, numRowsOther, row__,ofs[j1]);
					}
					startJointNodeB = jointNodeArray[startJointNodeB].nextJointNodeIndex;
				}
			}
		}

		{
			CProfileSample sample = new CProfileSample("compute diagonal");
			// compute diagonal blocks of m_A

			int  row__ = 0;
			int numJointRows = m_allConstraintPtrArray.Count;

			int jj=0;
			for (;row__<numJointRows;)
			{

				//int sbA = m_allConstraintPtrArray[row__].m_solverBodyIdA;
				int sbB = m_allConstraintPtrArray[row__].m_solverBodyIdB;
			//	btRigidBody orgBodyA = m_tmpSolverBodyPool[sbA].m_originalBody;
				btRigidBody orgBodyB = m_tmpSolverBodyPool[sbB].m_originalBody;


				string nsigned int infom =  row__ < m_tmpSolverNonContactConstraintPool.Count ? m_tmpConstraintSizesPool[jj].m_numConstraintRows : numContactRows;
				
				double *JinvMrow = JinvM + 2*8*(size_t)row__;
				double *Jrow = Jptr + 2*8*(size_t)row__;
				m_A.multiply2_p8r (JinvMrow, Jrow, infom, infom, row__,row__);
				if (orgBodyB) 
				{
					m_A.multiplyAdd2_p8r (JinvMrow + 8*(size_t)infom, Jrow + 8*(size_t)infom, infom, infom,  row__,row__);
				}
				row__ += infom;
				jj++;
			}
		}
	}

	if (1)
	{
		// add cfm to the diagonal of m_A
		for ( int i=0; i<m_A.rows(); ++i) 
		{
			m_A.setElem(i,i,m_A(i,i)+ m_cfm / infoGlobal.m_timeStep);
		}
	}
				   
	///fill the upper triangle of the matrix, to make it symmetric
	{
		CProfileSample sample = new CProfileSample("fill the upper triangle ");
		m_A.copyLowerToUpperTriangle();
	}

	{
		CProfileSample sample = new CProfileSample("resize/init x");
		m_x.resize(numConstraintRows);
		m_xSplit.resize(numConstraintRows);

		if (infoGlobal.m_solverMode&SOLVER_USE_WARMSTARTING)
		{
			for (int i=0;i<m_allConstraintPtrArray.Count;i++)
			{
				btSolverConstraint c = *m_allConstraintPtrArray[i];
				m_x[i]=c.m_appliedImpulse;
				m_xSplit[i] = c.m_appliedPushImpulse;
			}
		} else
		{
			m_x.setZero();
			m_xSplit.setZero();
		}
	}

}

void btMLCPSolver::createMLCP(btContactSolverInfo& infoGlobal)
{
	int numBodies = this.m_tmpSolverBodyPool.Count;
	int numConstraintRows = m_allConstraintPtrArray.Count;

	m_b.resize(numConstraintRows);
	if (infoGlobal.m_splitImpulse)
		m_bSplit.resize(numConstraintRows);
 
	m_bSplit.setZero();
	m_b.setZero();

	for (int i=0;i<numConstraintRows ;i++)
	{
		if (m_allConstraintPtrArray[i].m_jacDiagABInv)
		{
			m_b[i]=m_allConstraintPtrArray[i].m_rhs/m_allConstraintPtrArray[i].m_jacDiagABInv;
			if (infoGlobal.m_splitImpulse)
				m_bSplit[i] = m_allConstraintPtrArray[i].m_rhsPenetration/m_allConstraintPtrArray[i].m_jacDiagABInv;
		}
	}
 
	static btMatrixXu Minv;
	Minv.resize(6*numBodies,6*numBodies);
	Minv.setZero();
	for (int i=0;i<numBodies;i++)
	{
		btSolverBody rb = m_tmpSolverBodyPool[i];
		ref btVector3 invMass = rb.m_invMass;
		setElem(Minv,i*6+0,i*6+0,invMass[0]);
		setElem(Minv,i*6+1,i*6+1,invMass[1]);
		setElem(Minv,i*6+2,i*6+2,invMass[2]);
		btRigidBody orgBody = m_tmpSolverBodyPool[i].m_originalBody;
 
		for (int r=0;r<3;r++)
			for (int c=0;c<3;c++)
				setElem(Minv,i*6+3+r,i*6+3+c,orgBody? orgBody.getInvInertiaTensorWorld()[r][c] : 0);
	}
 
	static btMatrixXu J;
	J.resize(numConstraintRows,6*numBodies);
	J.setZero();
 
	m_lo.resize(numConstraintRows);
	m_hi.resize(numConstraintRows);
 
	for (int i=0;i<numConstraintRows;i++)
	{

		m_lo[i] = m_allConstraintPtrArray[i].m_lowerLimit;
		m_hi[i] = m_allConstraintPtrArray[i].m_upperLimit;
 
		int bodyIndex0 = m_allConstraintPtrArray[i].m_solverBodyIdA;
		int bodyIndex1 = m_allConstraintPtrArray[i].m_solverBodyIdB;
		if (m_tmpSolverBodyPool[bodyIndex0].m_originalBody)
		{
			setElem(J,i,6*bodyIndex0+0,m_allConstraintPtrArray[i].m_contactNormal1[0]);
			setElem(J,i,6*bodyIndex0+1,m_allConstraintPtrArray[i].m_contactNormal1[1]);
			setElem(J,i,6*bodyIndex0+2,m_allConstraintPtrArray[i].m_contactNormal1[2]);
			setElem(J,i,6*bodyIndex0+3,m_allConstraintPtrArray[i].m_relpos1CrossNormal[0]);
			setElem(J,i,6*bodyIndex0+4,m_allConstraintPtrArray[i].m_relpos1CrossNormal[1]);
			setElem(J,i,6*bodyIndex0+5,m_allConstraintPtrArray[i].m_relpos1CrossNormal[2]);
		}
		if (m_tmpSolverBodyPool[bodyIndex1].m_originalBody)
		{
			setElem(J,i,6*bodyIndex1+0,m_allConstraintPtrArray[i].m_contactNormal2[0]);
			setElem(J,i,6*bodyIndex1+1,m_allConstraintPtrArray[i].m_contactNormal2[1]);
			setElem(J,i,6*bodyIndex1+2,m_allConstraintPtrArray[i].m_contactNormal2[2]);
			setElem(J,i,6*bodyIndex1+3,m_allConstraintPtrArray[i].m_relpos2CrossNormal[0]);
			setElem(J,i,6*bodyIndex1+4,m_allConstraintPtrArray[i].m_relpos2CrossNormal[1]);
			setElem(J,i,6*bodyIndex1+5,m_allConstraintPtrArray[i].m_relpos2CrossNormal[2]);
		}
	}
 
	static btMatrixXu J_transpose;
	J_transpose= J.transpose();

	static btMatrixXu tmp;

	{
		{
			CProfileSample sample = new CProfileSample("J*Minv");
			tmp = J*Minv;

		}
		{
			CProfileSample sample = new CProfileSample("J*tmp");
			m_A = tmp*J_transpose;
		}
	}

	if (1)
	{
		// add cfm to the diagonal of m_A
		for ( int i=0; i<m_A.rows(); ++i) 
		{
			m_A.setElem(i,i,m_A(i,i)+ m_cfm / infoGlobal.m_timeStep);
		}
	}

	m_x.resize(numConstraintRows);
	if (infoGlobal.m_splitImpulse)
		m_xSplit.resize(numConstraintRows);
//	m_x.setZero();

	for (int i=0;i<m_allConstraintPtrArray.Count;i++)
	{
		btSolverConstraint c = *m_allConstraintPtrArray[i];
		m_x[i]=c.m_appliedImpulse;
		if (infoGlobal.m_splitImpulse)
			m_xSplit[i] = c.m_appliedPushImpulse;
	}

}


double btMLCPSolver::solveGroupCacheFriendlyIterations(btCollisionObject bodies ,int numBodies,btPersistentManifold[] manifoldPtr, int numManifolds,btTypedConstraint[] constraints,int numConstraints,btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer)
{
	bool result = true;
	{
		CProfileSample sample = new CProfileSample("solveMLCP");
//		Console.WriteLine("m_A(%d,%d)\n", m_A.rows(),m_A.cols());
		result = solveMLCP(infoGlobal);
	}

	//check if solution is valid, and otherwise fallback to btSequentialImpulseConstraintSolver::solveGroupCacheFriendlyIterations
	if (result)
	{
		CProfileSample sample = new CProfileSample("process MLCP results");
		for (int i=0;i<m_allConstraintPtrArray.Count;i++)
		{
			{
				btSolverConstraint c = *m_allConstraintPtrArray[i];
				int sbA = c.m_solverBodyIdA;
				int sbB = c.m_solverBodyIdB;
				//btRigidBody orgBodyA = m_tmpSolverBodyPool[sbA].m_originalBody;
			//	btRigidBody orgBodyB = m_tmpSolverBodyPool[sbB].m_originalBody;

				btSolverBody solverBodyA = m_tmpSolverBodyPool[sbA];
				btSolverBody solverBodyB = m_tmpSolverBodyPool[sbB];
 
				{
					double deltaImpulse = m_x[i]-c.m_appliedImpulse;
					c.m_appliedImpulse = m_x[i];
					solverBodyA.internalApplyImpulse(c.m_contactNormal1*solverBodyA.internalGetInvMass(),c.m_angularComponentA,deltaImpulse);
					solverBodyB.internalApplyImpulse(c.m_contactNormal2*solverBodyB.internalGetInvMass(),c.m_angularComponentB,deltaImpulse);
				}

				if (infoGlobal.m_splitImpulse)
				{
					double deltaImpulse = m_xSplit[i] - c.m_appliedPushImpulse;
					solverBodyA.internalApplyPushImpulse(c.m_contactNormal1*solverBodyA.internalGetInvMass(),c.m_angularComponentA,deltaImpulse);
					solverBodyB.internalApplyPushImpulse(c.m_contactNormal2*solverBodyB.internalGetInvMass(),c.m_angularComponentB,deltaImpulse);
					c.m_appliedPushImpulse = m_xSplit[i];
				}
				
			}
		}
	}
	else
	{
	//	Console.WriteLine("m_fallback = %d\n",m_fallback);
		m_fallback++;
		btSequentialImpulseConstraintSolver::solveGroupCacheFriendlyIterations(bodies ,numBodies,manifoldPtr, numManifolds,constraints,numConstraints,infoGlobal,debugDrawer);
	}

	return 0;
}


