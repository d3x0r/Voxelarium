using BEPUphysics.Constraints.TwoEntity;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Entities;
using BEPUutilities;

namespace BEPUphysics.Constraints.SolverGroups
{
    /// <summary>
    /// Restricts the linear and angular motion between two entities.
    /// </summary>
    public class WeldJoint : SolverGroup
    {
        /// <summary>
        /// Constructs a new constraint which restricts the linear and angular motion between two entities.
        /// This constructs the internal constraints, but does not configure them.  Before using a constraint constructed in this manner,
        /// ensure that its active constituent constraints are properly configured.  The entire group as well as all internal constraints are initially inactive (IsActive = false).
        /// </summary>
        public WeldJoint()
        {
            IsActive = false;
            BallSocketJoint = new BallSocketJoint();
            NoRotationJoint = new NoRotationJoint();
            Add(BallSocketJoint);
            Add(NoRotationJoint);
        }

        private static Vector3 GetAnchorGuess(Entity connectionA, Entity connectionB)
        {
            var anchor = new Vector3();
            if (connectionA != null)
                anchor.Add( ref connectionA.position, out anchor );
            if (connectionB != null)
                anchor.Add( ref connectionB.position, out anchor );
            if (connectionA != null && connectionB != null)
                anchor.Mult( 0.5f, out anchor );
            return anchor;
        }

		private static void GetAnchorGuess( Entity connectionA, Entity connectionB, out Vector3 anchor )
		{
			if( connectionA != null && connectionB != null )
			{
				connectionA.position.Add( ref connectionA.position, out anchor );
				anchor.Mult( 0.5f, out anchor );
			}
			else if( connectionA != null )
				anchor = connectionA.position;
			else if( connectionB != null )
				anchor = connectionB.position;
			anchor = Vector3.Zero;
		}

		/// <summary>
		/// Constructs a new constraint which restricts the linear and angular motion between two entities.
		/// Uses the average of the two entity positions for the anchor.
		/// </summary>
		/// <param name="connectionA">First entity of the constraint pair.</param>
		/// <param name="connectionB">Second entity of the constraint pair.</param>
		public WeldJoint(Entity connectionA, Entity connectionB)
        {
			if( connectionA == null )
				connectionA = TwoEntityConstraint.WorldEntity;
			if( connectionB == null )
				connectionB = TwoEntityConstraint.WorldEntity;
			Vector3 anchor;  GetAnchorGuess( connectionA, connectionB, out anchor );
            BallSocketJoint = new BallSocketJoint( connectionA, connectionB, ref anchor );
			NoRotationJoint = new NoRotationJoint( connectionA, connectionB );
			Add( BallSocketJoint );
			Add( NoRotationJoint );
		}

		/// <summary>
		/// Constructs a new constraint which restricts the linear and angular motion between two entities.
		/// </summary>
		/// <param name="connectionA">First entity of the constraint pair.</param>
		/// <param name="connectionB">Second entity of the constraint pair.</param>
		/// <param name="anchor">The location of the weld.</param>
		public WeldJoint(Entity connectionA, Entity connectionB, ref Vector3 anchor)
        {
            if (connectionA == null)
                connectionA = TwoEntityConstraint.WorldEntity;
            if (connectionB == null)
                connectionB = TwoEntityConstraint.WorldEntity;
            BallSocketJoint = new BallSocketJoint(connectionA, connectionB, ref anchor);
            NoRotationJoint = new NoRotationJoint(connectionA, connectionB);
            Add(BallSocketJoint);
            Add(NoRotationJoint);
        }

        /// <summary>
        /// Gets the ball socket joint that restricts linear degrees of freedom.
        /// </summary>
        public BallSocketJoint BallSocketJoint { get; private set; }

        /// <summary>
        /// Gets the no rotation joint that prevents angular motion.
        /// </summary>
        public NoRotationJoint NoRotationJoint { get; private set; }

        
    }
}