using BEPUphysics.Constraints.TwoEntity;
using BEPUphysics.Constraints.TwoEntity.JointLimits;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities;
using BEPUutilities;
 

namespace BEPUphysics.Constraints.SolverGroups
{
    /// <summary>
    /// Restricts three degrees of linear motion and one degree of angular motion.
    /// Acts like two hinges in immediate sequence.
    /// </summary>
    public class UniversalJoint : SolverGroup
    {
        /// <summary>
        /// Constructs a new constraint which restricts three degrees of linear freedom and one degree of twisting angular freedom between two entities.
        /// This constructs the internal constraints, but does not configure them.  Before using a constraint constructed in this manner,
        /// ensure that its active constituent constraints are properly configured.  The entire group as well as all internal constraints are initially inactive (IsActive = false).
        /// </summary>
        public UniversalJoint()
        {
            IsActive = false;
            BallSocketJoint = new BallSocketJoint();
            TwistJoint = new TwistJoint();
            Limit = new TwistLimit();
            Motor = new TwistMotor();
            Add(BallSocketJoint);
            Add(TwistJoint);
            Add(Limit);
            Add(Motor);
        }


        /// <summary>
        /// Constructs a new constraint which restricts three degrees of linear freedom and one degree of twisting angular freedom between two entities.
        /// </summary>
        /// <param name="connectionA">First entity of the constraint pair.</param>
        /// <param name="connectionB">Second entity of the constraint pair.</param>
        /// <param name="anchor">Point around which both entities rotate in world space.</param>
        public UniversalJoint(Entity connectionA, Entity connectionB, ref Vector3 anchor)
        {
            if (connectionA == null)
                connectionA = TwoEntityConstraint.WorldEntity;
            if (connectionB == null)
                connectionB = TwoEntityConstraint.WorldEntity;
            BallSocketJoint = new BallSocketJoint(connectionA, connectionB, ref anchor);
			Vector3 tmp; BallSocketJoint.OffsetB.Invert( out tmp );
            TwistJoint = new TwistJoint(connectionA, connectionB, ref BallSocketJoint.worldOffsetA, ref tmp);
            Limit = new TwistLimit(connectionA, connectionB, ref BallSocketJoint.worldOffsetA, ref tmp, 0, 0);
            Motor = new TwistMotor(connectionA, connectionB, ref BallSocketJoint.worldOffsetA, ref tmp );
            Limit.IsActive = false;
            Motor.IsActive = false;
            Add(BallSocketJoint);
            Add(TwistJoint);
            Add(Limit);
            Add(Motor);
        }
#if ALLOW_LAZY_VECTORS
		public UniversalJoint( Entity a, Entity b, Vector3 anchor ) : this( a, b, ref anchor ) { }
#endif
		/// <summary>
		/// Gets the ball socket joint that restricts linear degrees of freedom.
		/// </summary>
		public BallSocketJoint BallSocketJoint { get; private set; }

        /// <summary>
        /// Gets the rotational limit of the universal joint.
        /// This constraint overlaps with the twistJoint; if the limit is activated,
        /// the twistJoint should be generally deactivated and vice versa.
        /// </summary>
        public TwistLimit Limit { get; private set; }

        /// <summary>
        /// Gets the motor of the universal joint.
        /// This constraint overlaps with the twistJoint; if the motor is activated,
        /// the twistJoint should generally be deactivated and vice versa.
        /// </summary>
        public TwistMotor Motor { get; private set; }

        /// <summary>
        /// Gets the angular joint which removes one twisting degree of freedom.
        /// </summary>
        public TwistJoint TwistJoint { get; private set; }
    }
}