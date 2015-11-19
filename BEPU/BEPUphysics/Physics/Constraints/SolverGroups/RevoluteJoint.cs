using BEPUphysics.Constraints.TwoEntity;
using BEPUphysics.Constraints.TwoEntity.JointLimits;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Constraints.TwoEntity.Motors;
using BEPUphysics.Entities;
using BEPUutilities;
 

namespace BEPUphysics.Constraints.SolverGroups
{
    /// <summary>
    /// Restricts linear motion while allowing one degree of angular freedom.
    /// Acts like a normal door hinge.
    /// </summary>
    public class RevoluteJoint : SolverGroup
    {

        /// <summary>
        /// Constructs a new constraint which restricts three degrees of linear freedom and two degrees of angular freedom between two entities.
        /// This constructs the internal constraints, but does not configure them.  Before using a constraint constructed in this manner,
        /// ensure that its active constituent constraints are properly configured.  The entire group as well as all internal constraints are initially inactive (IsActive = false).
        /// </summary>
        public RevoluteJoint()
        {
            IsActive = false;
            BallSocketJoint = new BallSocketJoint();
            AngularJoint = new RevoluteAngularJoint();
            Limit = new RevoluteLimit();
            Motor = new RevoluteMotor();


            Add(BallSocketJoint);
            Add(AngularJoint);
            Add(Limit);
            Add(Motor);
        }


        /// <summary>
        /// Constructs a new constraint which restricts three degrees of linear freedom and two degrees of angular freedom between two entities.
        /// </summary>
        /// <param name="connectionA">First entity of the constraint pair.</param>
        /// <param name="connectionB">Second entity of the constraint pair.</param>
        /// <param name="anchor">Point around which both entities rotate.</param>
        /// <param name="freeAxis">Axis around which the hinge can rotate.</param>
        public RevoluteJoint(Entity connectionA, Entity connectionB, ref Vector3 anchor, ref Vector3 freeAxis)
        {
            if (connectionA == null)
                connectionA = TwoEntityConstraint.WorldEntity;
            if (connectionB == null)
                connectionB = TwoEntityConstraint.WorldEntity;
            BallSocketJoint = new BallSocketJoint(connectionA, connectionB, ref anchor);
            AngularJoint = new RevoluteAngularJoint(connectionA, connectionB, freeAxis);
            Limit = new RevoluteLimit(connectionA, connectionB);
            Motor = new RevoluteMotor(connectionA, connectionB, freeAxis);
            Limit.IsActive = false;
            Motor.IsActive = false;

			//Ensure that the base and test direction is perpendicular to the free axis.
			Vector3 baseAxis; anchor.Sub( ref connectionA.position, out baseAxis );
            if (baseAxis.LengthSquared() < Toolbox.BigEpsilon) //anchor and connection a in same spot, so try the other way.
                connectionB.position.Sub( ref anchor, out baseAxis );
            baseAxis.AddScaled( ref freeAxis, - Vector3.Dot( ref baseAxis, ref freeAxis ), out baseAxis );
            if (baseAxis.LengthSquared() < Toolbox.BigEpsilon)
            {
                //However, if the free axis is totally aligned (like in an axis constraint), pick another reasonable direction.
                Vector3.Cross( ref freeAxis, ref Vector3.Up, out baseAxis);
                if (baseAxis.LengthSquared() < Toolbox.BigEpsilon)
                {
                    Vector3.Cross( ref freeAxis, ref Vector3.Right, out baseAxis );
                }
            }
            Limit.Basis.SetWorldAxes(ref freeAxis, ref baseAxis, ref connectionA.orientationMatrix);
            Motor.Basis.SetWorldAxes( ref freeAxis, ref baseAxis, ref connectionA.orientationMatrix);

            connectionB.position.Sub( ref anchor, out baseAxis );
            baseAxis.AddScaled( ref freeAxis,  -Vector3.Dot( ref baseAxis, ref freeAxis ), out baseAxis );
            if (baseAxis.LengthSquared() < Toolbox.BigEpsilon)
            {
                //However, if the free axis is totally aligned (like in an axis constraint), pick another reasonable direction.
                Vector3.Cross( ref freeAxis, ref Vector3.Up, out baseAxis);
                if (baseAxis.LengthSquared() < Toolbox.BigEpsilon)
                {
                    Vector3.Cross( ref freeAxis, ref Vector3.Right, out baseAxis);
                }
            }
            Limit.TestAxis = baseAxis;
            Motor.TestAxis = baseAxis;


            Add(BallSocketJoint);
            Add(AngularJoint);
            Add(Limit);
            Add(Motor);
        }
#if ALLOW_LAZY_VECTORS
		public RevoluteJoint( Entity connectionA, Entity connectionB, Vector3 anchor, Vector3 freeAxis )
			: this( connectionA, connectionB, ref anchor, ref freeAxis )
		{
		}
#endif
		/// <summary>
		/// Gets the angular joint which removes two degrees of freedom.
		/// </summary>
		public RevoluteAngularJoint AngularJoint { get; private set; }

        /// <summary>
        /// Gets the ball socket joint that restricts linear degrees of freedom.
        /// </summary>
        public BallSocketJoint BallSocketJoint { get; private set; }

        /// <summary>
        /// Gets the rotational limit of the hinge.
        /// </summary>
        public RevoluteLimit Limit { get; private set; }

        /// <summary>
        /// Gets the motor of the hinge.
        /// </summary>
        public RevoluteMotor Motor { get; private set; }
    }
}