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
    /// Acts like a tablet pc monitor hinge.
    /// </summary>
    public class SwivelHingeJoint : SolverGroup
    {
        /// <summary>
        /// Constructs a new constraint which restricts three degrees of linear freedom and one degree of angular freedom between two entities.
        /// This constructs the internal constraints, but does not configure them.  Before using a constraint constructed in this manner,
        /// ensure that its active constituent constraints are properly configured.  The entire group as well as all internal constraints are initially inactive (IsActive = false).
        /// </summary>
        public SwivelHingeJoint()
        {
            IsActive = false;
            BallSocketJoint = new BallSocketJoint();
            AngularJoint = new SwivelHingeAngularJoint();
            HingeLimit = new RevoluteLimit();
            HingeMotor = new RevoluteMotor();
            TwistLimit = new TwistLimit();
            TwistMotor = new TwistMotor();

            Add(BallSocketJoint);
            Add(AngularJoint);
            Add(HingeLimit);
            Add(HingeMotor);
            Add(TwistLimit);
            Add(TwistMotor);
        }

        /// <summary>
        /// Constructs a new constraint which restricts three degrees of linear freedom and one degree of angular freedom between two entities.
        /// </summary>
        /// <param name="connectionA">First entity of the constraint pair.</param>
        /// <param name="connectionB">Second entity of the constraint pair.</param>
        /// <param name="anchor">Point around which both entities rotate.</param>
        /// <param name="hingeAxis">Axis of allowed rotation in world space to be attached to connectionA.  Will be kept perpendicular with the twist axis.</param>
        public SwivelHingeJoint(Entity connectionA, Entity connectionB, ref Vector3 anchor, ref Vector3 hingeAxis)
        {
            if (connectionA == null)
                connectionA = TwoEntityConstraint.WorldEntity;
            if (connectionB == null)
                connectionB = TwoEntityConstraint.WorldEntity;
            BallSocketJoint = new BallSocketJoint(connectionA, connectionB, ref anchor);
			Vector3 tmp;
			BallSocketJoint.OffsetB.Invert( out tmp );
            AngularJoint = new SwivelHingeAngularJoint(connectionA, connectionB, ref hingeAxis, ref tmp );
            HingeLimit = new RevoluteLimit(connectionA, connectionB);
            HingeMotor = new RevoluteMotor(connectionA, connectionB, hingeAxis);
            TwistLimit = new TwistLimit(connectionA, connectionB, ref BallSocketJoint.worldOffsetA, ref tmp, 0, 0);
            TwistMotor = new TwistMotor(connectionA, connectionB, ref BallSocketJoint.worldOffsetA, ref tmp );
            HingeLimit.IsActive = false;
            HingeMotor.IsActive = false;
            TwistLimit.IsActive = false;
            TwistMotor.IsActive = false;

			//Ensure that the base and test direction is perpendicular to the free axis.
			Vector3 baseAxis; anchor.Sub( ref connectionA.position, out baseAxis );
            if (baseAxis.LengthSquared() < Toolbox.BigEpsilon) //anchor and connection a in same spot, so try the other way.
                connectionB.position.Sub( ref anchor, out baseAxis );
            baseAxis.AddScaled( ref hingeAxis, -Vector3.Dot( ref baseAxis, ref hingeAxis) , out baseAxis );
            if (baseAxis.LengthSquared() < Toolbox.BigEpsilon)
            {
                //However, if the free axis is totally aligned (like in an axis constraint), pick another reasonable direction.
                Vector3.Cross(ref hingeAxis, ref Vector3.Up, out baseAxis);
                if (baseAxis.LengthSquared() < Toolbox.BigEpsilon)
                {
                    Vector3.Cross(ref hingeAxis, ref Vector3.Right, out baseAxis);
                }
            }
            HingeLimit.Basis.SetWorldAxes(ref hingeAxis, ref baseAxis, ref connectionA.orientationMatrix);
            HingeMotor.Basis.SetWorldAxes( ref hingeAxis, ref baseAxis, ref connectionA.orientationMatrix);

			connectionB.position.Sub( ref anchor, out baseAxis );
            baseAxis.AddScaled( ref hingeAxis, -Vector3.Dot(ref baseAxis, ref hingeAxis), out baseAxis );
            if (baseAxis.LengthSquared() < Toolbox.BigEpsilon)
            {
                //However, if the free axis is totally aligned (like in an axis constraint), pick another reasonable direction.
                Vector3.Cross(ref hingeAxis, ref Vector3.Up, out baseAxis);
                if (baseAxis.LengthSquared() < Toolbox.BigEpsilon)
                {
                    Vector3.Cross(ref hingeAxis, ref Vector3.Right, out baseAxis);
                }
            }
            HingeLimit.TestAxis = baseAxis;
            HingeMotor.TestAxis = baseAxis;


            Add(BallSocketJoint);
            Add(AngularJoint);
            Add(HingeLimit);
            Add(HingeMotor);
            Add(TwistLimit);
            Add(TwistMotor);
        }

#if ALLOW_LAZY_VECTORS
		public SwivelHingeJoint( Entity connectionA, Entity connectionB, Vector3 anchor, Vector3 hingeAxis )
			: this( connectionA, connectionB, ref anchor, ref hingeAxis )
		{
		}
#endif

		/// <summary>
		/// Gets the angular joint which removes one degree of freedom.
		/// </summary>
		public SwivelHingeAngularJoint AngularJoint { get; private set; }

        /// <summary>
        /// Gets the ball socket joint that restricts linear degrees of freedom.
        /// </summary>
        public BallSocketJoint BallSocketJoint { get; private set; }

        /// <summary>
        /// Gets the rotational limit of the hinge.
        /// </summary>
        public RevoluteLimit HingeLimit { get; private set; }

        /// <summary>
        /// Gets the motor of the hinge.
        /// </summary>
        public RevoluteMotor HingeMotor { get; private set; }

        /// <summary>
        /// Gets the rotational limit of the swivel hinge.
        /// </summary>
        public TwistLimit TwistLimit { get; private set; }

        /// <summary>
        /// Gets the twist motor of the swivel hinge.
        /// </summary>
        public TwistMotor TwistMotor { get; private set; }
    }
}