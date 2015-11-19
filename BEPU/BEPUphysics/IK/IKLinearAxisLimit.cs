using BEPUutilities;

namespace BEPUik
{
    /// <summary>
    /// Keeps an anchor point on one bone between planes defined by another bone.
    /// </summary>
    public class IKLinearAxisLimit : IKLimit
    {
        /// <summary>
        /// Gets or sets the offset in connection A's local space from the center of mass to the anchor point of the line.
        /// </summary>
        public Vector3 LocalLineAnchor;

        /// <summary>
        /// Gets or sets the direction of the line in connection A's local space.
        /// Must be unit length.
        /// </summary>
        public Vector3 LocalLineDirection;

        /// <summary>
        /// Gets or sets the offset in connection B's local space from the center of mass to the anchor point which will be kept on the line.
        /// </summary>
        public Vector3 LocalAnchorB;

        /// <summary>
        /// Gets the world space location of the line anchor attached to connection A.
        /// </summary>
        public void GetLineAnchor(out Vector3 r)
        {
            Quaternion.Transform(ref LocalLineAnchor, ref ConnectionA.Orientation, out r);
            ConnectionA.Position.Add(ref r, out r);
        }
        /// <summary>
        /// Sets the world space location of the line anchor attached to connection A.
        /// </summary>
        public void SetLineAnchor(ref Vector3 value)
        {
            Vector3 tmp;
            value.Sub(ref ConnectionA.Position, out tmp);
            Quaternion.Transform(ref tmp, ref ConnectionA.ConjOrientation, out LocalLineAnchor);
        }

        /// <summary>
        /// Gets or sets the world space direction of the line attached to connection A.
        /// Must be unit length.
        /// </summary>
        public Vector3 LineDirection
        {
            get { return Quaternion.Transform(LocalLineDirection, ConnectionA.Orientation); }
            set { LocalLineDirection = Quaternion.Transform(value, Quaternion.Conjugate(ConnectionA.Orientation)); }
        }

        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection B to the anchor point.
        /// </summary>
        public void GetAnchorB(out Vector3 r)
        {
            Quaternion.Transform(ref LocalAnchorB, ref ConnectionB.Orientation, out r);
            ConnectionB.Position.Add(ref r, out r);
        }
        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection B to the anchor point.
        /// </summary>
        public void SetAnchorB(ref Vector3 value)
        {
            Vector3 tmp;
            value.Sub(ref ConnectionB.Position, out tmp);
            Quaternion.Transform(ref tmp, ref ConnectionB.ConjOrientation, out LocalAnchorB);
        }

        private float minimumDistance;
        /// <summary>
        /// Gets or sets the minimum distance that the joint connections should be kept from each other.
        /// </summary>
        public float MinimumDistance
        {
            get { return minimumDistance; }
            set { minimumDistance = value; }
        }

        private float maximumDistance;
        /// <summary>
        /// Gets or sets the maximum distance that the joint connections should be kept from each other.
        /// </summary>
        public float MaximumDistance
        {
            get { return maximumDistance; }
            set { maximumDistance = value; }
        }

        /// <summary>
        /// Constructs a new axis limit.
        /// </summary>
        /// <param name="connectionA">First bone connected by the joint.</param>
        /// <param name="connectionB">Second bone connected by the joint.</param>
        /// <param name="lineAnchor">Anchor point of the line attached to the first bone in world space.</param>
        /// <param name="lineDirection">Direction of the line attached to the first bone in world space. Must be unit length.</param>
        /// <param name="anchorB">Anchor point on the second bone in world space which is measured against the other connection's anchor.</param>
        /// <param name="minimumDistance">Minimum distance that the joint connections should be kept from each other along the axis.</param>
        /// <param name="maximumDistance">Maximum distance that the joint connections should be kept from each other along the axis.</param>
        public IKLinearAxisLimit(Bone connectionA, Bone connectionB, ref Vector3 lineAnchor, ref Vector3 lineDirection, ref Vector3 anchorB, float minimumDistance, float maximumDistance)
            : base(connectionA, connectionB)
        {
            SetLineAnchor(ref lineAnchor);
            LineDirection = lineDirection;
            SetAnchorB(ref anchorB);
            MinimumDistance = minimumDistance;
            MaximumDistance = maximumDistance;
        }

        protected internal override void UpdateJacobiansAndVelocityBias()
        {
            //Transform the anchors and offsets into world space.
            Vector3 offsetA, offsetB, lineDirection;
            Quaternion.Transform(ref LocalLineAnchor, ref ConnectionA.Orientation, out offsetA);
            Quaternion.Transform(ref LocalLineDirection, ref ConnectionA.Orientation, out lineDirection);
            Quaternion.Transform(ref LocalAnchorB, ref ConnectionB.Orientation, out offsetB);
            Vector3 anchorA, anchorB;
            Vector3.Add(ref ConnectionA.Position, ref offsetA, out anchorA);
            Vector3.Add(ref ConnectionB.Position, ref offsetB, out anchorB);

            //Compute the distance.
            Vector3 separation;
            Vector3.Subtract(ref anchorB, ref anchorA, out separation);
            //This entire constraint is very similar to the IKDistanceLimit, except the current distance is along an axis.
            float currentDistance;
            Vector3.Dot(ref separation, ref lineDirection, out currentDistance);

            //Compute jacobians
            if (currentDistance > maximumDistance)
            {
                //We are exceeding the maximum limit.
                velocityBias = new Vector3(errorCorrectionFactor * (currentDistance - maximumDistance), 0, 0);
            }
            else if (currentDistance < minimumDistance)
            {
                //We are exceeding the minimum limit.
                velocityBias = new Vector3(errorCorrectionFactor * (minimumDistance - currentDistance), 0, 0);
                //The limit can only push in one direction. Flip the jacobian!
                Vector3.Negate(ref lineDirection, out lineDirection);
            }
            else if (currentDistance - minimumDistance > (maximumDistance - minimumDistance) * 0.5f)
            {
                //The objects are closer to hitting the maximum limit.
                velocityBias = new Vector3(currentDistance - maximumDistance, 0, 0);
            }
            else
            {
                //The objects are closer to hitting the minimum limit.
                velocityBias = new Vector3(minimumDistance - currentDistance, 0, 0);
                //The limit can only push in one direction. Flip the jacobian!
                Vector3.Negate(ref lineDirection, out lineDirection);
            }

            Vector3 angularA, angularB;
            //We can't just use the offset to anchor for A's jacobian- the 'collision' location is way out there at anchorB!
            Vector3 rA;
            Vector3.Subtract(ref anchorB, ref ConnectionA.Position, out rA);
            Vector3.Cross(ref rA, ref lineDirection, out angularA);
            //linearB = -linearA, so just swap the cross product order.
            Vector3.Cross(ref lineDirection, ref offsetB, out angularB);

            //Put all the 1x3 jacobians into a 3x3 matrix representation.
            linearJacobianA.M1 = lineDirection;
            lineDirection.Invert(out linearJacobianB.M1);
            angularJacobianA.M1 = angularA;
            angularJacobianB.M1 = angularB;

        }
    }
}
