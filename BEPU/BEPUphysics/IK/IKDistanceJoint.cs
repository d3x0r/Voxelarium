using System;
using BEPUutilities;

namespace BEPUik
{
    /// <summary>
    /// Keeps the anchor points on two bones at the same distance.
    /// </summary>
    public class IKDistanceJoint : IKJoint
    {
        /// <summary>
        /// Gets or sets the offset in connection A's local space from the center of mass to the anchor point.
        /// </summary>
        public Vector3 LocalAnchorA;
        /// <summary>
        /// Gets or sets the offset in connection B's local space from the center of mass to the anchor point.
        /// </summary>
        public Vector3 LocalAnchorB;

        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection A to the anchor point.
        /// </summary>
        public void GetAnchorA(out Vector3 r)
        {
            Quaternion.Transform(ref LocalAnchorA, ref ConnectionA.Orientation, out r);
            ConnectionA.Position.Add(ref r, out r);
        }

        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection A to the anchor point.
        /// </summary>
        public void GetAnchorB(out Vector3 r)
        {
            Quaternion.Transform(ref LocalAnchorB, ref ConnectionB.Orientation, out r);
            ConnectionB.Position.Add(ref r, out r);
        }
        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection A to the anchor point.
        /// </summary>
        public void SetAnchorA(ref Vector3 value)
        {
            Vector3 tmp;
            value.Sub(ref ConnectionA.Position, out tmp);
            Quaternion.Transform(ref tmp, ref ConnectionA.ConjOrientation, out LocalAnchorA);
        }

        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection A to the anchor point.
        /// </summary>
        public void SetAnchorB(ref Vector3 value)
        {
            Vector3 tmp;
            value.Sub(ref ConnectionB.Position, out tmp);
            Quaternion.Transform(ref tmp, ref ConnectionB.ConjOrientation, out LocalAnchorB);
        }

        private float distance;
        /// <summary>
        /// Gets or sets the distance that the joint connections should be kept from each other.
        /// </summary>
        public float Distance
        {
            get { return distance; }
            set { distance = Math.Max(0, value); }
        }

        /// <summary>
        /// Constructs a new distance joint.
        /// </summary>
        /// <param name="connectionA">First bone connected by the joint.</param>
        /// <param name="connectionB">Second bone connected by the joint.</param>
        /// <param name="anchorA">Anchor point on the first bone in world space.</param>
        /// <param name="anchorB">Anchor point on the second bone in world space.</param>
        public IKDistanceJoint(Bone connectionA, Bone connectionB, ref Vector3 anchorA, ref Vector3 anchorB)
            : base(connectionA, connectionB)
        {
            SetAnchorA(ref anchorA);
            SetAnchorB(ref anchorB);
            Vector3.Distance(ref anchorA, ref anchorB, out distance);
        }

        protected internal override void UpdateJacobiansAndVelocityBias()
        {
            //Transform the anchors and offsets into world space.
            Vector3 offsetA, offsetB;
            Quaternion.Transform(ref LocalAnchorA, ref ConnectionA.Orientation, out offsetA);
            Quaternion.Transform(ref LocalAnchorB, ref ConnectionB.Orientation, out offsetB);
            Vector3 anchorA, anchorB;
            Vector3.Add(ref ConnectionA.Position, ref offsetA, out anchorA);
            Vector3.Add(ref ConnectionB.Position, ref offsetB, out anchorB);

            //Compute the distance.
            Vector3 separation;
            Vector3.Subtract(ref anchorB, ref anchorA, out separation);
            float currentDistance = separation.Length();

            //Compute jacobians
            Vector3 linearA;
#if !WINDOWS
            linearA = new Vector3();
#endif
            if (currentDistance > Toolbox.Epsilon)
            {
                linearA.X = separation.X / currentDistance;
                linearA.Y = separation.Y / currentDistance;
                linearA.Z = separation.Z / currentDistance;

                velocityBias = new Vector3(errorCorrectionFactor * (currentDistance - distance), 0, 0);
            }
            else
            {
                velocityBias = new Vector3();
                linearA = new Vector3();
            }

            Vector3 angularA, angularB;
            Vector3.Cross(ref offsetA, ref linearA, out angularA);
            //linearB = -linearA, so just swap the cross product order.
            Vector3.Cross(ref linearA, ref offsetB, out angularB);

            //Put all the 1x3 jacobians into a 3x3 matrix representation.
            linearJacobianA.M1 = linearA;
            linearA.Invert(out linearJacobianB.M1);
            angularJacobianA.M1 = angularA;
            angularJacobianB.M1 = angularB;

        }
    }
}
