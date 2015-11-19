using System;
using BEPUutilities;

namespace BEPUik
{
    /// <summary>
    /// Keeps the anchor points on two bones at the same distance.
    /// </summary>
    public class IKPointOnLineJoint : IKJoint
    {
        /// <summary>
        /// Gets or sets the offset in connection A's local space from the center of mass to the anchor point of the line.
        /// </summary>
        public Vector3 LocalLineAnchor;

        private Vector3 localLineDirection;
        /// <summary>
        /// Gets or sets the direction of the line in connection A's local space.
        /// Must be unit length.
        /// </summary>
        public Vector3 LocalLineDirection
        {
            get { return localLineDirection; }
            set
            {
                localLineDirection = value;
                ComputeRestrictedAxes();
            }
        }


        /// <summary>
        /// Gets or sets the offset in connection B's local space from the center of mass to the anchor point which will be kept on the line.
        /// </summary>
        public Vector3 LocalAnchorB;

        /// <summary>
        /// Gets or sets the world space location of the line anchor attached to connection A.
        /// </summary>
        public Vector3 LineAnchor
        {
            //get { return ConnectionA.Position + Quaternion.Transform(LocalLineAnchor, ConnectionA.Orientation); }
            set
            {
                Vector3 tmp; value.Sub(ref ConnectionA.Position, out tmp);
                Quaternion.Transform(ref tmp, ref ConnectionA.ConjOrientation, out LocalLineAnchor);
            }
        }
        /// <summary>
        /// Gets the world space location of the line anchor attached to connection A.
        /// </summary>
        public void GetLineAnchor(out Vector3 result)
        {
            Quaternion.Transform(ref LocalLineAnchor, ref ConnectionA.Orientation, out result);
            ConnectionA.Position.Add(ref result, out result);
        }

        /// <summary>
        /// Gets or sets the world space direction of the line attached to connection A.
        /// Must be unit length.
        /// </summary>
        public Vector3 LineDirection
        {
            get { return Quaternion.Transform(localLineDirection, ConnectionA.Orientation); }
            set { LocalLineDirection = Quaternion.Transform(value, Quaternion.Conjugate(ConnectionA.Orientation)); }
        }

        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection B to the anchor point.
        /// </summary>
        public Vector3 AnchorB
        {
            //get { return ConnectionB.Position + Quaternion.Transform(LocalAnchorB, ConnectionB.Orientation); }
            set
            {
                Vector3 tmp; value.Sub(ref ConnectionB.Position, out tmp);
                Quaternion.Transform(ref tmp, ref ConnectionB.ConjOrientation, out LocalAnchorB);
            }
        }

        /// <summary>
        /// Gets the offset in world space from the center of mass of connection B to the anchor point.
        /// </summary>
        public void GetAnchorB(out Vector3 result)
        {
            Quaternion.Transform(ref LocalAnchorB, ref ConnectionB.Orientation, out result);
            ConnectionB.Position.Add(ref result, out result);
        }

        private Vector3 localRestrictedAxis1, localRestrictedAxis2;
        void ComputeRestrictedAxes()
        {
            Vector3 cross;
            Vector3.Cross(ref localLineDirection, ref Toolbox.UpVector, out cross);
            float lengthSquared = cross.LengthSquared();
            if (lengthSquared > Toolbox.Epsilon)
            {
                Vector3.Divide(ref cross, (float)Math.Sqrt(lengthSquared), out localRestrictedAxis1);
            }
            else
            {
                //Oops! The direction is aligned with the up vector.
                Vector3.Cross(ref localLineDirection, ref Toolbox.RightVector, out cross);
                Vector3.Normalize(ref cross, out localRestrictedAxis1);
            }
            //Don't need to normalize this; cross product of two unit length perpendicular vectors.
            Vector3.Cross(ref localRestrictedAxis1, ref localLineDirection, out localRestrictedAxis2);
        }

        /// <summary>
        /// Constructs a new point on line joint.
        /// </summary>
        /// <param name="connectionA">First bone connected by the joint.</param>
        /// <param name="connectionB">Second bone connected by the joint.</param>
        /// <param name="lineAnchor">Anchor point of the line attached to the first bone in world space.</param>
        /// <param name="lineDirection">Direction of the line attached to the first bone in world space. Must be unit length.</param>
        /// <param name="anchorB">Anchor point on the second bone in world space which tries to stay on connection A's line.</param>
        public IKPointOnLineJoint(Bone connectionA, Bone connectionB, Vector3 lineAnchor, Vector3 lineDirection, Vector3 anchorB)
            : base(connectionA, connectionB)
        {
            LineAnchor = lineAnchor;
            LineDirection = lineDirection;
            AnchorB = anchorB;

        }

        protected internal override void UpdateJacobiansAndVelocityBias()
        {

            //Transform local stuff into world space
            Vector3 worldRestrictedAxis1, worldRestrictedAxis2;
            Quaternion.Transform(ref localRestrictedAxis1, ref ConnectionA.Orientation, out worldRestrictedAxis1);
            Quaternion.Transform(ref localRestrictedAxis2, ref ConnectionA.Orientation, out worldRestrictedAxis2);

            Vector3 worldLineAnchor;
            Quaternion.Transform(ref LocalLineAnchor, ref ConnectionA.Orientation, out worldLineAnchor);
            Vector3.Add(ref worldLineAnchor, ref ConnectionA.Position, out worldLineAnchor);
            Vector3 lineDirection;
            Quaternion.Transform(ref localLineDirection, ref ConnectionA.Orientation, out lineDirection);

            Vector3 rB;
            Quaternion.Transform(ref LocalAnchorB, ref ConnectionB.Orientation, out rB);
            Vector3 worldPoint;
            Vector3.Add(ref rB, ref ConnectionB.Position, out worldPoint);

            //Find the point on the line closest to the world point.
            Vector3 offset;
            Vector3.Subtract(ref worldPoint, ref worldLineAnchor, out offset);
            float distanceAlongAxis;
            Vector3.Dot(ref offset, ref lineDirection, out distanceAlongAxis);

            Vector3 worldNearPoint;
            Vector3.Multiply(ref lineDirection, distanceAlongAxis, out offset);
            Vector3.Add(ref worldLineAnchor, ref offset, out worldNearPoint);
            Vector3 rA;
            Vector3.Subtract(ref worldNearPoint, ref ConnectionA.Position, out rA);

            //Error
            Vector3 error3D;
            Vector3.Subtract(ref worldPoint, ref worldNearPoint, out error3D);

            Vector2 error;
            Vector3.Dot(ref error3D, ref worldRestrictedAxis1, out error.X);
            Vector3.Dot(ref error3D, ref worldRestrictedAxis2, out error.Y);

            velocityBias.X = errorCorrectionFactor * error.X;
            velocityBias.Y = errorCorrectionFactor * error.Y;


            //Set up the jacobians
            Vector3 angularA1, angularA2, angularB1, angularB2;
            Vector3.Cross(ref rA, ref worldRestrictedAxis1, out angularA1);
            Vector3.Cross(ref rA, ref worldRestrictedAxis2, out angularA2);
            Vector3.Cross(ref worldRestrictedAxis1, ref rB, out angularB1);
            Vector3.Cross(ref worldRestrictedAxis2, ref rB, out angularB2);

            //Put all the 1x3 jacobians into a 3x3 matrix representation.
            linearJacobianA = new Matrix3x3
            {
                M1 = worldRestrictedAxis1,
                M2 = worldRestrictedAxis2
            };
            Matrix3x3.Negate(ref linearJacobianA, out linearJacobianB);

            angularJacobianA = new Matrix3x3
            {
                M1 = angularA1,
                M2 = angularA2,
            };
            angularJacobianB = new Matrix3x3
            {
                M1 = angularB1,
                M2 = angularB2,
            };
        }
    }
}
