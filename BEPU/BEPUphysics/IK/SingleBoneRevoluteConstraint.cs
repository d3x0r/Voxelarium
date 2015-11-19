using System;
using BEPUutilities;

namespace BEPUik
{
    public class SingleBoneRevoluteConstraint : SingleBoneConstraint
    {
        private Vector3 freeAxis;
        private Vector3 constrainedAxis1;
        private Vector3 constrainedAxis2;

        /// <summary>
        /// Gets or sets the direction to constrain the bone free axis to.
        /// </summary>
        public Vector3 FreeAxis
        {
            get { return freeAxis; }
            set
            {
                freeAxis = value;
                Vector3.Cross(ref freeAxis,ref  Vector3.Up, out constrainedAxis1);
                if (constrainedAxis1.LengthSquared() < Toolbox.Epsilon)
                {
                    Vector3.Cross(ref freeAxis, ref Vector3.Right, out constrainedAxis1);
                }
                constrainedAxis1.Normalize();
                Vector3.Cross(ref freeAxis, ref constrainedAxis1, out constrainedAxis2);
            }
        }


        /// <summary>
        /// Axis of allowed rotation in the bone's local space.
        /// </summary>
        public Vector3 BoneLocalFreeAxis;

        protected internal override void UpdateJacobiansAndVelocityBias()
        {
 

            linearJacobian = new Matrix3x3();

            Vector3 boneAxis;
            Quaternion.Transform(ref BoneLocalFreeAxis, ref TargetBone.Orientation, out boneAxis);


            angularJacobian = new Matrix3x3
            {
                M1 = constrainedAxis1,
                M2 = constrainedAxis2,
            };


            Vector3 error;
            Vector3.Cross(ref boneAxis, ref freeAxis, out error);
            Vector2 constraintSpaceError;
            Vector3.Dot(ref error, ref constrainedAxis1, out constraintSpaceError.X);
            Vector3.Dot(ref error, ref constrainedAxis2, out constraintSpaceError.Y);
            velocityBias.X = errorCorrectionFactor * constraintSpaceError.X;
            velocityBias.Y = errorCorrectionFactor * constraintSpaceError.Y;


        }


    }
}
