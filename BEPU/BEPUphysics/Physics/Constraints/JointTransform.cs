using System;
using BEPUutilities;
 

namespace BEPUphysics.Constraints
{
    /// <summary>
    /// Defines a three dimensional orthonormal basis used by a constraint.
    /// </summary>
    public class JointBasis3D
    {
        internal Vector3 localPrimaryAxis = Vector3.Backward;
        internal Vector3 localXAxis = Vector3.Right;
        internal Vector3 localYAxis = Vector3.Up;
        internal Vector3 primaryAxis = Vector3.Backward;
        internal Matrix3x3 rotationMatrix = Matrix3x3.Identity;
        internal Vector3 xAxis = Vector3.Right;
        internal Vector3 yAxis = Vector3.Up;

        /// <summary>
        /// Gets the primary axis of the transform in local space.
        /// </summary>
        public Vector3 LocalPrimaryAxis
        {
            get { return localPrimaryAxis; }
        }

        /// <summary>
        /// Gets or sets the local transform of the basis.
        /// </summary>
        public Matrix3x3 LocalTransform
        {
            get
            {
                var toReturn = new Matrix3x3 {M1 = localXAxis, M2 = localYAxis, M3 = localPrimaryAxis};
                return toReturn;
            }
            set { SetLocalAxes(ref value); }
        }

        /// <summary>
        /// Gets the X axis of the transform in local space.
        /// </summary>
        public Vector3 LocalXAxis
        {
            get { return localXAxis; }
        }

        /// <summary>
        /// Gets the Y axis of the transform in local space.
        /// </summary>
        public Vector3 LocalYAxis
        {
            get { return localYAxis; }
        }

        /// <summary>
        /// Gets the primary axis of the transform.
        /// </summary>
        public Vector3 PrimaryAxis
        {
            get { return primaryAxis; }
        }

        /// <summary>
        /// Gets or sets the rotation matrix used by the joint transform to convert local space axes to world space.
        /// </summary>
        public Matrix3x3 RotationMatrix
        {
            get { return rotationMatrix; }
            set
            {
                rotationMatrix = value;
                ComputeWorldSpaceAxes();
            }
        }

        /// <summary>
        /// Gets or sets the world transform of the basis.
        /// </summary>
        public Matrix3x3 WorldTransform
        {
            get
            {
                var toReturn = new Matrix3x3 {M1 = xAxis, M2 = yAxis, M3 = primaryAxis};
                return toReturn;
            }
            set { SetWorldAxes(ref value); }
        }

        /// <summary>
        /// Gets the X axis of the transform.
        /// </summary>
        public Vector3 XAxis
        {
            get { return xAxis; }
        }

        /// <summary>
        /// Gets the Y axis of the transform.
        /// </summary>
        public Vector3 YAxis
        {
            get { return yAxis; }
        }


        /// <summary>
        /// Sets up the axes of the transform and ensures that it is an orthonormal basis.
        /// </summary>
        /// <param name="primaryAxis">First axis in the transform.  Usually aligned along the main axis of a joint, like the twist axis of a TwistLimit.</param>
        /// <param name="xAxis">Second axis in the transform.</param>
        /// <param name="yAxis">Third axis in the transform.</param>
        /// <param name="rotationMatrix">Matrix to use to transform the local axes into world space.</param>
        public void SetLocalAxes( ref Vector3 primaryAxis, ref Vector3 xAxis, ref Vector3 yAxis, ref Matrix3x3 rotationMatrix )
        {
            this.rotationMatrix = rotationMatrix;
            SetLocalAxes( ref primaryAxis, ref xAxis, ref yAxis );
        }


        /// <summary>
        /// Sets up the axes of the transform and ensures that it is an orthonormal basis.
        /// </summary>
        /// <param name="primaryAxis">First axis in the transform.  Usually aligned along the main axis of a joint, like the twist axis of a TwistLimit.</param>
        /// <param name="xAxis">Second axis in the transform.</param>
        /// <param name="yAxis">Third axis in the transform.</param>
        public void SetLocalAxes( ref Vector3 primaryAxis, ref Vector3 xAxis, ref Vector3 yAxis )
        {
            if (Math.Abs(Vector3.Dot(ref primaryAxis, ref xAxis ) ) > Toolbox.BigEpsilon ||
                Math.Abs(Vector3.Dot( ref primaryAxis, ref yAxis ) ) > Toolbox.BigEpsilon ||
                Math.Abs(Vector3.Dot( ref xAxis, ref yAxis ) ) > Toolbox.BigEpsilon)
                throw new ArgumentException("The axes provided to the joint transform do not form an orthonormal basis.  Ensure that each axis is perpendicular to the other two.");

            Vector3.Normalize( ref primaryAxis, out localPrimaryAxis );
            Vector3.Normalize( ref xAxis, out localXAxis );
            Vector3.Normalize( ref yAxis, out localYAxis );
            ComputeWorldSpaceAxes();
        }

        /// <summary>
        /// Sets up the axes of the transform and ensures that it is an orthonormal basis.
        /// </summary>
        /// <param name="matrix">Rotation matrix representing the three axes.
        /// The matrix's backward vector is used as the primary axis.  
        /// The matrix's right vector is used as the x axis.
        /// The matrix's up vector is used as the y axis.</param>
        public void SetLocalAxes( ref Matrix3x3 matrix )
        {
            if (Math.Abs(Vector3.Dot(ref  matrix.M3, ref matrix.M1 ) ) > Toolbox.BigEpsilon ||
                Math.Abs(Vector3.Dot( ref matrix.M3, ref matrix.M2 ) ) > Toolbox.BigEpsilon ||
                Math.Abs(Vector3.Dot( ref matrix.M1, ref matrix.M2 ) ) > Toolbox.BigEpsilon)
                throw new ArgumentException("The axes provided to the joint transform do not form an orthonormal basis.  Ensure that each axis is perpendicular to the other two.");

            Vector3.Normalize( ref matrix.M3, out localPrimaryAxis );
            Vector3.Normalize( ref matrix.M1, out localXAxis );
            Vector3.Normalize( ref matrix.M2, out localYAxis);
            ComputeWorldSpaceAxes();
        }


        /// <summary>
        /// Sets up the axes of the transform and ensures that it is an orthonormal basis.
        /// </summary>
        /// <param name="primaryAxis">First axis in the transform.  Usually aligned along the main axis of a joint, like the twist axis of a TwistLimit.</param>
        /// <param name="xAxis">Second axis in the transform.</param>
        /// <param name="yAxis">Third axis in the transform.</param>
        /// <param name="rotationMatrix">Matrix to use to transform the local axes into world space.</param>
        public void SetWorldAxes( ref Vector3 primaryAxis, ref Vector3 xAxis, ref Vector3 yAxis, ref Matrix3x3 rotationMatrix )
        {
            this.rotationMatrix = rotationMatrix;
            SetWorldAxes( ref primaryAxis, ref xAxis, ref yAxis );
        }

        /// <summary>
        /// Sets up the axes of the transform and ensures that it is an orthonormal basis.
        /// </summary>
        /// <param name="primaryAxis">First axis in the transform.  Usually aligned along the main axis of a joint, like the twist axis of a TwistLimit.</param>
        /// <param name="xAxis">Second axis in the transform.</param>
        /// <param name="yAxis">Third axis in the transform.</param>
        public void SetWorldAxes( ref Vector3 primaryAxis, ref Vector3 xAxis, ref Vector3 yAxis )
        {
            if (Math.Abs(Vector3.Dot( ref primaryAxis, ref xAxis ) ) > Toolbox.BigEpsilon ||
                Math.Abs(Vector3.Dot( ref primaryAxis, ref yAxis ) ) > Toolbox.BigEpsilon ||
                Math.Abs(Vector3.Dot( ref xAxis, ref yAxis ) ) > Toolbox.BigEpsilon)
                throw new ArgumentException("The axes provided to the joint transform do not form an orthonormal basis.  Ensure that each axis is perpendicular to the other two.");

            Vector3.Normalize( ref primaryAxis, out this.primaryAxis);
            Vector3.Normalize( ref xAxis, out this.xAxis );
            Vector3.Normalize( ref yAxis, out this.yAxis );
            Matrix3x3.TransformTranspose(ref this.primaryAxis, ref rotationMatrix, out localPrimaryAxis);
            Matrix3x3.TransformTranspose(ref this.xAxis, ref rotationMatrix, out localXAxis);
            Matrix3x3.TransformTranspose(ref this.yAxis, ref rotationMatrix, out localYAxis);
        }

        /// <summary>
        /// Sets up the axes of the transform and ensures that it is an orthonormal basis.
        /// </summary>
        /// <param name="matrix">Rotation matrix representing the three axes.
        /// The matrix's backward vector is used as the primary axis.  
        /// The matrix's right vector is used as the x axis.
        /// The matrix's up vector is used as the y axis.</param>
        public void SetWorldAxes( ref Matrix3x3 matrix )
        {
            if (Math.Abs(Vector3.Dot( ref matrix.M3, ref matrix.M1 ) ) > Toolbox.BigEpsilon ||
                Math.Abs(Vector3.Dot( ref matrix.M3, ref matrix.M2)) > Toolbox.BigEpsilon ||
                Math.Abs(Vector3.Dot( ref matrix.M1, ref matrix.M2 ) ) > Toolbox.BigEpsilon)
                throw new ArgumentException("The axes provided to the joint transform do not form an orthonormal basis.  Ensure that each axis is perpendicular to the other two.");

            Vector3.Normalize( ref matrix.M3, out primaryAxis);
            Vector3.Normalize( ref matrix.M1, out xAxis);
            Vector3.Normalize( ref matrix.M2, out yAxis);
            Matrix3x3.TransformTranspose(ref this.primaryAxis, ref rotationMatrix, out localPrimaryAxis);
            Matrix3x3.TransformTranspose(ref this.xAxis, ref rotationMatrix, out localXAxis);
            Matrix3x3.TransformTranspose(ref this.yAxis, ref rotationMatrix, out localYAxis);
        }

        internal void ComputeWorldSpaceAxes()
        {
            Matrix3x3.Transform(ref localPrimaryAxis, ref rotationMatrix, out primaryAxis);
            Matrix3x3.Transform(ref localXAxis, ref rotationMatrix, out xAxis);
            Matrix3x3.Transform(ref localYAxis, ref rotationMatrix, out yAxis);
        }
    }

    /// <summary>
    /// Defines a two axes which are perpendicular to each other used by a constraint.
    /// </summary>
    public class JointBasis2D
    {
        internal Vector3 localPrimaryAxis = Vector3.Backward;
        internal Vector3 localXAxis = Vector3.Right;
        internal Vector3 primaryAxis = Vector3.Backward;
        internal Matrix3x3 rotationMatrix = Matrix3x3.Identity;
        internal Vector3 xAxis = Vector3.Right;

        /// <summary>
        /// Gets the primary axis of the transform in local space.
        /// </summary>
        public Vector3 LocalPrimaryAxis
        {
            get { return localPrimaryAxis; }
        }

        /// <summary>
        /// Gets the X axis of the transform in local space.
        /// </summary>
        public Vector3 LocalXAxis
        {
            get { return localXAxis; }
        }

        /// <summary>
        /// Gets the primary axis of the transform.
        /// </summary>
        public Vector3 PrimaryAxis
        {
            get { return primaryAxis; }
        }

        /// <summary>
        /// Gets or sets the rotation matrix used by the joint transform to convert local space axes to world space.
        /// </summary>
        public Matrix3x3 RotationMatrix
        {
            get { return rotationMatrix; }
            set
            {
                rotationMatrix = value;
                ComputeWorldSpaceAxes();
            }
        }

        /// <summary>
        /// Gets the X axis of the transform.
        /// </summary>
        public Vector3 XAxis
        {
            get { return xAxis; }
        }


        /// <summary>
        /// Sets up the axes of the transform and ensures that it is an orthonormal basis.
        /// </summary>
        /// <param name="primaryAxis">First axis in the transform.  Usually aligned along the main axis of a joint, like the twist axis of a TwistLimit.</param>
        /// <param name="xAxis">Second axis in the transform.</param>
        /// <param name="rotationMatrix">Matrix to use to transform the local axes into world space.</param>
        public void SetLocalAxes( ref Vector3 primaryAxis, ref Vector3 xAxis, ref Matrix3x3 rotationMatrix )
        {
            this.rotationMatrix = rotationMatrix;
            SetLocalAxes( ref primaryAxis, ref xAxis );
        }

        /// <summary>
        /// Sets up the axes of the transform and ensures that it is an orthonormal basis.
        /// </summary>
        /// <param name="primaryAxis">First axis in the transform.  Usually aligned along the main axis of a joint, like the twist axis of a TwistLimit.</param>
        /// <param name="xAxis">Second axis in the transform.</param>
        public void SetLocalAxes( ref Vector3 primaryAxis, ref Vector3 xAxis )
        {
            if (Math.Abs(Vector3.Dot(ref primaryAxis, ref xAxis)) > Toolbox.BigEpsilon)
                throw new ArgumentException("The axes provided to the joint transform are not perpendicular.  Ensure that the specified axes form a valid constraint.");

            Vector3.Normalize(ref primaryAxis, out localPrimaryAxis);
            Vector3.Normalize(ref xAxis, out localXAxis );
            ComputeWorldSpaceAxes();
        }

        /// <summary>
        /// Sets up the axes of the transform and ensures that it is an orthonormal basis.
        /// </summary>
        /// <param name="matrix">Rotation matrix representing the three axes.
        /// The matrix's backward vector is used as the primary axis.  
        /// The matrix's right vector is used as the x axis.</param>
        public void SetLocalAxes( ref Matrix3x3 matrix )
        {
            if (Math.Abs(Vector3.Dot(ref matrix.M3, ref matrix.M1)) > Toolbox.BigEpsilon)
                throw new ArgumentException("The axes provided to the joint transform are not perpendicular.  Ensure that the specified axes form a valid constraint.");
            Vector3.Normalize(ref matrix.M2, out localPrimaryAxis);
            Vector3.Normalize(ref matrix.M1, out localXAxis);
            ComputeWorldSpaceAxes();
        }


        /// <summary>
        /// Sets up the axes of the transform and ensures that it is an orthonormal basis.
        /// </summary>
        /// <param name="primaryAxis">First axis in the transform.  Usually aligned along the main axis of a joint, like the twist axis of a TwistLimit.</param>
        /// <param name="xAxis">Second axis in the transform.</param>
        /// <param name="rotationMatrix">Matrix to use to transform the local axes into world space.</param>
        public void SetWorldAxes( ref Vector3 primaryAxis, ref Vector3 xAxis, ref Matrix3x3 rotationMatrix )
        {
            this.rotationMatrix = rotationMatrix;
            SetWorldAxes( ref primaryAxis, ref xAxis );
        }
#if ALLOW_LAZY_VECTORS
		public void SetWorldAxes( Vector3 primaryAxis, Vector3 xAxis, Matrix3x3 rotationMatrix )
		{
			SetWorldAxes( ref primaryAxis, ref xAxis, ref rotationMatrix );
		}
#endif
		/// <summary>
		/// Sets up the axes of the transform and ensures that it is an orthonormal basis.
		/// </summary>
		/// <param name="primaryAxis">First axis in the transform.  Usually aligned along the main axis of a joint, like the twist axis of a TwistLimit.</param>
		/// <param name="xAxis">Second axis in the transform.</param>
		public void SetWorldAxes(ref Vector3 primaryAxis, ref Vector3 xAxis)
        {
            if (Math.Abs(Vector3.Dot( ref primaryAxis, ref xAxis)) > Toolbox.BigEpsilon)
                throw new ArgumentException("The axes provided to the joint transform are not perpendicular.  Ensure that the specified axes form a valid constraint.");
            Vector3.Normalize( ref primaryAxis, out this.primaryAxis);
            Vector3.Normalize( ref xAxis, out this.xAxis );
            Matrix3x3.TransformTranspose(ref this.primaryAxis, ref rotationMatrix, out localPrimaryAxis);
            Matrix3x3.TransformTranspose(ref this.xAxis, ref rotationMatrix, out localXAxis);
        }
#if ALLOW_LAZY_VECTORS
		public void SetWorldAxes( Vector3 primaryAxis, Vector3 xAxis )
		{
			SetWorldAxes( ref primaryAxis, ref xAxis );
		}
#endif
		/// <summary>
		/// Sets up the axes of the transform and ensures that it is an orthonormal basis.
		/// </summary>
		/// <param name="matrix">Rotation matrix representing the three axes.
		/// The matrix's backward vector is used as the primary axis.  
		/// The matrix's right vector is used as the x axis.</param>
		public void SetWorldAxes(Matrix3x3 matrix)
        {
            if (Math.Abs(Vector3.Dot(ref matrix.M3, ref matrix.M1)) > Toolbox.BigEpsilon)
                throw new ArgumentException("The axes provided to the joint transform are not perpendicular.  Ensure that the specified axes form a valid constraint.");
            Vector3.Normalize(ref matrix.M3, out primaryAxis);
            Vector3.Normalize(ref matrix.M1, out xAxis);
            Matrix3x3.TransformTranspose(ref this.primaryAxis, ref rotationMatrix, out localPrimaryAxis);
            Matrix3x3.TransformTranspose(ref this.xAxis, ref rotationMatrix, out localXAxis);
        }

        internal void ComputeWorldSpaceAxes()
        {
            Matrix3x3.Transform(ref localPrimaryAxis, ref rotationMatrix, out primaryAxis);
            Matrix3x3.Transform(ref localXAxis, ref rotationMatrix, out xAxis);
        }
    }
}