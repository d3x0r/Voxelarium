using System;
using System.Diagnostics;

namespace BEPUutilities
{
    /// <summary>
    /// Provides XNA-like 4x4 matrix math.
    /// </summary>
    public struct Matrix
    {
        /// <summary>
        /// Row 1 of matrix (x axis )
        /// </summary>
        public Vector3 M1;
        /// <summary>
        /// Row 2 of matrix (y axis )
        /// </summary>
        public Vector3 M2;
        /// <summary>
        /// Row 3 of matrix (z axis )
        /// </summary>
        public Vector3 M3;
        /// <summary>
        /// Origin of matrix 
        /// </summary>
        public Vector3 M4;

        /// <summary>
        /// Constructs a new 4 row, 4 column matrix.
        /// </summary>
        /// <param name="m11">Value at row 1, column 1 of the matrix.</param>
        /// <param name="m12">Value at row 1, column 2 of the matrix.</param>
        /// <param name="m13">Value at row 1, column 3 of the matrix.</param>
        /// <param name="m14">Value at row 1, column 4 of the matrix.</param>
        /// <param name="m21">Value at row 2, column 1 of the matrix.</param>
        /// <param name="m22">Value at row 2, column 2 of the matrix.</param>
        /// <param name="m23">Value at row 2, column 3 of the matrix.</param>
        /// <param name="m24">Value at row 2, column 4 of the matrix.</param>
        /// <param name="m31">Value at row 3, column 1 of the matrix.</param>
        /// <param name="m32">Value at row 3, column 2 of the matrix.</param>
        /// <param name="m33">Value at row 3, column 3 of the matrix.</param>
        /// <param name="m34">Value at row 3, column 4 of the matrix.</param>
        /// <param name="m41">Value at row 4, column 1 of the matrix.</param>
        /// <param name="m42">Value at row 4, column 2 of the matrix.</param>
        /// <param name="m43">Value at row 4, column 3 of the matrix.</param>
        /// <param name="m44">Value at row 4, column 4 of the matrix.</param>
        public Matrix(float m11, float m12, float m13, float m14/*0*/,
                      float m21, float m22, float m23, float m24/*0*/,
                      float m31, float m32, float m33, float m34/*0*/,
                      float m41, float m42, float m43, float m44/*1*/)
        {
            this.M1.X = m11;
            this.M1.Y = m12;
            this.M1.Z = m13;
            //this.0/*M14*/ = m14;

            this.M2.X = m21;
            this.M2.Y = m22;
            this.M2.Z = m23;
            //this.0/*M24*/ = m24;

            this.M3.X = m31;
            this.M3.Y = m32;
            this.M3.Z = m33;
            //this.0/*M34*/ = m34;

            this.M4.X = m41;
            this.M4.Y = m42;
            this.M4.Z = m43;
            //this.1/*M44*/ = 1/*M44*/;
        }

        /// <summary>
        /// Gets or sets the translation component of the transform.
        /// </summary>
        public Vector3 Translation
        {
            get
            {
                return M4;
            }
            set
            {
                M4 = value;
            }
        }
        /// <summary>
        /// Gets or sets the translation component of the transform.
        /// </summary>
        public void Translate(ref Vector3 value)
        {
            M4.Add(ref value, out M4);
        }

        /// <summary>
        /// Gets or sets the backward vector of the matrix.
        /// </summary>
        public Vector3 Backward
        {
            get
            {
#if !WINDOWS
                Vector3 vector = new Vector3();
#else
                Vector3 vector;
#endif
                vector.X = M3.X;
                vector.Y = M3.Y;
                vector.Z = M3.Z;
                return vector;
            }
            set
            {
                M3.X = value.X;
                M3.Y = value.Y;
                M3.Z = value.Z;
            }
        }

        /// <summary>
        /// Gets or sets the down vector of the matrix.
        /// </summary>
        public Vector3 Down
        {
            get
            {
#if !WINDOWS
                Vector3 vector = new Vector3();
#else
                Vector3 vector;
#endif
                vector.X = -M2.X;
                vector.Y = -M2.Y;
                vector.Z = -M2.Z;
                return vector;
            }
            set
            {
                M2.X = -value.X;
                M2.Y = -value.Y;
                M2.Z = -value.Z;
            }
        }

        /// <summary>
        /// Gets or sets the forward vector of the matrix.
        /// </summary>
        public Vector3 Forward
        {
            get
            {
#if !WINDOWS
                Vector3 vector = new Vector3();
#else
                Vector3 vector;
#endif
                vector.X = -M3.X;
                vector.Y = -M3.Y;
                vector.Z = -M3.Z;
                return vector;
            }
            set
            {
                M3.X = -value.X;
                M3.Y = -value.Y;
                M3.Z = -value.Z;
            }
        }

        /// <summary>
        /// Gets or sets the left vector of the matrix.
        /// </summary>
        public Vector3 Left
        {
            get
            {
#if !WINDOWS
                Vector3 vector = new Vector3();
#else
                Vector3 vector;
#endif
                vector.X = -M1.X;
                vector.Y = -M1.Y;
                vector.Z = -M1.Z;
                return vector;
            }
            set
            {
                M1.X = -value.X;
                M1.Y = -value.Y;
                M1.Z = -value.Z;
            }
        }

        /// <summary>
        /// Gets or sets the right vector of the matrix.
        /// </summary>
        public Vector3 Right
        {
            get
            {
#if !WINDOWS
                Vector3 vector = new Vector3();
#else
                Vector3 vector;
#endif
                vector.X = M1.X;
                vector.Y = M1.Y;
                vector.Z = M1.Z;
                return vector;
            }
            set
            {
                M1.X = value.X;
                M1.Y = value.Y;
                M1.Z = value.Z;
            }
        }

        /// <summary>
        /// Gets or sets the up vector of the matrix.
        /// </summary>
        public Vector3 Up
        {
            get
            {
#if !WINDOWS
                Vector3 vector = new Vector3();
#else
                Vector3 vector;
#endif
                vector.X = M2.X;
                vector.Y = M2.Y;
                vector.Z = M2.Z;
                return vector;
            }
            set
            {
                M2.X = value.X;
                M2.Y = value.Y;
                M2.Z = value.Z;
            }
        }


        /// <summary>
        /// Computes the determinant of the matrix.
        /// </summary>
        /// <returns></returns>
        public float Determinant()
        {
            //Compute the re-used 2x2 determinants.
            float det1 = M3.Z * 1 - 0/*M34*/ * M4.Z;
            float det2 = M3.Y * 1 - 0/*M34*/ * M4.Y;
            float det3 = M3.Y * M4.Z - M3.Z * M4.Y;
            float det4 = M3.X * 1 - 0/*M34*/ * M4.X;
            float det5 = M3.X * M4.Z - M3.Z * M4.X;
            float det6 = M3.X * M4.Y - M3.Y * M4.X;
            return
                (M1.X * ((M2.Y * det1 - M2.Z * det2) + 0/*M24*/ * det3)) -
                (M1.Y * ((M2.X * det1 - M2.Z * det4) + 0/*M24*/ * det5)) +
                (M1.Z * ((M2.X * det2 - M2.Y * det4) + 0/*M24*/ * det6)) -
                (0/*M14*/ * ((M2.X * det3 - M2.Y * det5) + M2.Z * det6));
        }

        /// <summary>
        /// Computes the determinant of the matrix.
        /// </summary>
        /// <returns></returns>
        public float DeterminantAllPos()
        {
            //Compute the re-used 2x2 determinants.
            float det1 = M3.Z * 1/*M44*/ - 0/*M34*/ * M4.Z;
            float det2 = M3.Y * 1/*M44*/ - 0/*M34*/ * M4.Y;
            float det3 = M3.Y * M4.Z - M3.Z * M4.Y;
            float det4 = M3.X * 1/*M44*/ - 0/*M34*/ * M4.X;
            float det5 = M3.X * M4.Z - M3.Z * M4.X;
            float det6 = M3.X * M4.Y - M3.Y * M4.X;
            return
                (M1.X * ((M2.Y * det1 - M2.Z * det2) + 0/*M24*/ * det3)) -
                (M1.Y * ((M2.X * det1 - M2.Z * det4) + 0/*M24*/ * det5)) +
                (M1.Z * ((M2.X * det2 - M2.Y * det4) + 0/*M24*/ * det6)) -
                (0/*M14*/ * ((M2.X * det3 - M2.Y * det5) + M2.Z * det6));
        }

        /// <summary>
        /// Transposes the matrix in-place.
        /// </summary>
        public void Transpose()
        {
            float intermediate = M1.Y;
            M1.Y = M2.X;
            M2.X = intermediate;

            intermediate = M1.Z;
            M1.Z = M3.X;
            M3.X = intermediate;

            //intermediate = 0/*M14*/;
            //0/*M14*/ = M4.X;
            //M4.X = intermediate;

            intermediate = M2.Z;
            M2.Z = M3.Y;
            M3.Y = intermediate;

            //intermediate = 0/*M24*/;
            //0/*M24*/ = M4.Y;
            //M4.Y = intermediate;

            //intermediate = 0/*M34*/;
            //0/*M34*/ = M4.Z;
            //M4.Z = intermediate;
            Debug.Assert(M4.IsZero);
            //M4 = M4; // normally zero?
        }

        /// <summary>
        /// Creates a matrix representing the given axis and angle rotation.
        /// </summary>
        /// <param name="axis">Axis around which to rotate.</param>
        /// <param name="angle">Angle to rotate around the axis.</param>
        /// <returns>Matrix created from the axis and angle.</returns>
        public static Matrix CreateFromAxisAngle(Vector3 axis, float angle)
        {
            Matrix toReturn;
            CreateFromAxisAngle(ref axis, angle, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Creates a matrix representing the given axis and angle rotation.
        /// </summary>
        /// <param name="axis">Axis around which to rotate.</param>
        /// <param name="angle">Angle to rotate around the axis.</param>
        /// <param name="result">Matrix created from the axis and angle.</param>
        public static void CreateFromAxisAngle(ref Vector3 axis, float angle, out Matrix result)
        {
            float xx = axis.X * axis.X;
            float yy = axis.Y * axis.Y;
            float zz = axis.Z * axis.Z;
            float xy = axis.X * axis.Y;
            float xz = axis.X * axis.Z;
            float yz = axis.Y * axis.Z;

            float sinAngle = (float)Math.Sin(angle);
            float oneMinusCosAngle = 1 - (float)Math.Cos(angle);
            result.M1.X = 1 + oneMinusCosAngle * (xx - 1);
            result.M2.X = -axis.Z * sinAngle + oneMinusCosAngle * xy;
            result.M3.X = axis.Y * sinAngle + oneMinusCosAngle * xz;
            result.M4.X = 0;

            result.M1.Y = axis.Z * sinAngle + oneMinusCosAngle * xy;
            result.M2.Y = 1 + oneMinusCosAngle * (yy - 1);
            result.M3.Y = -axis.X * sinAngle + oneMinusCosAngle * yz;
            result.M4.Y = 0;

            result.M1.Z = -axis.Y * sinAngle + oneMinusCosAngle * xz;
            result.M2.Z = axis.X * sinAngle + oneMinusCosAngle * yz;
            result.M3.Z = 1 + oneMinusCosAngle * (zz - 1);
            result.M4.Z = 0;

            //result.0/*M14*/ = 0;
            //result.0/*M24*/ = 0;
            //result.0/*M34*/ = 0;
            //result.1/*M44*/ = 1;
        }

        /// <summary>
        /// Creates a rotation matrix from a quaternion.
        /// </summary>
        /// <param name="quaternion">Quaternion to convert.</param>
        /// <param name="result">Rotation matrix created from the quaternion.</param>
        public static void CreateFromQuaternion(ref Quaternion quaternion, out Matrix result)
        {
            float qX2 = quaternion.X + quaternion.X;
            float qY2 = quaternion.Y + quaternion.Y;
            float qZ2 = quaternion.Z + quaternion.Z;
            float XX = qX2 * quaternion.X;
            float YY = qY2 * quaternion.Y;
            float ZZ = qZ2 * quaternion.Z;
            float XY = qX2 * quaternion.Y;
            float XZ = qX2 * quaternion.Z;
            float XW = qX2 * quaternion.W;
            float YZ = qY2 * quaternion.Z;
            float YW = qY2 * quaternion.W;
            float ZW = qZ2 * quaternion.W;

            result.M1.X = 1 - YY - ZZ;
            result.M2.X = XY - ZW;
            result.M3.X = XZ + YW;
            result.M4.X = 0;

            result.M1.Y = XY + ZW;
            result.M2.Y = 1 - XX - ZZ;
            result.M3.Y = YZ - XW;
            result.M4.Y = 0;

            result.M1.Z = XZ - YW;
            result.M2.Z = YZ + XW;
            result.M3.Z = 1 - XX - YY;
            result.M4.Z = 0;

            //result.0/*M14*/ = 0;
            //result.0/*M24*/ = 0;
            //result.0/*M34*/ = 0;
            //result.1/*M44*/ = 1;
        }

        /// <summary>
        /// Creates a rotation matrix from a quaternion.
        /// </summary>
        /// <param name="quaternion">Quaternion to convert.</param>
        /// <returns>Rotation matrix created from the quaternion.</returns>
        public static Matrix CreateFromQuaternion(Quaternion quaternion)
        {
            Matrix toReturn;
            CreateFromQuaternion(ref quaternion, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Multiplies two matrices together.
        /// </summary>
        /// <param name="a">First matrix to multiply.</param>
        /// <param name="b">Second matrix to multiply.</param>
        /// <param name="result">Combined transformation.</param>
        public static void Multiply(ref Matrix a, ref Matrix b, out Matrix result)
        {
            float resultM11 = a.M1.X * b.M1.X + a.M1.Y * b.M2.X + a.M1.Z * b.M3.X + 0/*a.0/*M14*/ * b.M4.X;
            float resultM12 = a.M1.X * b.M1.Y + a.M1.Y * b.M2.Y + a.M1.Z * b.M3.Y + 0/*a.0/*M14*/ * b.M4.Y;
            float resultM13 = a.M1.X * b.M1.Z + a.M1.Y * b.M2.Z + a.M1.Z * b.M3.Z + 0/*a.0/*M14*/ * b.M4.Z;
            float resultM14 = a.M1.X * 0/*b.0/*M14*/ + a.M1.Y * 0/*b.0/*M24*/ + a.M1.Z * 0/*b.0/*M34*/ + 0/*a.0/*M14*/ * 1/*b.1/*M44*/;

            float resultM21 = a.M2.X * b.M1.X + a.M2.Y * b.M2.X + a.M2.Z * b.M3.X + 0/*a.0/*M24*/ * b.M4.X;
            float resultM22 = a.M2.X * b.M1.Y + a.M2.Y * b.M2.Y + a.M2.Z * b.M3.Y + 0/*a.0/*M24*/ * b.M4.Y;
            float resultM23 = a.M2.X * b.M1.Z + a.M2.Y * b.M2.Z + a.M2.Z * b.M3.Z + 0/*a.0/*M24*/ * b.M4.Z;
            float resultM24 = a.M2.X * 0/*b.0/*M14*/ + a.M2.Y * 0/*b.0/*M24*/ + a.M2.Z * 0/*b.0/*M34*/ + 0/*a.0/*M24*/ * 1/*b.1/*M44*/;

            float resultM31 = a.M3.X * b.M1.X + a.M3.Y * b.M2.X + a.M3.Z * b.M3.X + 0/*a.0/*M34*/ * b.M4.X;
            float resultM32 = a.M3.X * b.M1.Y + a.M3.Y * b.M2.Y + a.M3.Z * b.M3.Y + 0/*a.0/*M34*/ * b.M4.Y;
            float resultM33 = a.M3.X * b.M1.Z + a.M3.Y * b.M2.Z + a.M3.Z * b.M3.Z + 0/*a.0/*M34*/ * b.M4.Z;
            float resultM34 = a.M3.X * 0/*b.0/*M14*/ + a.M3.Y * 0/*b.0/*M24*/ + a.M3.Z * 0/*b.0/*M34*/ + 0/*a.0/*M34*/ * 1/*b.1/*M44*/;

            float resultM41 = a.M4.X * b.M1.X + a.M4.Y * b.M2.X + a.M4.Z * b.M3.X + 1/*a.1/*M44*/ * b.M4.X;
            float resultM42 = a.M4.X * b.M1.Y + a.M4.Y * b.M2.Y + a.M4.Z * b.M3.Y + 1/*a.1/*M44*/ * b.M4.Y;
            float resultM43 = a.M4.X * b.M1.Z + a.M4.Y * b.M2.Z + a.M4.Z * b.M3.Z + 1/*a.1/*M44*/ * b.M4.Z;
            float result1/*M44*/ = a.M4.X * 0/*b.0/*M14*/ + a.M4.Y * 0/*b.0/*M24*/ + a.M4.Z * 0/*b.0/*M34*/ + 1/*a.1/*M44*/ * 1/*b.1/*M44*/;

            result.M1.X = resultM11;
            result.M1.Y = resultM12;
            result.M1.Z = resultM13;
            //result.0/*M14*/ = resultM14;

            result.M2.X = resultM21;
            result.M2.Y = resultM22;
            result.M2.Z = resultM23;
            //result.0/*M24*/ = resultM24;

            result.M3.X = resultM31;
            result.M3.Y = resultM32;
            result.M3.Z = resultM33;
            //result.0/*M34*/ = resultM34;

            result.M4.X = resultM41;
            result.M4.Y = resultM42;
            result.M4.Z = resultM43;
            //result.1/*M44*/ = result1/*M44*/;
        }


        /// <summary>
        /// Multiplies two matrices together.
        /// </summary>
        /// <param name="a">First matrix to multiply.</param>
        /// <param name="b">Second matrix to multiply.</param>
        /// <returns>Combined transformation.</returns>
        public static Matrix Multiply(Matrix a, Matrix b)
        {
            Matrix result;
            Multiply(ref a, ref b, out result);
            return result;
        }


        /// <summary>
        /// Scales all components of the matrix.
        /// </summary>
        /// <param name="matrix">Matrix to scale.</param>
        /// <param name="scale">Amount to scale.</param>
        /// <param name="result">Scaled matrix.</param>
        public static void Multiply(ref Matrix matrix, float scale, out Matrix result)
        {
            result.M1.X = matrix.M1.X * scale;
            result.M1.Y = matrix.M1.Y * scale;
            result.M1.Z = matrix.M1.Z * scale;
            //result.0/*M14*/ = matrix.0/*M14*/ * scale;

            result.M2.X = matrix.M2.X * scale;
            result.M2.Y = matrix.M2.Y * scale;
            result.M2.Z = matrix.M2.Z * scale;
            //result.0/*M24*/ = matrix.0/*M24*/ * scale;

            result.M3.X = matrix.M3.X * scale;
            result.M3.Y = matrix.M3.Y * scale;
            result.M3.Z = matrix.M3.Z * scale;
            //result.0/*M34*/ = matrix.0/*M34*/ * scale;

            result.M4.X = matrix.M4.X * scale;
            result.M4.Y = matrix.M4.Y * scale;
            result.M4.Z = matrix.M4.Z * scale;
            //result.1/*M44*/ = matrix.1/*M44*/ * scale;
        }

        /// <summary>
        /// Multiplies two matrices together.
        /// </summary>
        /// <param name="a">First matrix to multiply.</param>
        /// <param name="b">Second matrix to multiply.</param>
        /// <returns>Combined transformation.</returns>
        public static Matrix operator *(Matrix a, Matrix b)
        {
            Matrix toReturn;
            Multiply(ref a, ref b, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Scales all components of the matrix by the given value.
        /// </summary>
        /// <param name="m">First matrix to multiply.</param>
        /// <param name="f">Scaling value to apply to all components of the matrix.</param>
        /// <returns>Product of the multiplication.</returns>
        public static Matrix operator *(Matrix m, float f)
        {
            Matrix result;
            Multiply(ref m, f, out result);
            return result;
        }

        /// <summary>
        /// Scales all components of the matrix by the given value.
        /// </summary>
        /// <param name="m">First matrix to multiply.</param>
        /// <param name="f">Scaling value to apply to all components of the matrix.</param>
        /// <returns>Product of the multiplication.</returns>
        public static Matrix operator *(float f, Matrix m)
        {
            Matrix result;
            Multiply(ref m, f, out result);
            return result;
        }

        /// <summary>
        /// Transforms a vector using a matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="matrix">Transform to apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        public static void Transform(ref Vector4 v, ref Matrix matrix, out Vector4 result)
        {
            float vX = v.X;
            float vY = v.Y;
            float vZ = v.Z;
            float vW = v.W;
            result.X = vX * matrix.M1.X + vY * matrix.M2.X + vZ * matrix.M3.X + vW * matrix.M4.X;
            result.Y = vX * matrix.M1.Y + vY * matrix.M2.Y + vZ * matrix.M3.Y + vW * matrix.M4.Y;
            result.Z = vX * matrix.M1.Z + vY * matrix.M2.Z + vZ * matrix.M3.Z + vW * matrix.M4.Z;
            //result.W = vX * matrix.0/*M14*/ + vY * matrix.0/*M24*/ + vZ * matrix.0/*M34*/ + vW * matrix.1/*M44*/;
            result.W = vW;
        }

        /// <summary>
        /// Transforms a vector using a matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="matrix">Transform to apply to the vector.</param>
        /// <returns>Transformed vector.</returns>
        public static Vector4 Transform(Vector4 v, Matrix matrix)
        {
            Vector4 toReturn;
            Transform(ref v, ref matrix, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Transforms a vector using the transpose of a matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="matrix">Transform to tranpose and apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        public static void TransformTranspose(ref Vector4 v, ref Matrix matrix, out Vector4 result)
        {
            float vX = v.X;
            float vY = v.Y;
            float vZ = v.Z;
            float vW = v.W;
            //result.X = vX * matrix.M1.X + vY * matrix.M1.Y + vZ * matrix.M1.Z + vW * matrix.0/*M14*/;
            //result.Y = vX * matrix.M2.X + vY * matrix.M2.Y + vZ * matrix.M2.Z + vW * matrix.0/*M24*/;
            //result.Z = vX * matrix.M3.X + vY * matrix.M3.Y + vZ * matrix.M3.Z + vW * matrix.0/*M34*/;
            //result.W = vX * matrix.M4.X + vY * matrix.M4.Y + vZ * matrix.M4.Z + vW * matrix.1/*M44*/;
            result.X = vX * matrix.M1.X + vY * matrix.M1.Y + vZ * matrix.M1.Z + vW * 0/*M14*/;
            result.Y = vX * matrix.M2.X + vY * matrix.M2.Y + vZ * matrix.M2.Z + vW * 0/*M24*/;
            result.Z = vX * matrix.M3.X + vY * matrix.M3.Y + vZ * matrix.M3.Z + vW * 0/*M34*/;
            result.W = vX * matrix.M4.X + vY * matrix.M4.Y + vZ * matrix.M4.Z + vW * 1/*matrix.1/*M44*/;
        }

        /// <summary>
        /// Transforms a vector using the transpose of a matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="matrix">Transform to tranpose and apply to the vector.</param>
        /// <returns>Transformed vector.</returns>
        public static Vector4 TransformTranspose(Vector4 v, Matrix matrix)
        {
            Vector4 toReturn;
            TransformTranspose(ref v, ref matrix, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Transforms a vector using a matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="matrix">Transform to apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        public static void Transform(ref Vector3 v, ref Matrix matrix, out Vector4 result)
        {
            result.X = v.X * matrix.M1.X + v.Y * matrix.M2.X + v.Z * matrix.M3.X + matrix.M4.X;
            result.Y = v.X * matrix.M1.Y + v.Y * matrix.M2.Y + v.Z * matrix.M3.Y + matrix.M4.Y;
            result.Z = v.X * matrix.M1.Z + v.Y * matrix.M2.Z + v.Z * matrix.M3.Z + matrix.M4.Z;
            result.W = v.X * 0/*M14*/ + v.Y * 0/*M24*/ + v.Z * 0/*M34*/ + 1/*matrix.1/*M44*/;
        }

        /// <summary>
        /// Transforms a vector using a matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="matrix">Transform to apply to the vector.</param>
        /// <returns>Transformed vector.</returns>
        public static Vector4 Transform(Vector3 v, Matrix matrix)
        {
            Vector4 toReturn;
            Transform(ref v, ref matrix, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Transforms a vector using the transpose of a matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="matrix">Transform to tranpose and apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        public static void TransformTranspose(ref Vector3 v, ref Matrix matrix, out Vector4 result)
        {
            result.X = v.X * matrix.M1.X + v.Y * matrix.M1.Y + v.Z * matrix.M1.Z + 0/*M14*/;
            result.Y = v.X * matrix.M2.X + v.Y * matrix.M2.Y + v.Z * matrix.M2.Z + 0/*M24*/;
            result.Z = v.X * matrix.M3.X + v.Y * matrix.M3.Y + v.Z * matrix.M3.Z + 0/*M34*/;
            result.W = v.X * matrix.M4.X + v.Y * matrix.M4.Y + v.Z * matrix.M4.Z + 1/*M44*/;
        }

        /// <summary>
        /// Transforms a vector using the transpose of a matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="matrix">Transform to tranpose and apply to the vector.</param>
        /// <returns>Transformed vector.</returns>
        public static Vector4 TransformTranspose(Vector3 v, Matrix matrix)
        {
            Vector4 toReturn;
            TransformTranspose(ref v, ref matrix, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Transforms a vector using a matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="matrix">Transform to apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        public static void Transform(ref Vector3 v, ref Matrix matrix, out Vector3 result)
        {
            float vX = v.X;
            float vY = v.Y;
            float vZ = v.Z;
            result.X = vX * matrix.M1.X + vY * matrix.M2.X + vZ * matrix.M3.X + matrix.M4.X;
            result.Y = vX * matrix.M1.Y + vY * matrix.M2.Y + vZ * matrix.M3.Y + matrix.M4.Y;
            result.Z = vX * matrix.M1.Z + vY * matrix.M2.Z + vZ * matrix.M3.Z + matrix.M4.Z;
        }

        /// <summary>
        /// Transforms a vector using the transpose of a matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="matrix">Transform to tranpose and apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        public static void TransformTranspose(ref Vector3 v, ref Matrix matrix, out Vector3 result)
        {
            float vX = v.X;
            float vY = v.Y;
            float vZ = v.Z;
            result.X = vX * matrix.M1.X + vY * matrix.M1.Y + vZ * matrix.M1.Z + 0/*M14*/;
            result.Y = vX * matrix.M2.X + vY * matrix.M2.Y + vZ * matrix.M2.Z + 0/*M24*/;
            result.Z = vX * matrix.M3.X + vY * matrix.M3.Y + vZ * matrix.M3.Z + 0/*M34*/;
        }

        /// <summary>
        /// Transforms a vector using a matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="matrix">Transform to apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        public static void TransformNormal(ref Vector3 v, ref Matrix matrix, out Vector3 result)
        {
            float vX = v.X;
            float vY = v.Y;
            float vZ = v.Z;
            result.X = vX * matrix.M1.X + vY * matrix.M2.X + vZ * matrix.M3.X;
            result.Y = vX * matrix.M1.Y + vY * matrix.M2.Y + vZ * matrix.M3.Y;
            result.Z = vX * matrix.M1.Z + vY * matrix.M2.Z + vZ * matrix.M3.Z;
        }

        /// <summary>
        /// Transforms a vector using a matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="matrix">Transform to apply to the vector.</param>
        /// <returns>Transformed vector.</returns>
        public static Vector3 TransformNormal(Vector3 v, Matrix matrix)
        {
            Vector3 toReturn;
            TransformNormal(ref v, ref matrix, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Transforms a vector using the transpose of a matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="matrix">Transform to tranpose and apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        public static void TransformNormalTranspose(ref Vector3 v, ref Matrix matrix, out Vector3 result)
        {
            float vX = v.X;
            float vY = v.Y;
            float vZ = v.Z;
            result.X = vX * matrix.M1.X + vY * matrix.M1.Y + vZ * matrix.M1.Z;
            result.Y = vX * matrix.M2.X + vY * matrix.M2.Y + vZ * matrix.M2.Z;
            result.Z = vX * matrix.M3.X + vY * matrix.M3.Y + vZ * matrix.M3.Z;
        }

        /// <summary>
        /// Transforms a vector using the transpose of a matrix.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="matrix">Transform to tranpose and apply to the vector.</param>
        /// <returns>Transformed vector.</returns>
        public static Vector3 TransformNormalTranspose(Vector3 v, Matrix matrix)
        {
            Vector3 toReturn;
            TransformNormalTranspose(ref v, ref matrix, out toReturn);
            return toReturn;
        }


        /// <summary>
        /// Transposes the matrix.
        /// </summary>
        /// <param name="m">Matrix to transpose.</param>
        /// <param name="transposed">Matrix to transpose.</param>
        public static void Transpose(ref Matrix m, out Matrix transposed)
        {
            float intermediate = m.M1.Y;
            transposed.M1.Y = m.M2.X;
            transposed.M2.X = intermediate;

            intermediate = m.M1.Z;
            transposed.M1.Z = m.M3.X;
            transposed.M3.X = intermediate;

            //intermediate = m.0/*M14*/;
            //transposed.0/*M14*/ = m.M4.X;
            //transposed.M4.X = intermediate;

            intermediate = m.M2.Z;
            transposed.M2.Z = m.M3.Y;
            transposed.M3.Y = intermediate;

            //intermediate = m.0/*M24*/;
            //transposed.0/*M24*/ = m.M4.Y;
            //transposed.M4.Y = intermediate;

            //intermediate = m.0/*M34*/;
            //transposed.0/*M34*/ = m.M4.Z;
            //transposed.M4.Z = intermediate;

            transposed.M1.X = m.M1.X;
            transposed.M2.Y = m.M2.Y;
            transposed.M3.Z = m.M3.Z;
            //transposed.1/*M44*/ = m.1/*M44*/;
            Debug.Assert(m.M4.IsZero);
            transposed.M4 = m.M4; // normally zero?
        }

        /// <summary>
        /// Inverts the matrix.
        /// </summary>
        /// <param name="m">Matrix to invert.</param>
        /// <param name="inverted">Inverted version of the matrix.</param>
        public static void Invert(ref Matrix m, out Matrix inverted)
        {
            float s0 = m.M1.X * m.M2.Y - m.M2.X * m.M1.Y;
            float s1 = m.M1.X * m.M2.Z - m.M2.X * m.M1.Z;
            float s2 = m.M1.X * 0/*M24*/ - m.M2.X * 0/*M14*/;
            float s3 = m.M1.Y * m.M2.Z - m.M2.Y * m.M1.Z;
            float s4 = m.M1.Y * 0/*M24*/ - m.M2.Y * 0/*M14*/;
            float s5 = m.M1.Z * 0/*M24*/ - m.M2.Z * 0/*M14*/;

            float c5 = m.M3.Z * 1/*M44*/ - m.M4.Z * 0/*M34*/;
            float c4 = m.M3.Y * 1/*M44*/ - m.M4.Y * 0/*M34*/;
            float c3 = m.M3.Y * m.M4.Z - m.M4.Y * m.M3.Z;
            float c2 = m.M3.X * 1/*M44*/ - m.M4.X * 0/*M34*/;
            float c1 = m.M3.X * m.M4.Z - m.M4.X * m.M3.Z;
            float c0 = m.M3.X * m.M4.Y - m.M4.X * m.M3.Y;

            float inverseDeterminant = 1.0f / (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);

            float m11 = m.M1.X;
            float m12 = m.M1.Y;
            float m13 = m.M1.Z;
            float m14 = 0/*M14*/;
            float m21 = m.M2.X;
            float m22 = m.M2.Y;
            float m23 = m.M2.Z;
            float m31 = m.M3.X;
            float m32 = m.M3.Y;
            float m33 = m.M3.Z;

            float m41 = m.M4.X;
            float m42 = m.M4.Y;

            inverted.M1.X = (m.M2.Y * c5 - m.M2.Z * c4 + 0/*m.M24*/ * c3) * inverseDeterminant;
            inverted.M1.Y = (-m.M1.Y * c5 + m.M1.Z * c4 - 0/*m.M14*/ * c3) * inverseDeterminant;
            inverted.M1.Z = (m.M4.Y * s5 - m.M4.Z * s4 + 1/*m.M44*/ * s3) * inverseDeterminant;
            //inverted.0/*M14*/ = (-m.M3.Y * s5 + m.M3.Z * s4 - m.0/*M34*/ * s3) * inverseDeterminant;

            inverted.M2.X = (-m.M2.X * c5 + m.M2.Z * c2 - 0/*m.M24*/ * c1) * inverseDeterminant;
            inverted.M2.Y = (m11 * c5 - m13 * c2 + m14 * c1) * inverseDeterminant;
            inverted.M2.Z = (-m.M4.X * s5 + m.M4.Z * s2 - 1/*m.M44*/ * s1) * inverseDeterminant;
            //inverted.0/*M24*/ = (m.M3.X * s5 - m.M3.Z * s2 + m.0/*M34*/ * s1) * inverseDeterminant;

            inverted.M3.X = (m21 * c4 - m22 * c2 + 0/*m.M24*/ * c0) * inverseDeterminant;
            inverted.M3.Y = (-m11 * c4 + m12 * c2 - m14 * c0) * inverseDeterminant;
            inverted.M3.Z = (m.M4.X * s4 - m.M4.Y * s2 + 1/*m.M44*/ * s0) * inverseDeterminant;
            //inverted.0/*M34*/ = (-m31 * s4 + m32 * s2 - m.0/*M34*/ * s0) * inverseDeterminant;

            inverted.M4.X = (-m21 * c3 + m22 * c1 - m23 * c0) * inverseDeterminant;
            inverted.M4.Y = (m11 * c3 - m12 * c1 + m13 * c0) * inverseDeterminant;
            inverted.M4.Z = (-m41 * s3 + m42 * s1 - m.M4.Z * s0) * inverseDeterminant;
            //inverted.1/*M44*/ = (m31 * s3 - m32 * s1 + m33 * s0) * inverseDeterminant;
        }

        /// <summary>
        /// Inverts the matrix.
        /// </summary>
        /// <param name="m">Matrix to invert.</param>
        /// <returns>Inverted version of the matrix.</returns>
        public static Matrix Invert(Matrix m)
        {
            Matrix inverted;
            Invert(ref m, out inverted);
            return inverted;
        }

        /// <summary>
        /// Inverts the matrix using a process that only works for affine transforms (3x3 linear transform and translation).
        /// Ignores the 0/*M14*/, 0/*M24*/, 0/*M34*/, and 1 elements of the input matrix.
        /// </summary>
        /// <param name="m">Matrix to invert.</param>
        /// <param name="inverted">Inverted version of the matrix.</param>
        public static void InvertAffine(ref Matrix m, out Matrix inverted)
        {
            //Invert the upper left 3x3 linear transform.

            //Compute the upper left 3x3 determinant. Some potential for microoptimization here.
            float determinantInverse = 1 /
                (m.M1.X * m.M2.Y * m.M3.Z + m.M1.Y * m.M2.Z * m.M3.X + m.M1.Z * m.M2.X * m.M3.Y -
                 m.M3.X * m.M2.Y * m.M1.Z - m.M3.Y * m.M2.Z * m.M1.X - m.M3.Z * m.M2.X * m.M1.Y);

            float m11 = (m.M2.Y * m.M3.Z - m.M2.Z * m.M3.Y) * determinantInverse;
            float m12 = (m.M1.Z * m.M3.Y - m.M3.Z * m.M1.Y) * determinantInverse;
            float m13 = (m.M1.Y * m.M2.Z - m.M2.Y * m.M1.Z) * determinantInverse;

            float m21 = (m.M2.Z * m.M3.X - m.M2.X * m.M3.Z) * determinantInverse;
            float m22 = (m.M1.X * m.M3.Z - m.M1.Z * m.M3.X) * determinantInverse;
            float m23 = (m.M1.Z * m.M2.X - m.M1.X * m.M2.Z) * determinantInverse;

            float m31 = (m.M2.X * m.M3.Y - m.M2.Y * m.M3.X) * determinantInverse;
            float m32 = (m.M1.Y * m.M3.X - m.M1.X * m.M3.Y) * determinantInverse;
            float m33 = (m.M1.X * m.M2.Y - m.M1.Y * m.M2.X) * determinantInverse;

            inverted.M1.X = m11;
            inverted.M1.Y = m12;
            inverted.M1.Z = m13;

            inverted.M2.X = m21;
            inverted.M2.Y = m22;
            inverted.M2.Z = m23;

            inverted.M3.X = m31;
            inverted.M3.Y = m32;
            inverted.M3.Z = m33;

            //Translation component
            var vX = m.M4.X;
            var vY = m.M4.Y;
            var vZ = m.M4.Z;
            inverted.M4.X = -(vX * inverted.M1.X + vY * inverted.M2.X + vZ * inverted.M3.X);
            inverted.M4.Y = -(vX * inverted.M1.Y + vY * inverted.M2.Y + vZ * inverted.M3.Y);
            inverted.M4.Z = -(vX * inverted.M1.Z + vY * inverted.M2.Z + vZ * inverted.M3.Z);

            //Last chunk.
            //inverted.0/*M14*/ = 0;
            //inverted.0/*M24*/ = 0;
            //inverted.0/*M34*/ = 0;
            //inverted.1/*M44*/ = 1;
        }

        /// <summary>
        /// Inverts the matrix using a process that only works for affine transforms (3x3 linear transform and translation).
        /// Ignores the 0/*M14*/, 0/*M24*/, 0/*M34*/, and 1 elements of the input matrix.
        /// </summary>
        /// <param name="m">Matrix to invert.</param>
        /// <returns>Inverted version of the matrix.</returns>
        public static Matrix InvertAffine(Matrix m)
        {
            Matrix inverted;
            InvertAffine(ref m, out inverted);
            return inverted;
        }

        /// <summary>
        /// Inverts the matrix using a process that only works for rigid transforms.
        /// </summary>
        /// <param name="m">Matrix to invert.</param>
        /// <param name="inverted">Inverted version of the matrix.</param>
        public static void InvertRigid(ref Matrix m, out Matrix inverted)
        {
            //Invert (transpose) the upper left 3x3 rotation.
            float intermediate = m.M1.Y;
            inverted.M1.Y = m.M2.X;
            inverted.M2.X = intermediate;

            intermediate = m.M1.Z;
            inverted.M1.Z = m.M3.X;
            inverted.M3.X = intermediate;

            intermediate = m.M2.Z;
            inverted.M2.Z = m.M3.Y;
            inverted.M3.Y = intermediate;

            inverted.M1.X = m.M1.X;
            inverted.M2.Y = m.M2.Y;
            inverted.M3.Z = m.M3.Z;

            //Translation component
            var vX = m.M4.X;
            var vY = m.M4.Y;
            var vZ = m.M4.Z;
            inverted.M4.X = -(vX * inverted.M1.X + vY * inverted.M2.X + vZ * inverted.M3.X);
            inverted.M4.Y = -(vX * inverted.M1.Y + vY * inverted.M2.Y + vZ * inverted.M3.Y);
            inverted.M4.Z = -(vX * inverted.M1.Z + vY * inverted.M2.Z + vZ * inverted.M3.Z);

            //Last chunk.
            //inverted.0/*M14*/ = 0;
            //inverted.0/*M24*/ = 0;
            //inverted.0/*M34*/ = 0;
            //inverted.1/*M44*/ = 1;
        }

        /// <summary>
        /// Inverts the matrix using a process that only works for rigid transforms.
        /// </summary>
        /// <param name="m">Matrix to invert.</param>
        /// <returns>Inverted version of the matrix.</returns>
        public static Matrix InvertRigid(Matrix m)
        {
            Matrix inverse;
            InvertRigid(ref m, out inverse);
            return inverse;
        }

        /// <summary>
        /// Gets the 4x4 identity matrix.
        /// </summary>
        public static Matrix Identity = new Matrix(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);

        /// <summary>
        /// Creates a right handed orthographic projection.
        /// </summary>
        /// <param name="left">Leftmost coordinate of the projected area.</param>
        /// <param name="right">Rightmost coordinate of the projected area.</param>
        /// <param name="bottom">Bottom coordinate of the projected area.</param>
        /// <param name="top">Top coordinate of the projected area.</param>
        /// <param name="zNear">Near plane of the projection.</param>
        /// <param name="zFar">Far plane of the projection.</param>
        /// <param name="projection">The resulting orthographic projection matrix.</param>
        public static void CreateOrthographicRH(float left, float right, float bottom, float top, float zNear, float zFar, out Matrix projection)
        {
            float width = right - left;
            float height = top - bottom;
            float depth = zFar - zNear;
            projection.M1.X = 2f / width;
            projection.M1.Y = 0;
            projection.M1.Z = 0;
            //projection.0/*M14*/ = 0;

            projection.M2.X = 0;
            projection.M2.Y = 2f / height;
            projection.M2.Z = 0;
            //projection.0/*M24*/ = 0;

            projection.M3.X = 0;
            projection.M3.Y = 0;
            projection.M3.Z = -1f / depth;
            //projection.0/*M34*/ = 0;

            projection.M4.X = (left + right) / -width;
            projection.M4.Y = (top + bottom) / -height;
            projection.M4.Z = zNear / -depth;
            //projection.1/*M44*/ = 1f;

        }

        /// <summary>
        /// Creates a right-handed perspective matrix.
        /// </summary>
        /// <param name="fieldOfView">Field of view of the perspective in radians.</param>
        /// <param name="aspectRatio">Width of the viewport over the height of the viewport.</param>
        /// <param name="nearClip">Near clip plane of the perspective.</param>
        /// <param name="farClip">Far clip plane of the perspective.</param>
        /// <param name="perspective">Resulting perspective matrix.</param>
        public static void CreatePerspectiveFieldOfViewRH(float fieldOfView, float aspectRatio, float nearClip, float farClip, out Matrix perspective)
        {
            float h = 1f / ((float)Math.Tan(fieldOfView * 0.5f));
            float w = h / aspectRatio;
            perspective.M1.X = w;
            perspective.M1.Y = 0;
            perspective.M1.Z = 0;
            //perspective.0/*M14*/ = 0;

            perspective.M2.X = 0;
            perspective.M2.Y = h;
            perspective.M2.Z = 0;
            //perspective.0/*M24*/ = 0;

            perspective.M3.X = 0;
            perspective.M3.Y = 0;
            perspective.M3.Z = farClip / (nearClip - farClip);
            //perspective.0/*M34*/ = -1;

            perspective.M4.X = 0;
            perspective.M4.Y = 0;
            //perspective.1/*M44*/ = 0;
            perspective.M4.Z = nearClip * perspective.M3.Z;

        }

        /// <summary>
        /// Creates a right-handed perspective matrix.
        /// </summary>
        /// <param name="fieldOfView">Field of view of the perspective in radians.</param>
        /// <param name="aspectRatio">Width of the viewport over the height of the viewport.</param>
        /// <param name="nearClip">Near clip plane of the perspective.</param>
        /// <param name="farClip">Far clip plane of the perspective.</param>
        /// <returns>Resulting perspective matrix.</returns>
        public static Matrix CreatePerspectiveFieldOfViewRH(float fieldOfView, float aspectRatio, float nearClip, float farClip)
        {
            Matrix perspective;
            CreatePerspectiveFieldOfViewRH(fieldOfView, aspectRatio, nearClip, farClip, out perspective);
            return perspective;
        }

        /// <summary>
        /// Creates a view matrix pointing from a position to a target with the given up vector.
        /// </summary>
        /// <param name="position">Position of the camera.</param>
        /// <param name="target">Target of the camera.</param>
        /// <param name="upVector">Up vector of the camera.</param>
        /// <param name="viewMatrix">Look at matrix.</param>
        public static void CreateLookAtRH(ref Vector3 position, ref Vector3 target, ref Vector3 upVector, out Matrix viewMatrix)
        {
            Vector3 forward;
            Vector3.Subtract(ref target, ref position, out forward);
            CreateViewRH(ref position, ref forward, ref upVector, out viewMatrix);
        }

        /// <summary>
        /// Creates a view matrix pointing from a position to a target with the given up vector.
        /// </summary>
        /// <param name="position">Position of the camera.</param>
        /// <param name="target">Target of the camera.</param>
        /// <param name="upVector">Up vector of the camera.</param>
        /// <returns>Look at matrix.</returns>
        public static Matrix CreateLookAtRH(Vector3 position, Vector3 target, Vector3 upVector)
        {
            Matrix lookAt;
            Vector3 forward;
            Vector3.Subtract(ref target, ref position, out forward);
            CreateViewRH(ref position, ref forward, ref upVector, out lookAt);
            return lookAt;
        }


        /// <summary>
        /// Creates a view matrix pointing looking in a direction with a given up vector.
        /// </summary>
        /// <param name="position">Position of the camera.</param>
        /// <param name="forward">Forward direction of the camera.</param>
        /// <param name="upVector">Up vector of the camera.</param>
        /// <param name="viewMatrix">Look at matrix.</param>
        public static void CreateViewRH(ref Vector3 position, ref Vector3 forward, ref Vector3 upVector, out Matrix viewMatrix)
        {
            Vector3 z;
            float length = forward.Length();
            Vector3.Divide(ref forward, -length, out z);
            Vector3 x;
            Vector3.Cross(ref upVector, ref z, out x);
            x.Normalize();
            Vector3 y;
            Vector3.Cross(ref z, ref x, out y);

            viewMatrix.M1.X = x.X;
            viewMatrix.M1.Y = y.X;
            viewMatrix.M1.Z = z.X;
            //viewMatrix.0/*M14*/ = 0f;
            viewMatrix.M2.X = x.Y;
            viewMatrix.M2.Y = y.Y;
            viewMatrix.M2.Z = z.Y;
            //viewMatrix.0/*M24*/ = 0f;
            viewMatrix.M3.X = x.Z;
            viewMatrix.M3.Y = y.Z;
            viewMatrix.M3.Z = z.Z;
            //viewMatrix.0/*M34*/ = 0f;
            Vector3.Dot(ref x, ref position, out viewMatrix.M4.X);
            Vector3.Dot(ref y, ref position, out viewMatrix.M4.Y);
            Vector3.Dot(ref z, ref position, out viewMatrix.M4.Z);
            viewMatrix.M4.X = -viewMatrix.M4.X;
            viewMatrix.M4.Y = -viewMatrix.M4.Y;
            viewMatrix.M4.Z = -viewMatrix.M4.Z;
            //viewMatrix.1/*M44*/ = 1f;

        }

        /// <summary>
        /// Creates a view matrix pointing looking in a direction with a given up vector.
        /// </summary>
        /// <param name="position">Position of the camera.</param>
        /// <param name="forward">Forward direction of the camera.</param>
        /// <param name="upVector">Up vector of the camera.</param>
        /// <returns>Look at matrix.</returns>
        public static Matrix CreateViewRH(Vector3 position, Vector3 forward, Vector3 upVector)
        {
            Matrix lookat;
            CreateViewRH(ref position, ref forward, ref upVector, out lookat);
            return lookat;
        }



        /// <summary>
        /// Creates a world matrix pointing from a position to a target with the given up vector.
        /// </summary>
        /// <param name="position">Position of the transform.</param>
        /// <param name="forward">Forward direction of the transformation.</param>
        /// <param name="upVector">Up vector which is crossed against the forward vector to compute the transform's basis.</param>
        /// <param name="worldMatrix">World matrix.</param>
        public static void CreateWorldRH(ref Vector3 position, ref Vector3 forward, ref Vector3 upVector, out Matrix worldMatrix)
        {
            Vector3 z;
            float length = forward.Length();
            Vector3.Divide(ref forward, -length, out z);
            Vector3 x;
            Vector3.Cross(ref upVector, ref z, out x);
            x.Normalize();
            Vector3 y;
            Vector3.Cross(ref z, ref x, out y);

            worldMatrix.M1.X = x.X;
            worldMatrix.M1.Y = x.Y;
            worldMatrix.M1.Z = x.Z;
            //worldMatrix.0/*M14*/ = 0f;
            worldMatrix.M2.X = y.X;
            worldMatrix.M2.Y = y.Y;
            worldMatrix.M2.Z = y.Z;
            //worldMatrix.0/*M24*/ = 0f;
            worldMatrix.M3.X = z.X;
            worldMatrix.M3.Y = z.Y;
            worldMatrix.M3.Z = z.Z;
            //worldMatrix.0/*M34*/ = 0f;

            worldMatrix.M4.X = position.X;
            worldMatrix.M4.Y = position.Y;
            worldMatrix.M4.Z = position.Z;
            //worldMatrix.1/*M44*/ = 1f;

        }


        /// <summary>
        /// Creates a world matrix pointing from a position to a target with the given up vector.
        /// </summary>
        /// <param name="position">Position of the transform.</param>
        /// <param name="forward">Forward direction of the transformation.</param>
        /// <param name="upVector">Up vector which is crossed against the forward vector to compute the transform's basis.</param>
        /// <returns>World matrix.</returns>
        public static Matrix CreateWorldRH(Vector3 position, Vector3 forward, Vector3 upVector)
        {
            Matrix lookat;
            CreateWorldRH(ref position, ref forward, ref upVector, out lookat);
            return lookat;
        }



        /// <summary>
        /// Creates a matrix representing a translation.
        /// </summary>
        /// <param name="translation">Translation to be represented by the matrix.</param>
        /// <param name="translationMatrix">Matrix representing the given translation.</param>
        public static void CreateTranslation(ref Vector3 translation, out Matrix translationMatrix)
        {
            translationMatrix = new Matrix();
            {
                translationMatrix.M1.X = 1;
                translationMatrix.M2.Y = 1;
                translationMatrix.M3.Z = 1;
                //1/*M44*/ = 1;
                translationMatrix.M4.X = translation.X;
                translationMatrix.M4.Y = translation.Y;
                translationMatrix.M4.Z = translation.Z;
            };
        }

        /// <summary>
        /// Creates a matrix representing a translation.
        /// </summary>
        /// <param name="translation">Translation to be represented by the matrix.</param>
        /// <returns>Matrix representing the given translation.</returns>
        public static Matrix CreateTranslation(Vector3 translation)
        {
            Matrix translationMatrix;
            CreateTranslation(ref translation, out translationMatrix);
            return translationMatrix;
        }

        /// <summary>
        /// Creates a matrix representing the given axis aligned scale.
        /// </summary>
        /// <param name="scale">Scale to be represented by the matrix.</param>
        /// <param name="scaleMatrix">Matrix representing the given scale.</param>
        public static void CreateScale(ref Vector3 scale, out Matrix scaleMatrix)
        {
            scaleMatrix = new Matrix();
            {
                scaleMatrix.M1.X = scale.X;
                scaleMatrix.M2.Y = scale.Y;
                scaleMatrix.M3.Z = scale.Z;
                //1/*M44*/ = 1
            };
        }

        /// <summary>
        /// Creates a matrix representing the given axis aligned scale.
        /// </summary>
        /// <param name="scale">Scale to be represented by the matrix.</param>
        /// <returns>Matrix representing the given scale.</returns>
        public static Matrix CreateScale(Vector3 scale)
        {
            Matrix scaleMatrix;
            CreateScale(ref scale, out scaleMatrix);
            return scaleMatrix;
        }

        /// <summary>
        /// Creates a matrix representing the given axis aligned scale.
        /// </summary>
        /// <param name="x">Scale along the x axis.</param>
        /// <param name="y">Scale along the y axis.</param>
        /// <param name="z">Scale along the z axis.</param>
        /// <param name="scaleMatrix">Matrix representing the given scale.</param>
        public static void CreateScale(float x, float y, float z, out Matrix scaleMatrix)
        {
            scaleMatrix = new Matrix();
            {
                scaleMatrix.M1.X = x;
                scaleMatrix.M2.Y = y;
                scaleMatrix.M3.Z = z;
                //scaleMatrix.1/*M44*/ = 1;
            };
        }

        /// <summary>
        /// Creates a matrix representing the given axis aligned scale.
        /// </summary>
        /// <param name="x">Scale along the x axis.</param>
        /// <param name="y">Scale along the y axis.</param>
        /// <param name="z">Scale along the z axis.</param>
        /// <returns>Matrix representing the given scale.</returns>
        public static Matrix CreateScale(float x, float y, float z)
        {
            Matrix scaleMatrix;
            CreateScale(x, y, z, out scaleMatrix);
            return scaleMatrix;
        }

		/// <summary>
		/// Move to a specific place in space
		/// </summary>
		/// <param name="x"></param>
		/// <param name="y"></param>
		/// <param name="z"></param>
		public void MoveTo(float x, float y, float z)
		{
			M4.X = x;
			M4.Y = y;
			M4.Z = z;
		}

		public void MoveUp(float delta)
		{
			M4.X -= M2.X * delta;
			M4.Y -= M2.Y * delta;
			M4.Z -= M2.Z * delta;
		}
		public void MoveDown(float delta)
		{
			M4.X += M2.X * delta;
			M4.Y += M2.Y * delta;
			M4.Z += M2.Z * delta;
		}
		public void MoveRight(float delta)
		{
			M4.X -= M1.X * delta;
			M4.Y -= M1.Y * delta;
			M4.Z -= M1.Z * delta;
		}
		public void MoveLeft(float delta)
		{
			M4.X += M1.X * delta;
			M4.Y += M1.Y * delta;
			M4.Z += M1.Z * delta;
		}
		public void MoveForward(float delta)
		{
			M4.X += M3.X * delta;
			M4.Y += M3.Y * delta;
			M4.Z += M3.Z * delta;
		}
		public void MoveBackward(float delta)
		{
			M4.X -= M3.X * delta;
			M4.Y -= M3.Y * delta;
			M4.Z -= M3.Z * delta;
		}

		public void GetGLMatrix(float[] m)
		{
			m[0] = M1.X;
			m[1] = M1.Y;
			m[2] = M1.Z;
			m[3] = 0;
			m[4] = M2.X;
			m[5] = M2.Y;
			m[6] = M2.Z;
			m[7] = 0;
			m[8] = M3.X;
			m[9] = M3.Y;
			m[10] = M3.Z;
			m[11] = 0;
			Vector3 o = new Vector3(M4.X, M4.Y, M4.Z);
			Vector3 ot;
			//TransformTranspose( ref o, ref this, out ot );
			TransformNormal(ref o, ref this, out ot);
			m[12] = ot.X;
			m[13] = ot.Y;
			m[14] = ot.Z;
			m[15] = 1;
		}

		public void GetGLCameraMatrix(float[] m)
		{
			m[0] = M1.X;
			m[1] = M2.X;
			m[2] = M3.X;
			m[3] = 0;
			m[4] = M1.Y;
			m[5] = M2.Y;
			m[6] = M3.Y;
			m[7] = 0;
			m[8] = M1.Z;
			m[9] = M2.Z;
			m[10] = M3.Z;
			m[11] = 0;
			Vector3 o = new Vector3(M4.X, M4.Y, M4.Z);
			Vector3 ot;
			//TransformTranspose( ref o, ref this, out ot );
			TransformNormalTranspose(ref o, ref this, out ot);
			m[12] = ot.X;
			m[13] = ot.Y;
			m[14] = ot.Z;
			m[15] = 1;
		}

		public void Rotate(int axis, float angle)
		{
			float dsin = (float)Math.Sin(angle)
				  , dcos = (float)Math.Cos(angle);
			switch (axis)
			{
				default:
				case 0:
					{
						float savex = M2.X;
						float savey = M2.Y;
						float savez = M2.Z;
						float v1x, v2x;
						float v1y, v2y;
						float v1z, v2z;
						v1x = M2.X * dcos;
						v1y = M2.Y * dcos;
						v1z = M2.Z * dcos;
						v2x = M3.X * dsin;
						v2y = M3.Y * dsin;
						v2z = M3.Z * dsin;
						M2.X = v1x - v2x;
						M2.Y = v1y - v2y;
						M2.Z = v1z - v2z;

						v2x = savex * dsin;
						v2y = savey * dsin;
						v2z = savez * dsin;
						v1x = M3.X * dcos;
						v1y = M3.Y * dcos;
						v1z = M3.Z * dcos;
						M3.X = v1x + v2x;
						M3.Y = v1y + v2y;
						M3.Z = v1z + v2z;
						break;
					}
				case 1:
					{
						float savex = M1.X;
						float savey = M1.Y;
						float savez = M1.Z;
						float v1x, v2x;
						float v1y, v2y;
						float v1z, v2z;
						v1x = M1.X * dcos;
						v1y = M1.Y * dcos;
						v1z = M1.Z * dcos;
						v2x = M3.X * dsin;
						v2y = M3.Y * dsin;
						v2z = M3.Z * dsin;
						M1.X = v1x - v2x;
						M1.Y = v1y - v2y;
						M1.Z = v1z - v2z;

						v2x = savex * dsin;
						v2y = savey * dsin;
						v2z = savez * dsin;
						v1x = M3.X * dcos;
						v1y = M3.Y * dcos;
						v1z = M3.Z * dcos;
						M3.X = v1x + v2x;
						M3.Y = v1y + v2y;
						M3.Z = v1z + v2z;
						break;
					}
				case 2:
					{
						float savex = M1.X;
						float savey = M1.Y;
						float savez = M1.Z;
						float v1x, v2x;
						float v1y, v2y;
						float v1z, v2z;
						v1x = M1.X * dcos;
						v1y = M1.Y * dcos;
						v1z = M1.Z * dcos;
						v2x = M2.X * dsin;
						v2y = M2.Y * dsin;
						v2z = M2.Z * dsin;
						M1.X = v1x - v2x;
						M1.Y = v1y - v2y;
						M1.Z = v1z - v2z;

						v2x = savex * dsin;
						v2y = savey * dsin;
						v2z = savez * dsin;
						v1x = M2.X * dcos;
						v1y = M2.Y * dcos;
						v1z = M2.Z * dcos;
						M2.X = v1x + v2x;
						M2.Y = v1y + v2y;
						M2.Z = v1z + v2z;
						break;
					}
			}

		}

		public void RotatePitch(float delta_angle)
		{
			Rotate(0, -delta_angle);
		}
		public void RotateYaw(float delta_angle)
		{
			Rotate(1, -delta_angle);
		}
		public void RotateRoll(float delta_angle)
		{
			Rotate(2, delta_angle);
		}

		/// <summary>
		/// Creates a string representation of the matrix.
		/// </summary>
		/// <returns>A string representation of the matrix.</returns>
		public override string ToString()
        {
            return "{" + M1.X + ", " + M1.Y + ", " + M1.Z + ", " + "0" + "} " +
                   "{" + M2.X + ", " + M2.Y + ", " + M2.Z + ", " + "0" + "} " +
                   "{" + M3.X + ", " + M3.Y + ", " + M3.Z + ", " + "0" + "} " +
                   "{" + M4.X + ", " + M4.Y + ", " + M4.Z + ", " + "1" + "}";
        }
		public string ToString(string name, string leader, string origin)
		{
			return String.Format("{0}({1},{2},{3},{4})\n{5}({6},{7},{8},{9})\n{5}({10},{11},{12},{13})\n{14}({15},{16},{17})"
				, name, M1.X, M1.Y, M1.Z, 0
				, leader, M2.X, M2.Y, M2.Z, 0
				, M3.X, M3.Y, M3.Z, 0
				, origin, M4.X, M4.Y, M4.Z
				);
		}
	}
}
