using System;



namespace BEPUutilities
{
    /// <summary>
    /// 3 row, 3 column matrix.
    /// </summary>
    public struct Matrix3x3
    {
        public Vector3 M1;
        public Vector3 M2;
        public Vector3 M3;
        /// <summary>
        /// Value at row 1, column 1 of the matrix.
        /// </summary>
        public float M11 { get { return M1.X; } }

        /// <summary>
        /// Value at row 1, column 2 of the matrix.
        /// </summary>
        public float M12 { get { return M1.Y; } }

        /// <summary>
        /// Value at row 1, column 3 of the matrix.
        /// </summary>
        public float M13 { get { return M1.Z; } }

        /// <summary>
        /// Value at row 2, column 1 of the matrix.
        /// </summary>
        public float M21 { get { return M2.X; } }

        /// <summary>
        /// Value at row 2, column 2 of the matrix.
        /// </summary>
        public float M22 { get { return M2.Y; } }

        /// <summary>
        /// Value at row 2, column 3 of the matrix.
        /// </summary>
        public float M23 { get { return M2.Z; } }

        /// <summary>
        /// Value at row 3, column 1 of the matrix.
        /// </summary>
        public float M31 { get { return M3.X; } }

        /// <summary>
        /// Value at row 3, column 2 of the matrix.
        /// </summary>
        public float M32 { get { return M3.Y; } }

        /// <summary>
        /// Value at row 3, column 3 of the matrix.
        /// </summary>
        public float M33 { get { return M3.Z; } }


        /// <summary>
        /// Constructs a new 3 row, 3 column matrix.
        /// </summary>
        /// <param name="m11">Value at row 1, column 1 of the matrix.</param>
        /// <param name="m12">Value at row 1, column 2 of the matrix.</param>
        /// <param name="m13">Value at row 1, column 3 of the matrix.</param>
        /// <param name="m21">Value at row 2, column 1 of the matrix.</param>
        /// <param name="m22">Value at row 2, column 2 of the matrix.</param>
        /// <param name="m23">Value at row 2, column 3 of the matrix.</param>
        /// <param name="m31">Value at row 3, column 1 of the matrix.</param>
        /// <param name="m32">Value at row 3, column 2 of the matrix.</param>
        /// <param name="m33">Value at row 3, column 3 of the matrix.</param>
        public Matrix3x3(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33)
        {
            M1.X = m11;
            M1.Y = m12;
            M1.Z = m13;
            M2.X = m21;
            M2.Y = m22;
            M2.Z = m23;
            M3.X = m31;
            M3.Y = m32;
            M3.Z = m33;
        }

        /// <summary>
        /// Set just the scaling parts of the marix
        /// </summary>
        /// <param name="sx"></param>
        /// <param name="sy"></param>
        /// <param name="sz"></param>
        public Matrix3x3(float sx, float sy, float sz)
        {
            M1.X = sx;
            M1.Y = 0;
            M1.Z = 0;
            M2.X = 0;
            M2.Y = sy;
            M2.Z = 0;
            M3.X = 0;
            M3.Y = 0;
            M3.Z = sz;
        }

        /// <summary>
        /// Gets the 3x3 identity matrix.
        /// </summary>
        public static Matrix3x3 Identity = new Matrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
        /// <summary>
        /// Gets the 3x3 identity matrix.
        /// </summary>
        public static Matrix3x3 InverseIdentity = new Matrix3x3(-1, 0, 0, 0, -1, 0, 0, 0, -1);


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
        /// Adds the two matrices together on a per-element basis.
        /// </summary>
        /// <param name="a">First matrix to add.</param>
        /// <param name="b">Second matrix to add.</param>
        /// <param name="result">Sum of the two matrices.</param>
        public static void Add(ref Matrix3x3 a, ref Matrix3x3 b, out Matrix3x3 result)
        {
            float m11 = a.M1.X + b.M1.X;
            float m12 = a.M1.Y + b.M1.Y;
            float m13 = a.M1.Z + b.M1.Z;

            float m21 = a.M2.X + b.M2.X;
            float m22 = a.M2.Y + b.M2.Y;
            float m23 = a.M2.Z + b.M2.Z;

            float m31 = a.M3.X + b.M3.X;
            float m32 = a.M3.Y + b.M3.Y;
            float m33 = a.M3.Z + b.M3.Z;

            result.M1.X = m11;
            result.M1.Y = m12;
            result.M1.Z = m13;

            result.M2.X = m21;
            result.M2.Y = m22;
            result.M2.Z = m23;

            result.M3.X = m31;
            result.M3.Y = m32;
            result.M3.Z = m33;
        }

        /// <summary>
        /// Adds the two matrices together on a per-element basis.
        /// </summary>
        /// <param name="a">First matrix to add.</param>
        /// <param name="b">Second matrix to add.</param>
        /// <param name="result">Sum of the two matrices.</param>
        public static void Add(ref Matrix a, ref Matrix3x3 b, out Matrix3x3 result)
        {
            float m11 = a.M1.X + b.M1.X;
            float m12 = a.M1.Y + b.M1.Y;
            float m13 = a.M1.Z + b.M1.Z;

            float m21 = a.M2.X + b.M2.X;
            float m22 = a.M2.Y + b.M2.Y;
            float m23 = a.M2.Z + b.M2.Z;

            float m31 = a.M3.X + b.M3.X;
            float m32 = a.M3.Y + b.M3.Y;
            float m33 = a.M3.Z + b.M3.Z;

            result.M1.X = m11;
            result.M1.Y = m12;
            result.M1.Z = m13;

            result.M2.X = m21;
            result.M2.Y = m22;
            result.M2.Z = m23;

            result.M3.X = m31;
            result.M3.Y = m32;
            result.M3.Z = m33;
        }

        /// <summary>
        /// Adds the two matrices together on a per-element basis.
        /// </summary>
        /// <param name="a">First matrix to add.</param>
        /// <param name="b">Second matrix to add.</param>
        /// <param name="result">Sum of the two matrices.</param>
        public static void Add(ref Matrix3x3 a, ref Matrix b, out Matrix3x3 result)
        {
            float m11 = a.M1.X + b.M1.X;
            float m12 = a.M1.Y + b.M1.Y;
            float m13 = a.M1.Z + b.M1.Z;

            float m21 = a.M2.X + b.M2.X;
            float m22 = a.M2.Y + b.M2.Y;
            float m23 = a.M2.Z + b.M2.Z;

            float m31 = a.M3.X + b.M3.X;
            float m32 = a.M3.Y + b.M3.Y;
            float m33 = a.M3.Z + b.M3.Z;

            result.M1.X = m11;
            result.M1.Y = m12;
            result.M1.Z = m13;

            result.M2.X = m21;
            result.M2.Y = m22;
            result.M2.Z = m23;

            result.M3.X = m31;
            result.M3.Y = m32;
            result.M3.Z = m33;
        }

        /// <summary>
        /// Adds the two matrices together on a per-element basis.
        /// </summary>
        /// <param name="a">First matrix to add.</param>
        /// <param name="b">Second matrix to add.</param>
        /// <param name="result">Sum of the two matrices.</param>
        public static void Add(ref Matrix a, ref Matrix b, out Matrix3x3 result)
        {
            float m11 = a.M1.X + b.M1.X;
            float m12 = a.M1.Y + b.M1.Y;
            float m13 = a.M1.Z + b.M1.Z;

            float m21 = a.M2.X + b.M2.X;
            float m22 = a.M2.Y + b.M2.Y;
            float m23 = a.M2.Z + b.M2.Z;

            float m31 = a.M3.X + b.M3.X;
            float m32 = a.M3.Y + b.M3.Y;
            float m33 = a.M3.Z + b.M3.Z;

            result.M1.X = m11;
            result.M1.Y = m12;
            result.M1.Z = m13;

            result.M2.X = m21;
            result.M2.Y = m22;
            result.M2.Z = m23;

            result.M3.X = m31;
            result.M3.Y = m32;
            result.M3.Z = m33;
        }

        /// <summary>
        /// Creates a skew symmetric matrix M from vector A such that M * B for some other vector B is equivalent to the cross product of A and B.
        /// </summary>
        /// <param name="v">Vector to base the matrix on.</param>
        /// <param name="result">Skew-symmetric matrix result.</param>
        public static void CreateCrossProduct(ref Vector3 v, out Matrix3x3 result)
        {
            result.M1.X = 0;
            result.M1.Y = -v.Z;
            result.M1.Z = v.Y;
            result.M2.X = v.Z;
            result.M2.Y = 0;
            result.M2.Z = -v.X;
            result.M3.X = -v.Y;
            result.M3.Y = v.X;
            result.M3.Z = 0;
        }

        /// <summary>
        /// Creates a 3x3 matrix from an XNA 4x4 matrix.
        /// </summary>
        /// <param name="matrix4X4">Matrix to extract a 3x3 matrix from.</param>
        /// <param name="matrix3X3">Upper 3x3 matrix extracted from the XNA matrix.</param>
        public static void CreateFromMatrix(ref Matrix matrix4X4, out Matrix3x3 matrix3X3)
        {
            matrix3X3.M1.X = matrix4X4.M1.X;
            matrix3X3.M1.Y = matrix4X4.M1.Y;
            matrix3X3.M1.Z = matrix4X4.M1.Z;

            matrix3X3.M2.X = matrix4X4.M2.X;
            matrix3X3.M2.Y = matrix4X4.M2.Y;
            matrix3X3.M2.Z = matrix4X4.M2.Z;

            matrix3X3.M3.X = matrix4X4.M3.X;
            matrix3X3.M3.Y = matrix4X4.M3.Y;
            matrix3X3.M3.Z = matrix4X4.M3.Z;
        }
        /// <summary>
        /// Creates a 3x3 matrix from an XNA 4x4 matrix.
        /// </summary>
        /// <param name="matrix4X4">Matrix to extract a 3x3 matrix from.</param>
        /// <returns>Upper 3x3 matrix extracted from the XNA matrix.</returns>
        public static Matrix3x3 CreateFromMatrix(Matrix matrix4X4)
        {
            Matrix3x3 matrix3X3;
            matrix3X3.M1.X = matrix4X4.M1.X;
            matrix3X3.M1.Y = matrix4X4.M1.Y;
            matrix3X3.M1.Z = matrix4X4.M1.Z;

            matrix3X3.M2.X = matrix4X4.M2.X;
            matrix3X3.M2.Y = matrix4X4.M2.Y;
            matrix3X3.M2.Z = matrix4X4.M2.Z;

            matrix3X3.M3.X = matrix4X4.M3.X;
            matrix3X3.M3.Y = matrix4X4.M3.Y;
            matrix3X3.M3.Z = matrix4X4.M3.Z;
            return matrix3X3;
        }

        /// <summary>
        /// Constructs a uniform scaling matrix.
        /// </summary>
        /// <param name="scale">Value to use in the diagonal.</param>
        /// <param name="matrix">Scaling matrix.</param>
        public static void CreateScale(float scale, out Matrix3x3 matrix)
        {
            matrix = new Matrix3x3();
            { matrix.M1.X = scale; matrix.M2.Y = scale; matrix.M3.Z = scale; }
        }

        /// <summary>
        /// Constructs a uniform scaling matrix.
        /// </summary>
        /// <param name="scale">Value to use in the diagonal.</param>
        /// <returns>Scaling matrix.</returns>
        public static Matrix3x3 CreateScale(float scale)
        {
            var matrix = new Matrix3x3();
            { matrix.M1.X = scale; matrix.M2.Y = scale; matrix.M3.Z = scale; }
            return matrix;
        }

        /// <summary>
        /// Constructs a non-uniform scaling matrix.
        /// </summary>
        /// <param name="scale">Values defining the axis scales.</param>
        /// <param name="matrix">Scaling matrix.</param>
        public static void CreateScale(ref Vector3 scale, out Matrix3x3 matrix)
        {
            matrix = new Matrix3x3(scale.X, scale.Y, scale.Z);
        }

        /// <summary>
        /// Constructs a non-uniform scaling matrix.
        /// </summary>
        /// <param name="scale">Values defining the axis scales.</param>
        /// <returns>Scaling matrix.</returns>
        public static Matrix3x3 CreateScale(ref Vector3 scale)
        {
            var matrix = new Matrix3x3(scale.X, scale.Y, scale.Z);
            return matrix;
        }


        /// <summary>
        /// Constructs a non-uniform scaling matrix.
        /// </summary>
        /// <param name="x">Scaling along the x axis.</param>
        /// <param name="y">Scaling along the y axis.</param>
        /// <param name="z">Scaling along the z axis.</param>
        /// <param name="matrix">Scaling matrix.</param>
        public static void CreateScale(float x, float y, float z, out Matrix3x3 matrix)
        {
            matrix = new Matrix3x3(x, y, z);
        }

        /// <summary>
        /// Constructs a non-uniform scaling matrix.
        /// </summary>
        /// <param name="x">Scaling along the x axis.</param>
        /// <param name="y">Scaling along the y axis.</param>
        /// <param name="z">Scaling along the z axis.</param>
        /// <returns>Scaling matrix.</returns>
        public static Matrix3x3 CreateScale(float x, float y, float z)
        {
            var matrix = new Matrix3x3(x, y, z);
            return matrix;
        }

        /// <summary>
        /// Inverts the given matix.
        /// </summary>
        /// <param name="matrix">Matrix to be inverted.</param>
        /// <param name="result">Inverted matrix.</param>
        public static void Invert(ref Matrix3x3 matrix, out Matrix3x3 result)
        {
            float determinantInverse = 1 / matrix.Determinant();
            float m11 = (matrix.M2.Y * matrix.M3.Z - matrix.M2.Z * matrix.M3.Y) * determinantInverse;
            float m12 = (matrix.M1.Z * matrix.M3.Y - matrix.M3.Z * matrix.M1.Y) * determinantInverse;
            float m13 = (matrix.M1.Y * matrix.M2.Z - matrix.M2.Y * matrix.M1.Z) * determinantInverse;

            float m21 = (matrix.M2.Z * matrix.M3.X - matrix.M2.X * matrix.M3.Z) * determinantInverse;
            float m22 = (matrix.M1.X * matrix.M3.Z - matrix.M1.Z * matrix.M3.X) * determinantInverse;
            float m23 = (matrix.M1.Z * matrix.M2.X - matrix.M1.X * matrix.M2.Z) * determinantInverse;

            float m31 = (matrix.M2.X * matrix.M3.Y - matrix.M2.Y * matrix.M3.X) * determinantInverse;
            float m32 = (matrix.M1.Y * matrix.M3.X - matrix.M1.X * matrix.M3.Y) * determinantInverse;
            float m33 = (matrix.M1.X * matrix.M2.Y - matrix.M1.Y * matrix.M2.X) * determinantInverse;

            result.M1.X = m11;
            result.M1.Y = m12;
            result.M1.Z = m13;

            result.M2.X = m21;
            result.M2.Y = m22;
            result.M2.Z = m23;

            result.M3.X = m31;
            result.M3.Y = m32;
            result.M3.Z = m33;
        }

        /// <summary>
        /// Inverts the given matix.
        /// </summary>
        /// <param name="matrix">Matrix to be inverted.</param>
        /// <returns>Inverted matrix.</returns>
        public static Matrix3x3 Invert(Matrix3x3 matrix)
        {
            Matrix3x3 toReturn;
            Invert(ref matrix, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Inverts the largest nonsingular submatrix in the matrix, excluding 2x2's that involve M1.Z or M3.X, and excluding 1x1's that include nondiagonal elements.
        /// </summary>
        /// <param name="matrix">Matrix to be inverted.</param>
        /// <param name="result">Inverted matrix.</param>
        public static void AdaptiveInvert(ref Matrix3x3 matrix, out Matrix3x3 result)
        {
            int submatrix;
            float determinantInverse = 1 / matrix.AdaptiveDeterminant(out submatrix);
            float m11, m12, m13, m21, m22, m23, m31, m32, m33;
            switch (submatrix)
            {
                case 0: //Full matrix.
                    m11 = (matrix.M2.Y * matrix.M3.Z - matrix.M2.Z * matrix.M3.Y) * determinantInverse;
                    m12 = (matrix.M1.Z * matrix.M3.Y - matrix.M3.Z * matrix.M1.Y) * determinantInverse;
                    m13 = (matrix.M1.Y * matrix.M2.Z - matrix.M2.Y * matrix.M1.Z) * determinantInverse;

                    m21 = (matrix.M2.Z * matrix.M3.X - matrix.M2.X * matrix.M3.Z) * determinantInverse;
                    m22 = (matrix.M1.X * matrix.M3.Z - matrix.M1.Z * matrix.M3.X) * determinantInverse;
                    m23 = (matrix.M1.Z * matrix.M2.X - matrix.M1.X * matrix.M2.Z) * determinantInverse;

                    m31 = (matrix.M2.X * matrix.M3.Y - matrix.M2.Y * matrix.M3.X) * determinantInverse;
                    m32 = (matrix.M1.Y * matrix.M3.X - matrix.M1.X * matrix.M3.Y) * determinantInverse;
                    m33 = (matrix.M1.X * matrix.M2.Y - matrix.M1.Y * matrix.M2.X) * determinantInverse;
                    break;
                case 1: //Upper left matrix, m11, m12, m21, m22.
                    m11 = matrix.M2.Y * determinantInverse;
                    m12 = -matrix.M1.Y * determinantInverse;
                    m13 = 0;

                    m21 = -matrix.M2.X * determinantInverse;
                    m22 = matrix.M1.X * determinantInverse;
                    m23 = 0;

                    m31 = 0;
                    m32 = 0;
                    m33 = 0;
                    break;
                case 2: //Lower right matrix, m22, m23, m32, m33.
                    m11 = 0;
                    m12 = 0;
                    m13 = 0;

                    m21 = 0;
                    m22 = matrix.M3.Z * determinantInverse;
                    m23 = -matrix.M2.Z * determinantInverse;

                    m31 = 0;
                    m32 = -matrix.M3.Y * determinantInverse;
                    m33 = matrix.M2.Y * determinantInverse;
                    break;
                case 3: //Corners, m11, m31, m13, m33.
                    m11 = matrix.M3.Z * determinantInverse;
                    m12 = 0;
                    m13 = -matrix.M1.Z * determinantInverse;

                    m21 = 0;
                    m22 = 0;
                    m23 = 0;

                    m31 = -matrix.M3.X * determinantInverse;
                    m32 = 0;
                    m33 = matrix.M1.X * determinantInverse;
                    break;
                case 4: //M1.X
                    m11 = 1 / matrix.M1.X;
                    m12 = 0;
                    m13 = 0;

                    m21 = 0;
                    m22 = 0;
                    m23 = 0;

                    m31 = 0;
                    m32 = 0;
                    m33 = 0;
                    break;
                case 5: //M2.Y
                    m11 = 0;
                    m12 = 0;
                    m13 = 0;

                    m21 = 0;
                    m22 = 1 / matrix.M2.Y;
                    m23 = 0;

                    m31 = 0;
                    m32 = 0;
                    m33 = 0;
                    break;
                case 6: //M3.Z
                    m11 = 0;
                    m12 = 0;
                    m13 = 0;

                    m21 = 0;
                    m22 = 0;
                    m23 = 0;

                    m31 = 0;
                    m32 = 0;
                    m33 = 1 / matrix.M3.Z;
                    break;
                default: //Completely singular.
                    m11 = 0; m12 = 0; m13 = 0; m21 = 0; m22 = 0; m23 = 0; m31 = 0; m32 = 0; m33 = 0;
                    break;
            }

            result.M1.X = m11;
            result.M1.Y = m12;
            result.M1.Z = m13;

            result.M2.X = m21;
            result.M2.Y = m22;
            result.M2.Z = m23;

            result.M3.X = m31;
            result.M3.Y = m32;
            result.M3.Z = m33;
        }

        /// <summary>
        /// <para>Computes the adjugate transpose of a matrix.</para>
        /// <para>The adjugate transpose A of matrix M is: det(M) * transpose(invert(M))</para>
        /// <para>This is necessary when transforming normals (bivectors) with general linear transformations.</para>
        /// </summary>
        /// <param name="matrix">Matrix to compute the adjugate transpose of.</param>
        /// <param name="result">Adjugate transpose of the input matrix.</param>
        public static void AdjugateTranspose(ref Matrix3x3 matrix, out Matrix3x3 result)
        {
            //Despite the relative obscurity of the operation, this is a fairly straightforward operation which is actually faster than a true invert (by virtue of cancellation).
            //Conceptually, this is implemented as transpose(det(M) * invert(M)), but that's perfectly acceptable:
            //1) transpose(invert(M)) == invert(transpose(M)), and
            //2) det(M) == det(transpose(M))
            //This organization makes it clearer that the invert's usual division by determinant drops out.

            float m11 = (matrix.M2.Y * matrix.M3.Z - matrix.M2.Z * matrix.M3.Y);
            float m12 = (matrix.M1.Z * matrix.M3.Y - matrix.M3.Z * matrix.M1.Y);
            float m13 = (matrix.M1.Y * matrix.M2.Z - matrix.M2.Y * matrix.M1.Z);

            float m21 = (matrix.M2.Z * matrix.M3.X - matrix.M2.X * matrix.M3.Z);
            float m22 = (matrix.M1.X * matrix.M3.Z - matrix.M1.Z * matrix.M3.X);
            float m23 = (matrix.M1.Z * matrix.M2.X - matrix.M1.X * matrix.M2.Z);

            float m31 = (matrix.M2.X * matrix.M3.Y - matrix.M2.Y * matrix.M3.X);
            float m32 = (matrix.M1.Y * matrix.M3.X - matrix.M1.X * matrix.M3.Y);
            float m33 = (matrix.M1.X * matrix.M2.Y - matrix.M1.Y * matrix.M2.X);

            //Note transposition.
            result.M1.X = m11;
            result.M1.Y = m21;
            result.M1.Z = m31;

            result.M2.X = m12;
            result.M2.Y = m22;
            result.M2.Z = m32;

            result.M3.X = m13;
            result.M3.Y = m23;
            result.M3.Z = m33;
        }

        /// <summary>
        /// <para>Computes the adjugate transpose of a matrix.</para>
        /// <para>The adjugate transpose A of matrix M is: det(M) * transpose(invert(M))</para>
        /// <para>This is necessary when transforming normals (bivectors) with general linear transformations.</para>
        /// </summary>
        /// <param name="matrix">Matrix to compute the adjugate transpose of.</param>
        /// <returns>Adjugate transpose of the input matrix.</returns>
        public static Matrix3x3 AdjugateTranspose(Matrix3x3 matrix)
        {
            Matrix3x3 toReturn;
            AdjugateTranspose(ref matrix, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Multiplies the two matrices.
        /// </summary>
        /// <param name="a">First matrix to multiply.</param>
        /// <param name="b">Second matrix to multiply.</param>
        /// <returns>Product of the multiplication.</returns>
        public static Matrix3x3 operator *(Matrix3x3 a, Matrix3x3 b)
        {
            Matrix3x3 result;
            Matrix3x3.Multiply(ref a, ref b, out result);
            return result;
        }

        /// <summary>
        /// Scales all components of the matrix by the given value.
        /// </summary>
        /// <param name="m">First matrix to multiply.</param>
        /// <param name="f">Scaling value to apply to all components of the matrix.</param>
        /// <returns>Product of the multiplication.</returns>
        public static Matrix3x3 operator *(Matrix3x3 m, float f)
        {
            Matrix3x3 result;
            Multiply(ref m, f, out result);
            return result;
        }

        /// <summary>
        /// Scales all components of the matrix by the given value.
        /// </summary>
        /// <param name="m">First matrix to multiply.</param>
        /// <param name="f">Scaling value to apply to all components of the matrix.</param>
        /// <returns>Product of the multiplication.</returns>
        public static Matrix3x3 operator *(float f, Matrix3x3 m)
        {
            Matrix3x3 result;
            Multiply(ref m, f, out result);
            return result;
        }

        /// <summary>
        /// Multiplies the two matrices.
        /// </summary>
        /// <param name="a">First matrix to multiply.</param>
        /// <param name="b">Second matrix to multiply.</param>
        /// <param name="result">Product of the multiplication.</param>
        public static void Multiply(ref Matrix3x3 a, ref Matrix3x3 b, out Matrix3x3 result)
        {
            float resultM11 = a.M1.X * b.M1.X + a.M1.Y * b.M2.X + a.M1.Z * b.M3.X;
            float resultM12 = a.M1.X * b.M1.Y + a.M1.Y * b.M2.Y + a.M1.Z * b.M3.Y;
            float resultM13 = a.M1.X * b.M1.Z + a.M1.Y * b.M2.Z + a.M1.Z * b.M3.Z;

            float resultM21 = a.M2.X * b.M1.X + a.M2.Y * b.M2.X + a.M2.Z * b.M3.X;
            float resultM22 = a.M2.X * b.M1.Y + a.M2.Y * b.M2.Y + a.M2.Z * b.M3.Y;
            float resultM23 = a.M2.X * b.M1.Z + a.M2.Y * b.M2.Z + a.M2.Z * b.M3.Z;

            float resultM31 = a.M3.X * b.M1.X + a.M3.Y * b.M2.X + a.M3.Z * b.M3.X;
            float resultM32 = a.M3.X * b.M1.Y + a.M3.Y * b.M2.Y + a.M3.Z * b.M3.Y;
            float resultM33 = a.M3.X * b.M1.Z + a.M3.Y * b.M2.Z + a.M3.Z * b.M3.Z;

            result.M1.X = resultM11;
            result.M1.Y = resultM12;
            result.M1.Z = resultM13;

            result.M2.X = resultM21;
            result.M2.Y = resultM22;
            result.M2.Z = resultM23;

            result.M3.X = resultM31;
            result.M3.Y = resultM32;
            result.M3.Z = resultM33;
        }

        /// <summary>
        /// Multiplies the two matrices.
        /// </summary>
        /// <param name="a">First matrix to multiply.</param>
        /// <param name="b">Second matrix to multiply.</param>
        /// <param name="result">Product of the multiplication.</param>
        public static void Multiply(ref Matrix3x3 a, ref Matrix b, out Matrix3x3 result)
        {
            float resultM11 = a.M1.X * b.M1.X + a.M1.Y * b.M2.X + a.M1.Z * b.M3.X;
            float resultM12 = a.M1.X * b.M1.Y + a.M1.Y * b.M2.Y + a.M1.Z * b.M3.Y;
            float resultM13 = a.M1.X * b.M1.Z + a.M1.Y * b.M2.Z + a.M1.Z * b.M3.Z;

            float resultM21 = a.M2.X * b.M1.X + a.M2.Y * b.M2.X + a.M2.Z * b.M3.X;
            float resultM22 = a.M2.X * b.M1.Y + a.M2.Y * b.M2.Y + a.M2.Z * b.M3.Y;
            float resultM23 = a.M2.X * b.M1.Z + a.M2.Y * b.M2.Z + a.M2.Z * b.M3.Z;

            float resultM31 = a.M3.X * b.M1.X + a.M3.Y * b.M2.X + a.M3.Z * b.M3.X;
            float resultM32 = a.M3.X * b.M1.Y + a.M3.Y * b.M2.Y + a.M3.Z * b.M3.Y;
            float resultM33 = a.M3.X * b.M1.Z + a.M3.Y * b.M2.Z + a.M3.Z * b.M3.Z;

            result.M1.X = resultM11;
            result.M1.Y = resultM12;
            result.M1.Z = resultM13;

            result.M2.X = resultM21;
            result.M2.Y = resultM22;
            result.M2.Z = resultM23;

            result.M3.X = resultM31;
            result.M3.Y = resultM32;
            result.M3.Z = resultM33;
        }

        /// <summary>
        /// Multiplies the two matrices.
        /// </summary>
        /// <param name="a">First matrix to multiply.</param>
        /// <param name="b">Second matrix to multiply.</param>
        /// <param name="result">Product of the multiplication.</param>
        public static void Multiply(ref Matrix a, ref Matrix3x3 b, out Matrix3x3 result)
        {
            float resultM11 = a.M1.X * b.M1.X + a.M1.Y * b.M2.X + a.M1.Z * b.M3.X;
            float resultM12 = a.M1.X * b.M1.Y + a.M1.Y * b.M2.Y + a.M1.Z * b.M3.Y;
            float resultM13 = a.M1.X * b.M1.Z + a.M1.Y * b.M2.Z + a.M1.Z * b.M3.Z;

            float resultM21 = a.M2.X * b.M1.X + a.M2.Y * b.M2.X + a.M2.Z * b.M3.X;
            float resultM22 = a.M2.X * b.M1.Y + a.M2.Y * b.M2.Y + a.M2.Z * b.M3.Y;
            float resultM23 = a.M2.X * b.M1.Z + a.M2.Y * b.M2.Z + a.M2.Z * b.M3.Z;

            float resultM31 = a.M3.X * b.M1.X + a.M3.Y * b.M2.X + a.M3.Z * b.M3.X;
            float resultM32 = a.M3.X * b.M1.Y + a.M3.Y * b.M2.Y + a.M3.Z * b.M3.Y;
            float resultM33 = a.M3.X * b.M1.Z + a.M3.Y * b.M2.Z + a.M3.Z * b.M3.Z;

            result.M1.X = resultM11;
            result.M1.Y = resultM12;
            result.M1.Z = resultM13;

            result.M2.X = resultM21;
            result.M2.Y = resultM22;
            result.M2.Z = resultM23;

            result.M3.X = resultM31;
            result.M3.Y = resultM32;
            result.M3.Z = resultM33;
        }


        /// <summary>
        /// Multiplies a transposed matrix with another matrix.
        /// </summary>
        /// <param name="matrix">Matrix to be multiplied.</param>
        /// <param name="transpose">Matrix to be transposed and multiplied.</param>
        /// <param name="result">Product of the multiplication.</param>
        public static void MultiplyTransposed(ref Matrix3x3 transpose, ref Matrix3x3 matrix, out Matrix3x3 result)
        {
            float resultM11 = transpose.M1.X * matrix.M1.X + transpose.M2.X * matrix.M2.X + transpose.M3.X * matrix.M3.X;
            float resultM12 = transpose.M1.X * matrix.M1.Y + transpose.M2.X * matrix.M2.Y + transpose.M3.X * matrix.M3.Y;
            float resultM13 = transpose.M1.X * matrix.M1.Z + transpose.M2.X * matrix.M2.Z + transpose.M3.X * matrix.M3.Z;

            float resultM21 = transpose.M1.Y * matrix.M1.X + transpose.M2.Y * matrix.M2.X + transpose.M3.Y * matrix.M3.X;
            float resultM22 = transpose.M1.Y * matrix.M1.Y + transpose.M2.Y * matrix.M2.Y + transpose.M3.Y * matrix.M3.Y;
            float resultM23 = transpose.M1.Y * matrix.M1.Z + transpose.M2.Y * matrix.M2.Z + transpose.M3.Y * matrix.M3.Z;

            float resultM31 = transpose.M1.Z * matrix.M1.X + transpose.M2.Z * matrix.M2.X + transpose.M3.Z * matrix.M3.X;
            float resultM32 = transpose.M1.Z * matrix.M1.Y + transpose.M2.Z * matrix.M2.Y + transpose.M3.Z * matrix.M3.Y;
            float resultM33 = transpose.M1.Z * matrix.M1.Z + transpose.M2.Z * matrix.M2.Z + transpose.M3.Z * matrix.M3.Z;

            result.M1.X = resultM11;
            result.M1.Y = resultM12;
            result.M1.Z = resultM13;

            result.M2.X = resultM21;
            result.M2.Y = resultM22;
            result.M2.Z = resultM23;

            result.M3.X = resultM31;
            result.M3.Y = resultM32;
            result.M3.Z = resultM33;
        }

        /// <summary>
        /// Multiplies a matrix with a transposed matrix.
        /// </summary>
        /// <param name="matrix">Matrix to be multiplied.</param>
        /// <param name="transpose">Matrix to be transposed and multiplied.</param>
        /// <param name="result">Product of the multiplication.</param>
        public static void MultiplyByTransposed(ref Matrix3x3 matrix, ref Matrix3x3 transpose, out Matrix3x3 result)
        {
            float resultM11 = matrix.M1.X * transpose.M1.X + matrix.M1.Y * transpose.M1.Y + matrix.M1.Z * transpose.M1.Z;
            float resultM12 = matrix.M1.X * transpose.M2.X + matrix.M1.Y * transpose.M2.Y + matrix.M1.Z * transpose.M2.Z;
            float resultM13 = matrix.M1.X * transpose.M3.X + matrix.M1.Y * transpose.M3.Y + matrix.M1.Z * transpose.M3.Z;

            float resultM21 = matrix.M2.X * transpose.M1.X + matrix.M2.Y * transpose.M1.Y + matrix.M2.Z * transpose.M1.Z;
            float resultM22 = matrix.M2.X * transpose.M2.X + matrix.M2.Y * transpose.M2.Y + matrix.M2.Z * transpose.M2.Z;
            float resultM23 = matrix.M2.X * transpose.M3.X + matrix.M2.Y * transpose.M3.Y + matrix.M2.Z * transpose.M3.Z;

            float resultM31 = matrix.M3.X * transpose.M1.X + matrix.M3.Y * transpose.M1.Y + matrix.M3.Z * transpose.M1.Z;
            float resultM32 = matrix.M3.X * transpose.M2.X + matrix.M3.Y * transpose.M2.Y + matrix.M3.Z * transpose.M2.Z;
            float resultM33 = matrix.M3.X * transpose.M3.X + matrix.M3.Y * transpose.M3.Y + matrix.M3.Z * transpose.M3.Z;

            result.M1.X = resultM11;
            result.M1.Y = resultM12;
            result.M1.Z = resultM13;

            result.M2.X = resultM21;
            result.M2.Y = resultM22;
            result.M2.Z = resultM23;

            result.M3.X = resultM31;
            result.M3.Y = resultM32;
            result.M3.Z = resultM33;
        }

        /// <summary>
        /// Scales all components of the matrix.
        /// </summary>
        /// <param name="matrix">Matrix to scale.</param>
        /// <param name="scale">Amount to scale.</param>
        /// <param name="result">Scaled matrix.</param>
        public static void Multiply(ref Matrix3x3 matrix, float scale, out Matrix3x3 result)
        {
            result.M1.X = matrix.M1.X * scale;
            result.M1.Y = matrix.M1.Y * scale;
            result.M1.Z = matrix.M1.Z * scale;

            result.M2.X = matrix.M2.X * scale;
            result.M2.Y = matrix.M2.Y * scale;
            result.M2.Z = matrix.M2.Z * scale;

            result.M3.X = matrix.M3.X * scale;
            result.M3.Y = matrix.M3.Y * scale;
            result.M3.Z = matrix.M3.Z * scale;
        }

        /// <summary>
        /// Negates every element in the matrix.
        /// </summary>
        /// <param name="matrix">Matrix to negate.</param>
        /// <param name="result">Negated matrix.</param>
        public static void Negate(ref Matrix3x3 matrix, out Matrix3x3 result)
        {
            result.M1.X = -matrix.M1.X;
            result.M1.Y = -matrix.M1.Y;
            result.M1.Z = -matrix.M1.Z;

            result.M2.X = -matrix.M2.X;
            result.M2.Y = -matrix.M2.Y;
            result.M2.Z = -matrix.M2.Z;

            result.M3.X = -matrix.M3.X;
            result.M3.Y = -matrix.M3.Y;
            result.M3.Z = -matrix.M3.Z;
        }

        /// <summary>
        /// Subtracts the two matrices from each other on a per-element basis.
        /// </summary>
        /// <param name="a">First matrix to subtract.</param>
        /// <param name="b">Second matrix to subtract.</param>
        /// <param name="result">Difference of the two matrices.</param>
        public static void Subtract(ref Matrix3x3 a, ref Matrix3x3 b, out Matrix3x3 result)
        {
            float m11 = a.M1.X - b.M1.X;
            float m12 = a.M1.Y - b.M1.Y;
            float m13 = a.M1.Z - b.M1.Z;

            float m21 = a.M2.X - b.M2.X;
            float m22 = a.M2.Y - b.M2.Y;
            float m23 = a.M2.Z - b.M2.Z;

            float m31 = a.M3.X - b.M3.X;
            float m32 = a.M3.Y - b.M3.Y;
            float m33 = a.M3.Z - b.M3.Z;

            result.M1.X = m11;
            result.M1.Y = m12;
            result.M1.Z = m13;

            result.M2.X = m21;
            result.M2.Y = m22;
            result.M2.Z = m23;

            result.M3.X = m31;
            result.M3.Y = m32;
            result.M3.Z = m33;
        }

        /// <summary>
        /// Creates a 4x4 matrix from a 3x3 matrix.
        /// </summary>
        /// <param name="a">3x3 matrix.</param>
        /// <param name="b">Created 4x4 matrix.</param>
        public static void ToMatrix4X4(ref Matrix3x3 a, out Matrix b)
        {
#if !WINDOWS
            b = new Matrix();
#endif
            b.M1.X = a.M1.X;
            b.M1.Y = a.M1.Y;
            b.M1.Z = a.M1.Z;

            b.M2.X = a.M2.X;
            b.M2.Y = a.M2.Y;
            b.M2.Z = a.M2.Z;

            b.M3.X = a.M3.X;
            b.M3.Y = a.M3.Y;
            b.M3.Z = a.M3.Z;

            //b.M44 = 1;
            //b.M14 = 0;
            //b.M24 = 0;
            //b.M34 = 0;
            b.M4.X = 0;
            b.M4.Y = 0;
            b.M4.Z = 0;
        }

        /// <summary>
        /// Creates a 4x4 matrix from a 3x3 matrix and an origin
        /// </summary>
        /// <param name="a">3x3 matrix.</param>
        /// <param name="origin"></param>
        /// <param name="b">Created 4x4 matrix.</param>
        public static void ToMatrix4X4(ref Matrix3x3 a, ref Vector3 origin, out Matrix b)
        {
#if !WINDOWS
            b = new Matrix();
#endif
            b.M1.X = a.M1.X;
            b.M1.Y = a.M1.Y;
            b.M1.Z = a.M1.Z;

            b.M2.X = a.M2.X;
            b.M2.Y = a.M2.Y;
            b.M2.Z = a.M2.Z;

            b.M3.X = a.M3.X;
            b.M3.Y = a.M3.Y;
            b.M3.Z = a.M3.Z;

            b.M4 = origin;
        }

        /// <summary>
        /// Creates a 4x4 matrix from a 3x3 matrix.
        /// </summary>
        /// <param name="a">3x3 matrix.</param>
        /// <returns>Created 4x4 matrix.</returns>
        public static Matrix ToMatrix4X4(Matrix3x3 a)
        {
#if !WINDOWS
            Matrix b = new Matrix();
#else
            Matrix b;
#endif
            b.M1.X = a.M1.X;
            b.M1.Y = a.M1.Y;
            b.M1.Z = a.M1.Z;

            b.M2.X = a.M2.X;
            b.M2.Y = a.M2.Y;
            b.M2.Z = a.M2.Z;

            b.M3.X = a.M3.X;
            b.M3.Y = a.M3.Y;
            b.M3.Z = a.M3.Z;

            //b.M44 = 1;
            //b.M14 = 0;
            //b.M24 = 0;
            //b.M34 = 0;
            b.M4.X = 0;
            b.M4.Y = 0;
            b.M4.Z = 0;
            return b;
        }

#if ALLOW_LAZY_VECTORS
        public static Vector3 Transform(Vector3 v, Matrix3x3 matrix)
        {
            Vector3 result;
            Transform(ref v, ref matrix, out result);
            return result;
        }
#endif
        /// <summary>
        /// Transforms the vector by the matrix.
        /// </summary>
        /// <param name="v">Vector3 to transform.</param>
        /// <param name="matrix">Matrix to use as the transformation.</param>
        /// <param name="result">Product of the transformation.</param>
        public static void Transform(ref Vector3 v, ref Matrix3x3 matrix, out Vector3 result)
        {
            float vX = v.X;
            float vY = v.Y;
            float vZ = v.Z;
#if !WINDOWS
            result = new Vector3();
#endif
            result.X = vX * matrix.M1.X + vY * matrix.M2.X + vZ * matrix.M3.X;
            result.Y = vX * matrix.M1.Y + vY * matrix.M2.Y + vZ * matrix.M3.Y;
            result.Z = vX * matrix.M1.Z + vY * matrix.M2.Z + vZ * matrix.M3.Z;
        }


        /// <summary>
        /// Transforms the vector by the matrix.
        /// </summary>
        /// <param name="v">Vector3 to transform.</param>
        /// <param name="matrix">Matrix to use as the transformation.</param>
        /// <returns>Product of the transformation.</returns>
        public static Vector3 Transform(ref Vector3 v, ref Matrix3x3 matrix)
        {
            Vector3 result;
#if !WINDOWS
            result = new Vector3();
#endif
            float vX = v.X;
            float vY = v.Y;
            float vZ = v.Z;

            result.X = vX * matrix.M1.X + vY * matrix.M2.X + vZ * matrix.M3.X;
            result.Y = vX * matrix.M1.Y + vY * matrix.M2.Y + vZ * matrix.M3.Y;
            result.Z = vX * matrix.M1.Z + vY * matrix.M2.Z + vZ * matrix.M3.Z;
            return result;
        }

        /// <summary>
        /// Transforms the vector by the matrix's transpose.
        /// </summary>
        /// <param name="v">Vector3 to transform.</param>
        /// <param name="matrix">Matrix to use as the transformation transpose.</param>
        /// <param name="result">Product of the transformation.</param>
        public static void TransformTranspose(ref Vector3 v, ref Matrix3x3 matrix, out Vector3 result)
        {
            float vX = v.X;
            float vY = v.Y;
            float vZ = v.Z;
#if !WINDOWS
            result = new Vector3();
#endif
            result.X = vX * matrix.M1.X + vY * matrix.M1.Y + vZ * matrix.M1.Z;
            result.Y = vX * matrix.M2.X + vY * matrix.M2.Y + vZ * matrix.M2.Z;
            result.Z = vX * matrix.M3.X + vY * matrix.M3.Y + vZ * matrix.M3.Z;
        }

        /// <summary>
        /// Transforms the vector by the matrix's transpose.
        /// </summary>
        /// <param name="v">Vector3 to transform.</param>
        /// <param name="matrix">Matrix to use as the transformation transpose.</param>
        /// <returns>Product of the transformation.</returns>
        public static Vector3 TransformTranspose(Vector3 v, Matrix3x3 matrix)
        {
            float vX = v.X;
            float vY = v.Y;
            float vZ = v.Z;
            Vector3 result;
#if !WINDOWS
            result = new Vector3();
#endif
            result.X = vX * matrix.M1.X + vY * matrix.M1.Y + vZ * matrix.M1.Z;
            result.Y = vX * matrix.M2.X + vY * matrix.M2.Y + vZ * matrix.M2.Z;
            result.Z = vX * matrix.M3.X + vY * matrix.M3.Y + vZ * matrix.M3.Z;
            return result;
        }

        /// <summary>
        /// Computes the transposed matrix of a matrix.
        /// </summary>
        /// <param name="matrix">Matrix to transpose.</param>
        /// <param name="result">Transposed matrix.</param>
        public static void Transpose(ref Matrix3x3 matrix, out Matrix3x3 result)
        {
            float m21 = matrix.M1.Y;
            float m31 = matrix.M1.Z;
            float m12 = matrix.M2.X;
            float m32 = matrix.M2.Z;
            float m13 = matrix.M3.X;
            float m23 = matrix.M3.Y;

            result.M1.X = matrix.M1.X;
            result.M1.Y = m12;
            result.M1.Z = m13;
            result.M2.X = m21;
            result.M2.Y = matrix.M2.Y;
            result.M2.Z = m23;
            result.M3.X = m31;
            result.M3.Y = m32;
            result.M3.Z = matrix.M3.Z;
        }

        /// <summary>
        /// Computes the transposed matrix of a matrix.
        /// </summary>
        /// <param name="matrix">Matrix to transpose.</param>
        /// <param name="result">Transposed matrix.</param>
        public static void Transpose(ref Matrix matrix, out Matrix3x3 result)
        {
            float m21 = matrix.M1.Y;
            float m31 = matrix.M1.Z;
            float m12 = matrix.M2.X;
            float m32 = matrix.M2.Z;
            float m13 = matrix.M3.X;
            float m23 = matrix.M3.Y;

            result.M1.X = matrix.M1.X;
            result.M1.Y = m12;
            result.M1.Z = m13;
            result.M2.X = m21;
            result.M2.Y = matrix.M2.Y;
            result.M2.Z = m23;
            result.M3.X = m31;
            result.M3.Y = m32;
            result.M3.Z = matrix.M3.Z;
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

            intermediate = M2.Z;
            M2.Z = M3.Y;
            M3.Y = intermediate;
        }


        /// <summary>
        /// Creates a string representation of the matrix.
        /// </summary>
        /// <returns>A string representation of the matrix.</returns>
        public override string ToString()
        {
            return "{" + M1.X + ", " + M1.Y + ", " + M1.Z + "} " +
                   "{" + M2.X + ", " + M2.Y + ", " + M2.Z + "} " +
                   "{" + M3.X + ", " + M3.Y + ", " + M3.Z + "}";
        }

        /// <summary>
        /// Calculates the determinant of the matrix.
        /// </summary>
        /// <returns>The matrix's determinant.</returns>
        public float Determinant()
        {
            return M1.X * M2.Y * M3.Z + M1.Y * M2.Z * M3.X + M1.Z * M2.X * M3.Y -
                   M3.X * M2.Y * M1.Z - M3.Y * M2.Z * M1.X - M3.Z * M2.X * M1.Y;
        }

        /// <summary>
        /// Calculates the determinant of largest nonsingular submatrix, excluding 2x2's that involve M1.Z or M3.X, and excluding all 1x1's that involve nondiagonal elements.
        /// </summary>
        /// <param name="subMatrixCode">Represents the submatrix that was used to compute the determinant.
        /// 0 is the full 3x3.  1 is the upper left 2x2.  2 is the lower right 2x2.  3 is the four corners.
        /// 4 is M1.X.  5 is M2.Y.  6 is M3.Z.</param>
        /// <returns>The matrix's determinant.</returns>
        internal float AdaptiveDeterminant(out int subMatrixCode)
        {
            //Try the full matrix first.
            float determinant = M1.X * M2.Y * M3.Z + M1.Y * M2.Z * M3.X + M1.Z * M2.X * M3.Y -
                                M3.X * M2.Y * M1.Z - M3.Y * M2.Z * M1.X - M3.Z * M2.X * M1.Y;
            if (determinant != 0) //This could be a little numerically flimsy.  Fortunately, the way this method is used, that doesn't matter!
            {
                subMatrixCode = 0;
                return determinant;
            }
            //Try m11, m12, m21, m22.
            determinant = M1.X * M2.Y - M1.Y * M2.X;
            if (determinant != 0)
            {
                subMatrixCode = 1;
                return determinant;
            }
            //Try m22, m23, m32, m33.
            determinant = M2.Y * M3.Z - M2.Z * M3.Y;
            if (determinant != 0)
            {
                subMatrixCode = 2;
                return determinant;
            }
            //Try m11, m13, m31, m33.
            determinant = M1.X * M3.Z - M1.Z * M1.Y;
            if (determinant != 0)
            {
                subMatrixCode = 3;
                return determinant;
            }
            //Try m11.
            if (M1.X != 0)
            {
                subMatrixCode = 4;
                return M1.X;
            }
            //Try m22.
            if (M2.Y != 0)
            {
                subMatrixCode = 5;
                return M2.Y;
            }
            //Try m33.
            if (M3.Z != 0)
            {
                subMatrixCode = 6;
                return M3.Z;
            }
            //It's completely singular!
            subMatrixCode = -1;
            return 0;
        }

        /// <summary>
        /// Creates a 3x3 matrix representing the orientation stored in the quaternion.
        /// </summary>
        /// <param name="quaternion">Quaternion to use to create a matrix.</param>
        /// <param name="result">Matrix representing the quaternion's orientation.</param>
        public static void CreateFromQuaternion(ref Quaternion quaternion, out Matrix3x3 result)
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

            result.M1.Y = XY + ZW;
            result.M2.Y = 1 - XX - ZZ;
            result.M3.Y = YZ - XW;

            result.M1.Z = XZ - YW;
            result.M2.Z = YZ + XW;
            result.M3.Z = 1 - XX - YY;
        }

        /// <summary>
        /// Creates a 3x3 matrix representing the orientation stored in the quaternion.
        /// </summary>
        /// <param name="quaternion">Quaternion to use to create a matrix.</param>
        /// <returns>Matrix representing the quaternion's orientation.</returns>
        public static Matrix3x3 CreateFromQuaternion(Quaternion quaternion)
        {
            Matrix3x3 result;
            CreateFromQuaternion(ref quaternion, out result);
            return result;
        }

        /// <summary>
        /// Computes the outer product of the given vectors.
        /// </summary>
        /// <param name="a">First vector.</param>
        /// <param name="b">Second vector.</param>
        /// <param name="result">Outer product result.</param>
        public static void CreateOuterProduct(ref Vector3 a, ref Vector3 b, out Matrix3x3 result)
        {
            result.M1.X = a.X * b.X;
            result.M1.Y = a.X * b.Y;
            result.M1.Z = a.X * b.Z;

            result.M2.X = a.Y * b.X;
            result.M2.Y = a.Y * b.Y;
            result.M2.Z = a.Y * b.Z;

            result.M3.X = a.Z * b.X;
            result.M3.Y = a.Z * b.Y;
            result.M3.Z = a.Z * b.Z;
        }

        /// <summary>
        /// Creates a matrix representing a rotation of a given angle around a given axis.
        /// </summary>
        /// <param name="axis">Axis around which to rotate.</param>
        /// <param name="angle">Amount to rotate.</param>
        /// <returns>Matrix representing the rotation.</returns>
        public static Matrix3x3 CreateFromAxisAngle(Vector3 axis, float angle)
        {
            Matrix3x3 toReturn;
            CreateFromAxisAngle(ref axis, angle, out toReturn);
            return toReturn;
        }

        /// <summary>
        /// Creates a matrix representing a rotation of a given angle around a given axis.
        /// </summary>
        /// <param name="axis">Axis around which to rotate.</param>
        /// <param name="angle">Amount to rotate.</param>
        /// <param name="result">Matrix representing the rotation.</param>
        public static void CreateFromAxisAngle(ref Vector3 axis, float angle, out Matrix3x3 result)
        {
            float xx = axis.X * axis.X;
            float yy = axis.Y * axis.Y;
            float zz = axis.Z * axis.Z;
            float xy = axis.X * axis.Y;
            float xz = axis.X * axis.Z;
            float yz = axis.Y * axis.Z;

            float sinAngle = (float)System.Math.Sin(angle);
            float oneMinusCosAngle = 1 - (float)System.Math.Cos(angle);

            result.M1.X = 1 + oneMinusCosAngle * (xx - 1);
            result.M2.X = -axis.Z * sinAngle + oneMinusCosAngle * xy;
            result.M3.X = axis.Y * sinAngle + oneMinusCosAngle * xz;

            result.M1.Y = axis.Z * sinAngle + oneMinusCosAngle * xy;
            result.M2.Y = 1 + oneMinusCosAngle * (yy - 1);
            result.M3.Y = -axis.X * sinAngle + oneMinusCosAngle * yz;

            result.M1.Z = -axis.Y * sinAngle + oneMinusCosAngle * xz;
            result.M2.Z = axis.X * sinAngle + oneMinusCosAngle * yz;
            result.M3.Z = 1 + oneMinusCosAngle * (zz - 1);
        }






    }
}