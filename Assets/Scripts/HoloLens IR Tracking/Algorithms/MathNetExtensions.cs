using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Spatial.Euclidean;

namespace MathNet.Numerics.LinearAlgebra
{
    public class LAExtensions
    {
        public static Matrix<double> SkewSymmetric(Vector<double> x)
        {
            Matrix<double> ss = Matrix<double>.Build.Dense(3, 3);
            if (x.Count != 3)
            {
                throw new ArgumentException("Vector not of length 3.", nameof(x));
            }
            ss[0, 1] = -x[2]; ss[1, 0] = x[2];
            ss[0, 2] = x[1]; ss[2, 0] = -x[1];
            ss[0, 1] = -x[2]; ss[1, 0] = x[2];
            ss[2, 1] = x[0]; ss[1, 2] = -x[0];

            return ss;
        }

        public static Vector<double> Cross(Vector<double> x, Vector<double> y)
        {
            if (x.Count != 3)
            {
                throw new ArgumentException("Vector not of length 3.", nameof(x));
            }
            if (y.Count != 3)
            {
                throw new ArgumentException("Vector not of length 3.", nameof(y));
            }

            Vector<double> p = Vector<double>.Build.Dense(3);
            p[0] = x[1] * y[2] - x[2] * y[1];
            p[1] = x[2] * y[0] - x[0] * y[2];
            p[2] = x[0] * y[1] - x[1] * y[0];

            return p;
        }

        public static Matrix<double> RotationFromQuat(Quaternion q)
        {
            Matrix<double> R = Matrix<double>.Build.DenseIdentity(3, 3);
            R[0, 0] -= 2 * (q.ImagY * q.ImagY + q.ImagZ * q.ImagZ);
            R[0, 1] = 2 * (q.ImagX * q.ImagY - q.ImagZ * q.Real);
            R[0, 2] = 2 * (q.ImagX * q.ImagZ + q.ImagY * q.Real);
            R[1, 0] = 2 * (q.ImagX * q.ImagY + q.ImagZ * q.Real);
            R[1, 1] -= 2 * (q.ImagX * q.ImagX + q.ImagZ * q.ImagZ);
            R[1, 2] = 2 * (q.ImagY * q.ImagZ - q.ImagX * q.Real);
            R[2, 0] = 2 * (q.ImagX * q.ImagZ - q.ImagY * q.Real);
            R[2, 1] = 2 * (q.ImagY * q.ImagZ + q.ImagX * q.Real);
            R[2, 2] -= 2 * (q.ImagX * q.ImagX + q.ImagY * q.ImagY);
            return R;
        }

        public static Quaternion QuatFromRotation(Matrix<double> R)
        {
            double qw = Math.Sqrt(1 + R[0,0] + R[1, 1] + R[2, 2]) / 2;
            double qx = (R[2,1] - R[1,2]) / (4 * qw);
            double qy = (R[0,2] - R[2, 0]) / (4 * qw);
            double qz = (R[1,0] - R[0,1]) / (4 * qw);

            return new Quaternion(qw, qx, qy, qz);
        }

        public static Quaternion QuatFromAngleAxis(double x, double y, double z)
        {
            double phi = Math.Sqrt(x * x + y * y + z * z);
            if (phi == 0)
            {
                return Quaternion.One;
            }
            double sin_ph = Math.Sin(phi / 2);
            double cos_ph = Math.Cos(phi / 2);
            return new Quaternion(cos_ph, x / phi * sin_ph, y / phi * sin_ph, z / phi * sin_ph);
        }
        public static Quaternion QuatFromAngleAxis(Vector<double> om)
        {
            if (om.Count != 3)
            {
                throw new ArgumentException("Vector not of length 3.", nameof(om));
            }

            double phi = om.L2Norm();
            if (phi == 0)
            {
                return Quaternion.One;
            }

            //om /= phi;

            double sin_ph = Math.Sin(phi / 2);
            double cos_ph = Math.Cos(phi / 2);
            return new Quaternion(cos_ph, om[0]/phi * sin_ph, om[1] / phi * sin_ph, om[2] / phi * sin_ph);
        }

        public static Matrix<double> RotationFromAngleAxis(Vector<double> om)
        {
            //Stopwatch watch = new Stopwatch();

            if (om.Count != 3)
            {
                throw new ArgumentException("Vector not of length 3.", nameof(om));
            }
            Matrix<double> R = Matrix<double>.Build.DenseIdentity(3, 3);
            if (om.L2Norm() == 0)
            {
                return R;
            }

            double sin_th = Math.Sin(om.L2Norm());
            double one_minus_cos_th = 1 - Math.Cos(om.L2Norm());
            om /= om.L2Norm();

            R[0, 0] += one_minus_cos_th * (-om[2] * om[2] - om[1] * om[1]);
            R[0, 1] = sin_th * -om[2] + one_minus_cos_th * om[0] * om[1];
            R[0, 2] = sin_th * om[1] + one_minus_cos_th * om[0] * om[2];
            R[1, 0] = sin_th * om[2] + one_minus_cos_th * om[0] * om[1];
            R[1, 1] += one_minus_cos_th * (-om[2] * om[2] - om[0] * om[0]);
            R[1, 2] = sin_th * -om[0] + one_minus_cos_th * om[1] * om[2];
            R[2, 0] = sin_th * -om[1] + one_minus_cos_th * om[0] * om[2];
            R[2, 1] = sin_th * om[0] + one_minus_cos_th * om[1] * om[2];
            R[2, 2] += one_minus_cos_th * (-om[1] * om[1] - om[0] * om[0]);

            Matrix<double> R2 = Matrix<double>.Build.DenseIdentity(3, 3) + sin_th * LAExtensions.SkewSymmetric(om) + one_minus_cos_th * LAExtensions.SkewSymmetric(om) * LAExtensions.SkewSymmetric(om);

            //Debug.WriteLine(R);
            //Debug.WriteLine(R2);
            //watch.Stop();
            //Debug.WriteLine(watch.ElapsedTicks);

            ////Debug.WriteLine(R.ToString());
            //watch.Reset();
            //watch.Start();

            //Matrix<double> K = SkewSymmetric(om);
            //watch.Stop();
            //Debug.WriteLine(watch.ElapsedTicks);
            //Matrix<double> R2 = Matrix<double>.Build.DenseIdentity(3) + sin_th * K + one_minus_cos_th * K * K;
            //watch.Stop();
            //Debug.WriteLine(watch.ElapsedTicks);
            //Debug.WriteLine(R2.ToString());

            return R;
        }
        public static void PrintMatrix(Matrix<double> m)
        {
            for (int i = 0; i < m.RowCount; i++)
            {
                for (int j = 0; j < m.ColumnCount; j++)
                {
                    Debug.Write(m[i, j].ToString("n3") + "\t");
                }
                Debug.WriteLine("");
            }
            Debug.WriteLine("");
        }
        public static Matrix<double> QuaternionRotDerivative(Quaternion q, Vector<double> v)
        {
            Matrix<double> dR_dq = Matrix<double>.Build.Dense(3, 4);
            Vector<double> q_imag = Vector<double>.Build.DenseOfArray(new double[] { q.ImagX, q.ImagY, q.ImagZ });
            Vector<double> col1 = q.Real * v + LAExtensions.Cross(q_imag, v);
            dR_dq[0, 0] = 2 * col1[0];
            dR_dq[1, 0] = 2 * col1[1];
            dR_dq[2, 0] = 2 * col1[2];
            dR_dq.SetSubMatrix(0, 1, 2 * (Matrix<double>.Build.Diagonal(3, 3, q_imag.DotProduct(v)) + q_imag.OuterProduct(v) - v.OuterProduct(q_imag) - q.Real * LAExtensions.SkewSymmetric(v)));
            return dR_dq;
        }
    }
}
