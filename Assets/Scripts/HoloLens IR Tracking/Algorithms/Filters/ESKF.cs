using MathNet.Numerics.LinearAlgebra;
using MathNet.Spatial.Euclidean;
using System.Collections.Generic;
using System.Diagnostics;

namespace StereoSensors.Tracking
{
    class ESKF : ErrorStateKF, IKalmanFilter
    {
        // Noise
        private readonly double var_a_nx, var_a_ny, var_a_nz;
        private readonly double var_om_nx, var_om_ny, var_om_nz;

        // Bias noise
        private readonly double var_a_wx, var_a_wy, var_a_wz;
        private readonly double var_om_wx, var_om_wy, var_om_wz;

        // Nominal variables

        // Orientation (Quaternion)
        private Quaternion q ;
        // Orientation (R Matrix)
        private Matrix<double> q_rot;
        // Position
        private Vector<double> p;
        // Velocity
        private Vector<double> v;
        // Accelerometer bias
        private Vector<double> a_b;
        // Gyroscope bias
        private Vector<double> om_b;
        // Gravity
        private Vector<double> g;
        // Accelerometer measurement
        private Vector<double> a_m_t;
        // Gyroscope measurement
        private Vector<double> om_m_t;
        private Vector<double> mag_ref;

        private List<Vector<double>> marker_refs;

        // One component of the H Matrix
        Matrix<double> X_dx;

        // Covariance reset matrix
        Matrix<double> G;
        
        private int state_size = 15;
        
        // Markers
        private int obs_size = 15;
        //private int nominal_state_size = 15+1
        public ESKF()
        {
            // Define initial error state
            state = Vector<double>.Build.Dense(state_size);

            // Define initial nominal states
            p = Vector<double>.Build.Dense(3);
            v = Vector<double>.Build.Dense(3);
            q_rot = Matrix<double>.Build.DenseIdentity(3);
            q = new Quaternion(1, 0, 0, 0);
            a_b = Vector<double>.Build.Dense(3); ;
            om_b= Vector<double>.Build.Dense(3); ;
            // Gravity
            g = Vector<double>.Build.DenseOfArray(new double[] { 0, -9.80665, 0 }); ;
            // Accelerometer measurement
            a_m_t = Vector<double>.Build.Dense(3);
            // Gyroscope measurement
            om_m_t = Vector<double>.Build.Dense(3);

            // State covariances
            P_pr = Matrix<double>.Build.DenseIdentity(state_size, state_size) * 0.01;
            P_po = Matrix<double>.Build.DenseIdentity(state_size, state_size) * 0.01;

            // Process nad measurement covariances
            Q = Matrix<double>.Build.Sparse(state_size, state_size);

            var_a_nx = 0.20669209162667224; var_a_ny = 0.18070485871372426; var_a_nz = 0.21218359826934503;
            var_om_nx = 0.009453351788631813;  var_om_ny = 0.0068112688842490565; var_om_nz= 0.008534051153340964;

            var_a_wx = 0.2060301949481104; var_a_wy = 0.14218195857796866; var_a_wz= 0.2120327470670869;
            var_om_wx = 0.0001; var_om_wy = 0.0001; var_om_wz = 0.0008506998819410744;

            R = Matrix<double>.Build.Sparse(obs_size, obs_size);
            R[0, 0] = 0.010462;
            R[1, 1] = 0.010937;
            R[2, 2] = 0.034030;

            R[3, 3] = 0.010462;
            R[4, 4] = 0.010937;
            R[5, 5] = 0.034030;

            R[6, 6] = 0.010462;
            R[7, 7] = 0.010937;
            R[8, 8] = 0.034030;

            R[9, 9] = 0.010462;
            R[10, 10] = 0.010937;
            R[11, 11] = 0.034030;

            R[12, 12] = 0.010462;
            R[13, 13] = 0.010937;
            R[14, 14] = 0.034030;

            //R[15, 15] = 0.0010462;
            //R[16, 16] = 0.0010937;
            //R[17, 17] = 0.0034030;

            // Matrix initializations so that instead of rebuilding every update, only the relevant entries are changed
            X_dx = Matrix<double>.Build.Sparse(state_size+1, state_size);
            X_dx.SetSubMatrix(0, 0, Matrix<double>.Build.DiagonalIdentity(6));
            X_dx.SetSubMatrix(10, 9, Matrix<double>.Build.DiagonalIdentity(6));
            G = Matrix<double>.Build.SparseIdentity(state_size, state_size);
            //LAExtensions.PrintMatrix(X_dx);

            //LAExtensions.PrintMatrix(G);

            F = Matrix<double>.Build.SparseIdentity(state_size, state_size);

            marker_refs = new List<Vector<double>>();
            Vector<double> m1 = Vector<double>.Build.DenseOfArray(new double[] { 0.07125, 0, 0});
            Vector<double> m2 = Vector<double>.Build.DenseOfArray(new double[] { 0.02125, 0, 0});
            Vector<double> m3 = Vector<double>.Build.DenseOfArray(new double[] { -0.06375, 0, 0.03 });
            Vector<double> m4 = Vector<double>.Build.DenseOfArray(new double[] { -0.02875,0, -0.03 });
            marker_refs.Add(m1); marker_refs.Add(m2); marker_refs.Add(m3); marker_refs.Add(m4);
        }

        public override void UpdateNominal(Vector<double> meas, double dt)
        {
            a_m_t = meas.SubVector(0, 3);
            om_m_t = meas.SubVector(3, 3);

            q = q * LAExtensions.QuatFromAngleAxis((om_m_t - om_b) * dt);
            q_rot = LAExtensions.RotationFromQuat(q);

            v += (q_rot * (a_m_t - a_b) + g) * dt;
            p += v * dt + 0.5 * (q_rot * (a_m_t - a_b) + g) * dt * dt;

        }

        public override void InjectError()
        {
            p[0] += state[0]; p[1] += state[1]; p[2] += state[2];
            v[0] += state[3]; v[1] += state[4]; v[2] += state[5];

            q = q * LAExtensions.QuatFromAngleAxis(state[6], state[7], state[8]);

            a_b[0] += state[9]; a_b[1] += state[10]; a_b[2] += state[11];
            om_b[0] += state[12]; om_b[1] += state[13]; om_b[2] += state[14];
            //g[0] += state[15]; g[1] += state[16]; g[2] += state[17];

            ResetError();
            //Debug.WriteLine(P_pr[3, 4] + " " + P_pr[3, 5] + " " + P_pr[4, 5]);
            //Debug.WriteLine(P_f[3, 3] + " " + P_pr[4, 4] + " " + P_pr[5, 5]);

        }
        public override void UpdatePosterior(Vector<double> obs, double dt)
        {
            if (mag_ref == null)
            {
                mag_ref = Vector<double>.Build.DenseOfArray(new double[] { obs[0], obs[1], obs[2] });
            }
            // TODO
            // Compute H Matrix
            //state = K * (obs - nominal); 

            X_dx[6, 6] = 0.5 * -q.ImagX; X_dx[6, 7] = 0.5 *  -q.ImagY; X_dx[6, 8] = 0.5 * -q.ImagZ;
            X_dx[7, 6] = 0.5 * q.Real; X_dx[7, 7] = 0.5 * -q.ImagZ; X_dx[7, 8] = 0.5 * q.ImagY;
            X_dx[8, 6] = 0.5 * q.ImagZ; X_dx[8, 7] = 0.5 * q.Real; X_dx[8, 8] = 0.5 *  -q.ImagX;
            X_dx[9, 6] = 0.5 * -q.ImagY; X_dx[9, 7] = 0.5 * q.ImagX; X_dx[9, 8] = 0.5 *  q.Real;

            //LAExtensions.PrintMatrix(X_dx);


            //x v q a_b w_b g

            Matrix<double> H_x = Matrix<double>.Build.Dense(obs.Count, state.Count + 1);
            // Derivative of rotating a vector wrt the vector is just the rotation
            // matrix itself
            //H_x.SetSubMatrix(0, 16, q_rot.Transpose());
            // Derivative of rotating a vector wrt the rotation is given by this complicated
            // formula
            // TODO: Write it all out in code for speedup

            //Vector<double> q_imag = Vector<double>.Build.DenseOfArray(new double[] { -q.ImagX, -q.ImagY, -q.ImagZ });
            //Vector<double> col1 = q.Real * g + LAExtensions.Cross(q_imag, g);
            //H_x[0, 6] = 2 * col1[0];
            //H_x[1, 6] = 2 * col1[1];
            //H_x[2, 6] = 2 * col1[2];
            //H_x.SetSubMatrix(0, 6, LAExtensions.QuaternionRotDerivative(q, g));


            //// Magnetometer
            ////H_x.SetSubMatrix(0, 6, LAExtensions.QuaternionRotDerivative(q, mag_ref));
            //Vector<double> obs_prediction = Vector<double>.Build.Dense(obs.Count);
            //    //H_x.SetSubMatrix(i * 3, 0, Matrix<double>.Build.DiagonalIdentity(3));
            //H_x.SetSubMatrix(0, 6, LAExtensions.QuaternionRotDerivative(q, mag_ref));
            ////LAExtensions.PrintMatrix(H_x);
            //obs_prediction.SetSubVector(0, 3, q_rot * mag_ref);

            // Markers
            //H_x.SetSubMatrix(0, 6, LAExtensions.QuaternionRotDerivative(q, mag_ref));
            Vector<double> obs_prediction = Vector<double>.Build.Dense(obs.Count);
            for (int i = 0; i < marker_refs.Count; i++)
            {
                //H_x.SetSubMatrix(i * 3, 0, Matrix<double>.Build.DenseIdentity(3));
                H_x.SetSubMatrix(i * 3, 6, LAExtensions.QuaternionRotDerivative(q, marker_refs[i]));
                obs_prediction.SetSubVector(i * 3, 3, q_rot * marker_refs[i]);
            }

            ////Gravity Observation
            //H_x.SetSubMatrix(15, 16, Matrix<double>.Build.DenseIdentity(3));
            //obs_prediction.SetSubVector(15, 3, g);


            //Debug.WriteLine(g);
            // Centroid Observation
            H_x.SetSubMatrix(3 * marker_refs.Count, 0, Matrix<double>.Build.DiagonalIdentity(3));
            obs_prediction.SetSubVector(3 * marker_refs.Count, 3, p);
            //LAExtensions.PrintMatrix(H_x);

            //H_x.SetSubMatrix(0, 0, Matrix<double>.Build.DiagonalIdentity(3));
            //obs_prediction.SetSubVector(0, 3, p);

            //LAExtensions.PrintMatrix(H_x);

            //H_x.SetSubMatrix(0, 7, 2 * (Matrix<double>.Build.Diagonal(3, 3, q_imag.DotProduct(g)) + q_imag.OuterProduct(g) - g.OuterProduct(q_imag) - q.Real * LAExtensions.SkewSymmetric(g)));
            //LAExtensions.PrintMatrix(H_x);
            //LAExtensions.PrintMatrix(X_dx);
            //Debug.WriteLine(H_x.SubMatrix(0, 3, 10, 9).ToString

            H = H_x * X_dx;

            //LAExtensions.PrintMatrix(H * F);
            //LAExtensions.PrintMatrix(H * F * F);
            //LAExtensions.PrintMatrix(H * F * F * F);

            //K = P_pr * H.Transpose() * (H * P_pr * H.Transpose() + R).Inverse();
            K = P_pr * H.Transpose() * (H * P_pr * H.Transpose() + R).Inverse();

            //LAExtensions.PrintMatrix(P_pr);
            //LAExtensions.PrintMatrix(K);

            //LAExtensions.PrintMatrix(H);
            //LAExtensions.PrintMatrix(K);
            //P_po = P_pr - K * (H * P_pr * H.Transpose() + R) * K.Transpose();
            //LAExtensions.PrintMatrix(P_po);

            //// Symmetric form
            //P_po = P_pr - K * (H * P_pr * H.Transpose() + R) * K.Transpose();

            //Joseph Form
            Matrix<double> A = (Matrix<double>.Build.DiagonalIdentity(K.RowCount, H.ColumnCount) - K * H);
            P_po = A * P_pr * A.Transpose() + K * R * K.Transpose();

            //// Standard form
            //P_po = (Matrix<double>.Build.DiagonalIdentity(K.RowCount, H.ColumnCount) - K * H) * P_pr;

            //LAExtensions.PrintMatrix(K);
            //Debug.WriteLine(K.SubMatrix(9, 9, 0, 3));

            //P_po = (Matrix<double>.Build.SparseIdentity(18) - K * H) * P_pr;// (H * P_pr * H.Transpose() + R) * K.Transpose();
            //Debug.WriteLine("Observation: " + obs.SubVector(3 * marker_refs.Count, 3).ToString());
            //Debug.WriteLine("Prediction: " + obs_prediction.SubVector(3*marker_refs.Count, 3).ToString());

            //Debug.WriteLine(P_po.ToString());
            //Debug.WriteLine(q);
            //Vector<double> obs_prediction = Vector<double>.Build.Dense(obs_size);
            //obs_prediction.SetSubVector(0, 3, q_rot * g);
            //obs_prediction.SetSubVector(0, 3, q_rot * mag_ref);
            //Debug.WriteLine("Observed: " + obs.ToString());
            //Debug.WriteLine("Estimate: "  + obs_prediction.ToString());

            state = K * (obs - obs_prediction);
        }

        protected void ResetError()
        {
            // Orientation error is
            // x - 6
            // y - 7
            // z - 8
            G[6, 7] = 0.5 * state[8];
            G[7, 6] = -0.5 * state[8];
            G[6, 8] = -0.5 * state[7];
            G[8, 6] = 0.5 * state[7];
            G[7, 8] = 0.5 * state[6];
            G[8, 7] = -0.5 * state[6];

            //LAExtensions.PrintMatrix(G);
            //LAExtensions.PrintMatrix(G);
            for (int i = 0; i < state.Count; i++)
            {
                state[i] = 0;
            }            

            P_po = G * P_po * G.Transpose();


        }
        public override void UpdatePrior(double dt)
        {
            F.SetSubMatrix(3, 6, -q_rot * LAExtensions.SkewSymmetric(a_m_t - a_b) * dt);
            F.SetSubMatrix(3, 9, -q_rot * dt);
            //F.SetSubMatrix(3, 15, Matrix<double>.Build.Diagonal(3, 3, dt));

            F.SetSubMatrix(6, 6, LAExtensions.RotationFromAngleAxis((om_m_t - om_b) * dt).Transpose());
            F.SetSubMatrix(6, 12, Matrix<double>.Build.Diagonal(3, 3, -dt));

            F.SetSubMatrix(0, 3, Matrix<double>.Build.Diagonal(3, 3, dt));
            //LAExtensions.PrintMatrix(F);
            Q[3, 3] = dt * dt * var_a_nx * var_a_nx;
            Q[4, 4] = dt * dt * var_a_ny * var_a_ny;
            Q[5, 5] = dt * dt * var_a_nz * var_a_nz;

            Q[6, 6] = dt * dt * var_om_nx * var_om_nx;
            Q[7, 7] = dt * dt * var_om_ny * var_om_ny;
            Q[8, 8] = dt * dt * var_om_nz * var_om_nz;

            Q[9, 9] = dt * var_a_wx * var_a_wx;
            Q[10, 10] = dt * var_a_wy * var_a_wy;
            Q[11, 11] = dt * var_a_wz * var_a_wz;

            Q[12, 12] = dt * var_om_wx * var_om_wx;
            Q[13, 13] = dt * var_om_wy * var_om_wy;
            Q[14, 14] = dt * var_om_wz * var_om_wz;

            //LAExtensions.PrintMatrix(Q);

            P_pr = F * P_po *F.Transpose() +  Q ;
        }

        public override Quaternion QueryOrientation()
        {
            return q;
        }
        public override Vector<double> QueryPosition()
        {
            return p;
        }

        public override void InitializeStates(Vector<double> s)
        {
            state = s;
        }

        public override void InitializePosition(Vector<double> s)
        {
            p = s;
        }
        public override void InitializeOrientation(Quaternion s)
        {
            q = s;
            q_rot = LAExtensions.RotationFromQuat(q);
        }

        public override Vector<double> GetState()
        {
            return state;
        }
    }
}