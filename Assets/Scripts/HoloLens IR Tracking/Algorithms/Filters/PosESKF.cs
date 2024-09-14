using MathNet.Numerics.LinearAlgebra;
using MathNet.Spatial.Euclidean;
using System.Collections.Generic;
using System.Diagnostics;

namespace StereoSensors.Tracking
{
    class PosESKF : ErrorStateKF
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
        // Gravity
        private Vector<double> g;
        // Accelerometer measurement
        private Vector<double> a_m_t;
        // Gyroscope measurement

        private Vector<double> mag_ref;

        private List<Vector<double>> marker_refs;

        // One component of the H Matrix
        Matrix<double> X_dx;

        // Covariance reset matrix
        Matrix<double> G;
        
        private int state_size = 9;
        
        // Markers
        private int obs_size = 3;
        //private int nominal_state_size = 15+1
        public PosESKF()
        {
            // Define initial error state
            state = Vector<double>.Build.Dense(state_size);

            // Define initial nominal states
            p = Vector<double>.Build.Dense(3);
            v = Vector<double>.Build.Dense(3);
            a_b = Vector<double>.Build.Dense(3); ;
            g = Vector<double>.Build.DenseOfArray(new double[] { 0, -9.80665, 0 }); ;
            //g = Vector<double>.Build.DenseOfArray(new double[] { 0, 0, 0 }); ;

            q_rot = Matrix<double>.Build.DenseIdentity(3);
            q = new Quaternion(1, 0, 0, 0);

            // State covariances
            P_pr = Matrix<double>.Build.DenseIdentity(state_size, state_size);
            P_po = Matrix<double>.Build.DenseIdentity(state_size, state_size);

            // Process nad measurement covariances
            Q = Matrix<double>.Build.Sparse(state_size, state_size);
            var_a_nx = 0.020669209162667224; var_a_ny = 0.018070485871372426; var_a_nz = 0.021218359826934503;
            var_a_wx = 0.02060301949481104; var_a_wy = 0.014218195857796866; var_a_wz = 0.02120327470670869;

            a_m_t = Vector<double>.Build.Dense(3);

            R = Matrix<double>.Build.Sparse(obs_size, obs_size);
            R[0, 0] = 0.0010462;
            R[1, 1] = 0.0010937;
            R[2, 2] = 0.0034030;

            //R[3, 3] = 0.0010462;
            //R[4, 4] = 0.0010937;
            //R[5, 5] = 0.0034030;

            //R[6, 6] = 0.0010462;
            //R[7, 7] = 0.0010937;
            //R[8, 8] = 0.0034030;

            //R[9, 9] = 0.0010462;
            //R[10, 10] = 0.0010937;
            //R[11, 11] = 0.0034030;

            //R[12, 12] = 0.0001;
            //R[13, 13] = 0.0001;
            //R[14, 14] = 0.0001;

            //R[15, 15] = 0.0010462;
            //R[16, 16] = 0.0010937;
            //R[17, 17] = 0.0034030;

            // Matrix initializations so that instead of rebuilding every update, only the relevant entries are changed
            X_dx = Matrix<double>.Build.SparseIdentity(state_size, state_size);
            //X_dx.SetSubMatrix(0, 0, Matrix<double>.Build.DiagonalIdentity(3));
            //X_dx.SetSubMatrix(7, 6, Matrix<double>.Build.DiagonalIdentity(3));
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
        public override void UpdateNominal(Vector<double> meas,  double dt)
        {
            a_m_t = meas.SubVector(0, 3);
            v += (q_rot * (a_m_t - a_b) + g) * dt;
            p += v * dt + 0.5 * (q_rot * (a_m_t - a_b) + g) * dt * dt;
            //a_m_t = meas.SubVector(0, 3);
        }

        public override void InjectError()
        {
            p[0] += state[0]; p[1] += state[1]; p[2] += state[2];
            //Debug.WriteLine(state[0] + ", " + state[1] + ", " + state[2]);
            v[0] += state[3]; v[1] += state[4]; v[2] += state[5];
            a_b[0] += state[6]; a_b[1] += state[7]; a_b[2] += state[8];
            //g[0] += state[15]; g[1] += state[16]; g[2] += state[17];
            Debug.WriteLine(a_m_t);
            Debug.WriteLine(a_b);
            Debug.WriteLine(a_m_t - a_b);

            ResetError();
        }
        public override void UpdatePosterior(Vector<double> obs, double dt)
        {
            if (mag_ref == null)
            {
                mag_ref = Vector<double>.Build.DenseOfArray(new double[] { obs[0], obs[1], obs[2] });
            }

            Matrix<double> H_x = Matrix<double>.Build.Dense(obs.Count, state.Count);

            // Markers
            //H_x.SetSubMatrix(0, 6, LAExtensions.QuaternionRotDerivative(q, mag_ref));
            Vector<double> obs_prediction = Vector<double>.Build.Dense(obs.Count);

            ////Gravity Observation
            //H_x.SetSubMatrix(15, 16, Matrix<double>.Build.DenseIdentity(3));
            //obs_prediction.SetSubVector(15, 3, g);


            //Debug.WriteLine(g);
            // Centroid Observation
            H_x.SetSubMatrix(0, 0, Matrix<double>.Build.DiagonalIdentity(3));
            obs_prediction.SetSubVector(0, 3, p);

            H = H_x * X_dx;

            K = P_pr * H.Transpose() * (H * P_pr * H.Transpose() + R).Inverse();

            //Joseph Form
            Matrix<double> A = (Matrix<double>.Build.DiagonalIdentity(K.RowCount, H.ColumnCount) - K * H);
            P_po = A * P_pr * A.Transpose() + K * R * K.Transpose();
            //LAExtensions.PrintMatrix(K);
            //// Standard form
            //P_po = (Matrix<double>.Build.DiagonalIdentity(K.RowCount, H.ColumnCount) - K * H) * P_pr;

            //LAExtensions.PrintMatrix(K);
            //Debug.WriteLine(K.SubMatrix(9, 9, 0, 3));

            //P_po = (Matrix<double>.Build.SparseIdentity(18) - K * H) * P_pr;// (H * P_pr * H.Transpose() + R) * K.Transpose();

            //Debug.WriteLine(P_po.ToString());
            //Debug.WriteLine(q);
            //Vector<double> obs_prediction = Vector<double>.Build.Dense(obs_size);
            //obs_prediction.SetSubVector(0, 3, q_rot * g);
            //obs_prediction.SetSubVector(0, 3, q_rot * mag_ref);
            //Debug.WriteLine("Observed: " + obs.ToString());
            //Debug.WriteLine("Estimate: "  + obs_prediction.ToString());

            state = K * (obs - obs_prediction);
            //Debug.WriteLine(obs-obs_prediction);
            //Debug.WriteLine(state);

        }

        protected void ResetError()
        {
            // Orientation error is
            // x - 6
            // y - 7
            // z - 8
            //G[6, 7] = -0.5 * state[5];
            //G[7, 6] = 0.5 * state[5];
            //G[6, 8] = 0.5 * state[4];
            //G[8, 6] = -0.5 * state[4];
            //G[7, 8] = -0.5 * state[3];
            //G[8, 7] = 0.5 * state[3];

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
            F.SetSubMatrix(0, 3, Matrix<double>.Build.Diagonal(3, 3, dt));

            //F.SetSubMatrix(3, 3, LAExtensions.RotationFromAngleAxis(-(om_m_t - om_b) * dt).Transpose());
            //F.SetSubMatrix(3, 6, Matrix<double>.Build.Diagonal(3, 3, -dt));
            F.SetSubMatrix(3, 6, -q_rot * dt);

            //LAExtensions.PrintMatrix(F);
            Q[3, 3] = dt * dt * var_a_nx * var_a_nx;
            Q[4, 4] = dt * dt * var_a_ny * var_a_ny;
            Q[5, 5] = dt * dt * var_a_nz * var_a_nz;

            Q[6, 6] = dt * var_a_wx * var_a_wx;
            Q[7, 7] = dt * var_a_wy * var_a_wy;
            Q[8, 8] = dt * var_a_wz * var_a_wz;

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

        public override void InitializePosition(Vector<double> s)
        {
            p = s;
        }
        public override void InitializeOrientation(Quaternion s)
        {
            q = s;
            q_rot = LAExtensions.RotationFromQuat(q);
        }

        public override void InitializeStates(Vector<double> s)
        {
            state = s;
        }

        public override Vector<double> GetState()
        {
            return state;
        }
    }
}