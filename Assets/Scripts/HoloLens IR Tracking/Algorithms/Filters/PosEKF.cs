using MathNet.Numerics.LinearAlgebra;
using MathNet.Spatial.Euclidean;
using System.Collections.Generic;
using System.Diagnostics;

namespace StereoSensors.Tracking
{
    class PosEKF : IKalmanFilter
    {
        // Noise
        private readonly double var_a_nx, var_a_ny, var_a_nz;
        private readonly double var_om_nx, var_om_ny, var_om_nz;

        // Bias noise
        private readonly double var_a_wx, var_a_wy, var_a_wz;
        private readonly double var_om_wx, var_om_wy, var_om_wz;

        // Nominal variables
        protected Vector<double> state;
        protected Vector<double> state_est;

        // Prior
        protected Matrix<double> P_pr;

        // Posterior
        protected Matrix<double> P_po;

        // Kalman Gain
        protected Matrix<double> K;

        // State Transition Matrix
        protected Matrix<double> F;

        // Observation Matrix
        protected Matrix<double> H;

        // Process Noise Covariance
        protected Matrix<double> Q;

        // Observation Noise Covariance
        protected Matrix<double> R;

        // Gravity
        private Vector<double> g;

        private List<Vector<double>> marker_refs;
        
        private int state_size = 9;

        // Markers
        private int obs_size = 6;
        //private int nominal_state_size = 15+1
        public PosEKF()
        {
            // Define initial error state
            state = Vector<double>.Build.Dense(state_size);
            state_est = Vector<double>.Build.Dense(state_size);

            // Define initial nominal states
            //p = Vector<double>.Build.Dense(3);
            //v = Vector<double>.Build.Dense(3);
            //a_b = Vector<double>.Build.Dense(3); ;
            g = Vector<double>.Build.DenseOfArray(new double[] { 0, -9.80665, 0 });
            //g = Vector<double>.Build.DenseOfArray(new double[] { 0, 0, 0 });


            // State covariances
            P_pr = Matrix<double>.Build.DenseIdentity(state_size, state_size);
            P_pr.SetSubMatrix(0, 0,  Matrix<double>.Build.Dense(6,6));// = Matrix<double>.Build.DenseIdentity(state_size, state_size);
            P_po = Matrix<double>.Build.DenseIdentity(state_size, state_size);
            P_po.SetSubMatrix(0, 0, Matrix<double>.Build.Dense(3, 3));// = Matrix<double>.Build.DenseIdentity(state_size, state_size);

            // Process nad measurement covariances
            Q = Matrix<double>.Build.Sparse(state_size, state_size);
            var_a_nx = 0.020669209162667224 * 0.020669209162667224; var_a_ny = 0.018070485871372426* 0.018070485871372426; var_a_nz = 0.021218359826934503 * 0.021218359826934503;
            var_a_wx = 0.02060301949481104* 0.02060301949481104; var_a_wy = 0.014218195857796866* 0.014218195857796866; var_a_wz = 0.02120327470670869* 0.02120327470670869;


            R = Matrix<double>.Build.Sparse(obs_size, obs_size);
            R[0, 0] = 0.0002510462;
            R[1, 1] = 0.0002510937;
            R[2, 2] = 0.0002534030;

            R[3, 3] = 0.0002510462;
            R[4, 4] = 0.0002510937;
            R[5, 5] = 0.0002534030;

            //LAExtensions.PrintMatrix(G);

            F = Matrix<double>.Build.SparseIdentity(state_size, state_size);

            marker_refs = new List<Vector<double>>();
            Vector<double> m1 = Vector<double>.Build.DenseOfArray(new double[] { 0.07125, 0, 0});
            Vector<double> m2 = Vector<double>.Build.DenseOfArray(new double[] { 0.02125, 0, 0});
            Vector<double> m3 = Vector<double>.Build.DenseOfArray(new double[] { -0.06375, 0, 0.03 });
            Vector<double> m4 = Vector<double>.Build.DenseOfArray(new double[] { -0.02875,0, -0.03 });
            marker_refs.Add(m1); marker_refs.Add(m2); marker_refs.Add(m3); marker_refs.Add(m4);
        }

        public void UpdatePosterior(Vector<double> obs, double dt)
        {
            H = Matrix<double>.Build.Dense(obs.Count, state.Count);
            H.SetSubMatrix(0, 0, Matrix<double>.Build.DiagonalIdentity(3));
            H.SetSubMatrix(0, 3, Matrix<double>.Build.DiagonalIdentity(3));

            Vector<double> obs_prediction = Vector<double>.Build.Dense(obs.Count);
            obs_prediction = H * state_est;

            //Debug.WriteLine("Prediciton " + obs_prediction.ToString());
            //Debug.WriteLine(obs);

            K = P_pr * H.Transpose() * (H * P_pr * H.Transpose() + R).Inverse();

            //Joseph Form
            Matrix<double> A = (Matrix<double>.Build.DiagonalIdentity(K.RowCount, H.ColumnCount) - K * H);
            P_po = A * P_pr * A.Transpose() + K * R * K.Transpose();

            state = state_est + K * (obs - obs_prediction);
            //Debug.WriteLine(obs);
            //Debug.WriteLine(state);
            //Debug.WriteLine(R_m_t * (a_m_t - state.SubVector(6, 3)) + g);
            //Debug.WriteLine(state);

            //Debug.WriteLine(state);
            //Debug.WriteLine(obs);
            //Debug.WriteLine(obs_prediction);
            //Debug.WriteLine(R_m_t * (a_m_t - state.SubVector(6, 3)) + g);
            //Debug.WriteLine(obs-obs_prediction);
            //Debug.WriteLine(state);

        }

        public void UpdatePrior(double dt)
        {
            Vector<double> p = state.SubVector(0,3);
            Vector<double> v = state.SubVector(3,3);
            Vector<double> a_b = state.SubVector(6, 3);

            //state_est.SetSubVector(0, 3, p + v * dt + 0.5 * (R_m_t * (a_m_t - a_b) + g) * dt * dt);
            //state_est.SetSubVector(3, 3, v + (R_m_t * (a_m_t - a_b) + g) * dt);
            //state_est.SetSubVector(6, 3, a_b);

            //F.SetSubMatrix(0, 3, Matrix<double>.Build.Diagonal(3, 3, dt));
            //F.SetSubMatrix(0, 6, -0.5 * R_m_t * dt * dt);
            //F.SetSubMatrix(3, 6, -R_m_t * dt);
            //F.SetSubMatrix(6, 6, -R_m * dt));

            //F.SetSubMatrix(3, 3, LAExtensions.RotationFromAngleAxis(-(om_m_t - om_b) * dt).Transpose());
            //F.SetSubMatrix(3, 6, Matrix<double>.Build.Diagonal(3, 3, -dt));
            //F.SetSubMatrix(3, 6, -q_rot * dt);

            //LAExtensions.PrintMatrix(F);
            Q[0, 0] = 0.001;
            Q[1, 1] = 0.001;
            Q[2, 2] = 0.001;

            Q[3, 3] = dt * dt * var_a_nx * var_a_nx;
            Q[4, 4] = dt * dt * var_a_ny * var_a_ny;
            Q[5, 5] = dt * dt * var_a_nz * var_a_nz;

            Q[6, 6] = dt * var_a_wx * var_a_wx;
            Q[7, 7] = dt * var_a_wy * var_a_wy;
            Q[8, 8] = dt * var_a_wz * var_a_wz;

            //LAExtensions.PrintMatrix(Q);

            P_pr = F * P_po *F.Transpose() +  Q ;
        }

        public void SetControlInput(Vector<double> a, Quaternion q)
        {
            //a_m_t = a;
            //R_m_t = LAExtensions.RotationFromQuat(q);
        }
        public Vector<double> GetState()
        {
            return state;
        }
        public Vector<double> QueryPosition()
        {
            return state.SubVector(0, 3);
        }
        public void InitializePosition(Vector<double> pos)
        {
            state.SetSubVector(0, 3, pos);
        }

    }
}