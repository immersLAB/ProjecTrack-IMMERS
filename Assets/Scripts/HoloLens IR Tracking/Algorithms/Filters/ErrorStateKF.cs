using MathNet.Numerics.LinearAlgebra;
using MathNet.Spatial.Euclidean;
using System.Collections.Generic;
using System.Diagnostics;

namespace StereoSensors.Tracking
{
    abstract class ErrorStateKF : IKalmanFilter
    {
        protected Vector<double> state;

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

        public abstract void InitializeStates(Vector<double> state);
        public abstract void InitializePosition(Vector<double> p);
        public abstract void InitializeOrientation(Quaternion q);

        public abstract Quaternion QueryOrientation();
        public abstract Vector<double> QueryPosition();

        public abstract void UpdateNominal(Vector<double> meas, double dt);
        public abstract void InjectError();
        public abstract void UpdatePrior(double dt);
        public abstract void UpdatePosterior(Vector<double> obs, double dt);
        public abstract Vector<double> GetState();
    }
}