using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
namespace StereoSensors.Tracking
{
    interface IKalmanFilter
    {
        public abstract void UpdatePrior(double dt);
        public abstract void UpdatePosterior(Vector<double> obs, double dt);
        public abstract Vector<double> GetState();
    }
}
