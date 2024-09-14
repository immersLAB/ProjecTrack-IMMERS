using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace StereoSensors.Tracking.Filters
{
    class ExtendedKF : IKalmanFilter
    {
        public Vector<double> GetState()
        {
            throw new NotImplementedException();
        }

        public void UpdatePosterior(Vector<double> obs, double dt)
        {
            throw new NotImplementedException();
        }

        public void UpdatePrior(double dt)
        {
            throw new NotImplementedException();
        }
    }
}
