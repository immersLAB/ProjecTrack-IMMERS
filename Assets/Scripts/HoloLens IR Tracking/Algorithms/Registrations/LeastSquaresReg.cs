using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Spatial.Euclidean;
public static class LeastSquaresReg
{
    // Finds the transform that transforms each centered point in the source cloud
    // to each point in the destination cloud.
    public static RegistrationData Register(PointCloud src, PointCloud dst)
    {
        // Get zero-centered points
        List<Vector3D> centeredSrc = src.ZeroCenteredPoints();
        List<Vector3D> centeredDst = dst.ZeroCenteredPoints();

        // Matching
        int minCount = (src.Count > dst.Count) ? dst.Count : src.Count;
        Matrix<double> cc = Matrix<double>.Build.Dense(3, 3);
        for (int i = 0; i < minCount; i++)
        {
            Vector<double> pairedSrc = Vector<double>.Build.DenseOfArray(new double[] { centeredSrc[i].X, centeredSrc[i].Y, centeredSrc[i].Z });
            Vector<double> pairedDst = Vector<double>.Build.DenseOfArray(new double[] { centeredDst[i].X, centeredDst[i].Y, centeredDst[i].Z });
            cc += pairedDst.OuterProduct(pairedSrc);
            //centeredDst.RemoveAt(jdx);
        }
        // SVD
        Svd<double> decomposition = cc.Svd();
        Matrix<double> R = decomposition.U * decomposition.VT;
        if (R.Determinant() < 0)
        {
            decomposition.VT.At(2, 0, -1 * decomposition.VT.At(2, 0));
            decomposition.VT.At(2, 1, -1 * decomposition.VT.At(2, 1));
            decomposition.VT.At(2, 2, -1 * decomposition.VT.At(2, 2));
            R = decomposition.U * decomposition.VT;
        }

        Vector<double> muSrc = Vector<double>.Build.DenseOfArray(new double[] { src.Centroid.X, src.Centroid.Y, src.Centroid.Z });
        Vector<double> muDst = Vector<double>.Build.DenseOfArray(new double[] { dst.Centroid.X, dst.Centroid.Y, dst.Centroid.Z });

        Vector<double> t = muDst - R * muSrc;

        RegistrationData result = new RegistrationData();
        result.R = R;
        result.t = Vector3D.OfVector(t);

        //_transformMatrix = new Matrix<double>.Build.DenseOfArray(new float[,] { R{[0, 0], R[1, 0], R[2, 0], 0},
        //                            {R[0, 1], R[1, 1], R[2, 1], 0}},
        //                            { R[0, 2], R[1, 2], R[2, 2], 0}},
        //                            { t[0], t[1], t[2], 1} });
        //_errorSq = 0;
        //for (int i = 0; i < src.Count; i++)
        //{
        //    _errorSq += (_transformMatrix.Transform(src.GetPoint(i).Position) - dst.GetPoint(i).Position).LengthSq;
        //}
        return result;
    }
}
