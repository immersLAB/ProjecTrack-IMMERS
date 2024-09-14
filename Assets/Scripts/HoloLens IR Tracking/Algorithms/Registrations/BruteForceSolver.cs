using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Spatial.Euclidean;
using System.Diagnostics;
using UnityEngine.UIElements;

//There's a smarter way to do this; I just haven't figured it out yet
static class BruteForceSolver
{
    public static RegistrationData Register(PointCloud s, PointCloud d)
    {
        Matrix<double> R = Matrix<double>.Build.DenseIdentity(3, 3);
        Vector3D tr = new Vector3D();
        K3DTree t = new K3DTree(d.GetPointsAsList());
        return Register(s, d, R, tr, t);
    }

    public static RegistrationData Register(PointCloud s, PointCloud d, Matrix<double> R, Vector3D tr, K3DTree t)
    {
        List<Vector3D> srcCopy = s.GetPointsAsList();
        List<Vector3D> dstCopy = d.GetPointsAsList();

        List<double> distMap = new List<double>(dstCopy.Count - 1);
        Matrix<double> distMat = Matrix<double>.Build.Dense(4,4);

        for (int i = 0; i < dstCopy.Count; i++)
        {
            for (int j = i; j < dstCopy.Count; j++)
            {
                distMat[i, j] = (dstCopy[i] - dstCopy[j]).DotProduct(dstCopy[i] - dstCopy[j]);
                distMat[j, i] = distMat[i, j];
            }
        }

        for (int i = 0; i < srcCopy.Count; i++)
        {
            List<double> distances = new List<double>();
            List<double> distCandidates = new List<double>();
            for (int j = 0; j < srcCopy.Count; j++)
            {
                if(i == j) { continue; }
                double dist = (srcCopy[i] - dstCopy[j]).DotProduct(srcCopy[i] - dstCopy[j]);
                
            }
        }

        RegistrationData result = new RegistrationData();
        result.R = R;
        result.t = tr;
        result.errorSq = ComputeErrorSq(s, d, R, tr, t);
        result.error = ComputeError(s, d, R, tr, t);

        return result;
    }

    private static List<Tuple<int, int>> FindMatchedPairs(double dist, Matrix<double> distMat)
    {
        List<Tuple<int,int>> candidates = new List<Tuple<int,int>>();
        for (int i = 0; i < distMat.RowCount; i++)
        {
            for (int j = i+1; j < distMat.RowCount; j++)
            {
                if (dist - distMat[i, j] < 0.015)
                {
                    candidates.Add(new Tuple<int, int>(i, j));
                }
            }
        }
        return candidates;
    }
    // Compute the error between two given point clouds and the transform
    public static double ComputeErrorSq(PointCloud s, PointCloud d, Matrix<double> rot, Vector3D tr, K3DTree t)
    {
        List<Vector3D> srcList = s.GetPointsAsList();
        List<Vector3D> dstOrdered = new List<Vector3D>();

        double error = 0;
        foreach (Vector3D sp in srcList)
        {
            Vector3D tP = sp.TransformBy(rot) + tr;
            Vector3D closest = t.FindClosest(tP);
            double errorTest = (closest-tP).DotProduct(closest - tP);
            error += errorTest;
        }
        return error;

    }

    public static double ComputeError(PointCloud s, PointCloud d, Matrix<double> rot, Vector3D tr, K3DTree t)
    {
        List<Vector3D> srcList = s.GetPointsAsList();
        List<Vector3D> dstOrdered = new List<Vector3D>();

        double error = 0;
        foreach (Vector3D sp in srcList)
        {
            Vector3D tP = sp.TransformBy(rot) + tr;
            Vector3D closest = t.FindClosest(tP);
            error += Math.Sqrt((closest - tP).DotProduct(closest - tP));
        }
        return error;
    }

    private static TransformData Step(PointCloud s, PointCloud d, K3DTree t)
    {
        TransformData cloudTf = new TransformData();
        List<Vector3D> srcList = s.GetPointsAsList();
        List<Vector3D> dstOrdered = new List<Vector3D>();

        foreach (Vector3D sp in srcList)
        {
            //Debug.WriteLine(sp.ToString());
            //Debug.WriteLine(t.FindClosest(sp).ToString());
            dstOrdered.Add(t.FindClosest(sp));

        }

        PointCloud dstMatch = new PointCloud(dstOrdered); // List<Vec3>();

        List<Vector3D> centeredSrc = s.ZeroCenteredPoints();
        List<Vector3D> centeredDst = dstMatch.ZeroCenteredPoints();

        Matrix<double> cc = Matrix<double>.Build.Dense(3, 3);
        for (int i = 0; i < centeredSrc.Count; i++)
        {
            cc += centeredDst[i].ToVector().OuterProduct(centeredSrc[i].ToVector());
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

        //Vector<double> muSrc = Vector<double>.Build.DenseOfArray(new double[] { s.Centroid.X, s.Centroid.Y, s.Centroid.Z });
        //Vector<double> muDst = Vector<double>.Build.DenseOfArray(new double[] { dstMatch.Centroid.X, dstMatch.Centroid.Y, dstMatch.Centroid.Z });

        Vector3D tra = dstMatch.Centroid - s.Centroid.TransformBy(R);
        //Matrix<double> cloudTf = Matrix<double>.Build.DenseOfArray(new double[,] {{R[0, 0], R[1, 0], R[2, 0], tra.X },
        //                            { R[0, 1], R[1, 1], R[2, 1], tra.Y },
        //                            { R[0, 2], R[1, 2], R[2, 2], tra.Z },
        //                            { 0,0,0, 1 }});
        cloudTf.R = R;
        cloudTf.t = tra;
        return cloudTf;
    }
}
