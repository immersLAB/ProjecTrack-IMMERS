using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Spatial.Euclidean;
using System.Diagnostics;

public struct RegistrationData 
{
    public Matrix<double> R;
    public Vector3D t;
    public double errorSq;
    public double error;
}
public struct TransformData
{
    public Matrix<double> R;
    public Vector3D t;
}

static class ICPSolver
{
    public static RegistrationData Register(PointCloud s, PointCloud d)
    {
        Matrix<double> R = Matrix<double>.Build.DenseIdentity(3,3);
        Vector3D tr = new Vector3D();
        K3DTree t = new K3DTree(d.GetPointsAsList());
        return Register(s, d, R, tr, t);
    }
    public static RegistrationData Register(PointCloud s, PointCloud d, Matrix<double> offset)
    {
        Matrix<double> R = Matrix<double>.Build.DenseIdentity(3, 3);
        Vector3D tr = new Vector3D();

        K3DTree t = new K3DTree(d.GetPointsAsList());
        return Register(s, d, R, tr, t);
    }

    public static RegistrationData Register(PointCloud s, PointCloud d, K3DTree t)
    {
        Matrix<double> R = Matrix<double>.Build.DenseIdentity(3, 3);
        Vector3D tr = new Vector3D();
        return Register(s, d, R, tr, t);
    }

    public static RegistrationData Register(PointCloud s, PointCloud d, Matrix<double> R, Vector3D tr, K3DTree t)
    {
        PointCloud srcCopy = new PointCloud(s.GetPointsAsList());
        srcCopy.TransformCloud(R, tr);
        for (int i = 0; i < 5; i++)
        {
            TransformData currentSol = Step(srcCopy, d, t);
            srcCopy.TransformCloud(currentSol.R, currentSol.t);
            R = currentSol.R * R;
            tr = tr.TransformBy(currentSol.R) + currentSol.t;
        }

        RegistrationData result = new RegistrationData();
        result.R = R;
        result.t = tr;
        result.errorSq = ComputeErrorSq(s, d, R, tr, t);
        result.error = ComputeError(s, d, R, tr, t);

        return result;
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
        List<Vector3D> inlierDstOrdered = new List<Vector3D>();

        double avgPairedDist = 0;

        foreach (Vector3D sp in srcList)
        {
            Vector3D closestPt = t.FindClosest(sp);
            dstOrdered.Add(closestPt);
            avgPairedDist += (closestPt-sp).Length;
        }

        avgPairedDist /= srcList.Count();

        List<Vector3D> inlierSrcList = new List<Vector3D>();

        PointCloud dstMatch = new PointCloud(dstOrdered);

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
