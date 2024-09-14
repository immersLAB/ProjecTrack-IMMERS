using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Spatial.Euclidean;
using MathNet.Numerics.Distributions;
using System.Diagnostics;
using Unity.VisualScripting;
using UnityEngine;

// Very jank solver that somehow works 
static class GeometryProbabilitySolver
{
    public static RegistrationData Register(PointCloud s, PointCloud d, double thresh = 0.003)
    {
        //Vector<double> distance = Vector<double>.Build.Dense(s.Count);

        Matrix<double> refDist = Matrix<double>.Build.Dense(s.Count, s.Count);//<double>.Build.Dense(s.Count);

        for (int i = 0; i < s.Count; i++)
        {
            for (int j = i; j < s.Count; j++)
            {
                refDist[i, j] = (s.GetPoint(i) - s.GetPoint(j)).Length;
                refDist[j, i] = refDist[i, j];            }
        }

        Matrix<double> weights = Matrix<double>.Build.Dense(s.Count, d.Count);

        // Select point in destination cloud
        for (int i = 0; i < weights.RowCount; i++)
        {
            Vector<double> distance = Vector<double>.Build.Dense(s.Count);
            // Compute pairwise distances
            for (int j = 0; j < weights.RowCount; j++)
            {
                if (i == j) continue;
                distance[j] = (d.GetPoint(i) - d.GetPoint(j)).Length;

                // Assume current point i is one of the model points
                for (int k = 0; k < weights.ColumnCount; k++)
                {
                    // Computed weighting for observing point i and j, assuming i corresponds to source point k and j to source point l
                    for (int l = 0; l < weights.ColumnCount; l++)
                    {
                        if ((distance[j] - refDist[k, l]) * (distance[j] - refDist[k, l]) < thresh * thresh)
                        {
                            weights[i, k] += 1;
                        }
                    }
                }
            }
        }

        //List<Vector3D> filteredD = new List<Vector3D>();
        //List<int> indices = new List<int>();

        //for (int i = 0; i < d.Count; i++)
        //{
        //    Vector3D refPoint = d.GetPoint(i);
        //    filteredD.Clear();
        //    indices.Clear();
        //    for (int j = 0; j < d.Count; j++)
        //    {                
        //        double testDist = (d.GetPoint(j) - refPoint).Length;
        //        // Check for matches
        //        for (int k = 0; k < distance.Count; k++)
        //        {
        //            double refDist = distance[k];
        //            if ((refDist - testDist) * (refDist - testDist) < thresh * thresh)
        //            {
        //                // Added point corresponds to k'th position of source marker cloud
        //                filteredD.Add(d.GetPoint(j));
        //                indices.Add(k);
        //                break;
        //            }
        //        }
        //    }
        //    if (filteredD.Count <= 2)
        //        continue;

        //    if (filteredD.Count >= s.Count -1)
        //        break;
        //}

        //if (filteredD.Count < s.Count - 1 || filteredD.Count <= 2)
        //{
        //    RegistrationData identity = new RegistrationData();
        //    identity.t = new Vector3D();
        //    identity.R = Matrix<double>.Build.DenseIdentity(3, 3) ;
        //    return identity;
        //}
        //List<Vector3D> orderedS = new List<Vector3D>();

        //foreach (int idx in indices)
        //{
        //    orderedS.Add(s.GetPoint(idx));
        //}
        
        return LeastSquaresReg.Register(s,s);
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
}
