using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Spatial.Euclidean;
public class PointCloud
{
    List<Vector3D> _points;
    Vector3D _pntSum;
    Vector3D _centroid;
    public int Count
    {
        get { return _points.Count; }
    }

    public Vector3D Centroid
    {
        get { return _centroid; }
    }
    private Quaternion _orientation;
    public PointCloud()
    {
        _points = new List<Vector3D>();
        _centroid = new Vector3D();
        _pntSum = new Vector3D();
        _orientation = Quaternion.One;
    }

    public PointCloud(List<Vector3D> p)
    {
        _points = new List<Vector3D>();
        _orientation = Quaternion.One;
        _pntSum = new Vector3D();
        foreach (Vector3D point in p)
            _points.Add(point);
        ComputeCentroid();
    }

    private void ComputeCentroid()
    {
        _centroid = new Vector3D();
        _pntSum = new Vector3D();
        foreach (Vector3D p in _points)
        {
            _pntSum += p;
        }
        _centroid = _pntSum/_points.Count;
    }
    public void AddPoint(Vector3D p)
    {
        _points.Add(p);
        _pntSum += p;
        _centroid = _pntSum / _points.Count;//   .ScaleBy(1.0/_points.Count);
    }
    // Take a matrix M and applies it to the local transform of the point cloud with 4x4 representation
    public void TransformCloud(Matrix<double> m)
    {
        for(int i = 0; i < _points.Count; i++)
        {
            _points[i] = _points[i].TransformBy(m.SubMatrix(0, 3, 0, 3)) + new Vector3D(m[0, 3], m[1, 3], m[2, 3]); 
        }
        _centroid = _centroid.TransformBy(m.SubMatrix(0, 3, 0, 3)) + new Vector3D(m[0, 3], m[1, 3], m[2, 3]); ;
    }
    // Apply 3x3 rotation and 3x1 translation
    public void TransformCloud(Matrix<double> r, Vector3D t)
    {
        for (int i = 0; i < _points.Count; i++)
        {
            _points[i] = _points[i].TransformBy(r) + t;
        }
        _centroid = _centroid.TransformBy(r) + t;
    }
    // Apply quaternion and 3x1 translation
    public void TransformCloud(Quaternion q, Vector3D t)
    {
        for (int i = 0; i < _points.Count; i++)
        {
            _points[i] = QuatMultVector(q, _points[i]) + t;
        }
        _centroid = QuatMultVector(q, _centroid) + t;
    }

    public List<Vector3D> ZeroCenteredPoints()
    {
        List<Vector3D> zeroCenteredPoints = new List<Vector3D>();
        foreach (Vector3D p in _points)
        {
            zeroCenteredPoints.Add(p - _centroid);
        }
        //_centroid = new Vector3D(0, 0, 0);
        return zeroCenteredPoints;
    }
    public List<Vector3D> PointsCenteredAt(Vector3D v)
    {
        List<Vector3D> vCenteredPoints = new List<Vector3D>();
        foreach (Vector3D p in _points)
        {
            vCenteredPoints.Add(p - _centroid + v);
        }
        //_centroid = v;
        return vCenteredPoints;
    }
    public List<Vector3D> GetPointsAsList()
    {
        return _points.ToList();
    }
    public Vector3D GetPoint(int i)
    {
        return _points[i];
    }

    public void SetPoint(Vector3D v, int i)
    {
        _points[i] = v;
    }

    public override string ToString()
    {
        string s = "";
        foreach (Vector3D v in _points)
        {
            s += String.Format("[{0}, {1}, {2}]", v.X.ToString("0.000000"), v.Y.ToString("0.000000"), v.Z.ToString("0.000000")) + "\n";
        }
        return s;
    }

    public PointCloud SubcloudInRegion(Vector3D p, double radius)
    {
        List<Vector3D> pointsInRegion = new List<Vector3D>();
        foreach (Vector3D v in _points)
        {
            if ((v - p).DotProduct(v - p) <= radius * radius)
            {
                pointsInRegion.Add(v);
            }
        }
        return new PointCloud(pointsInRegion);
    }

    private Vector3D QuatMultVector(Quaternion q, Vector3D v)
    {
        Vector3D qVec = new Vector3D(q.ImagX, q.ImagY, q.ImagZ);
        Vector3D tVal = 2 * (new Vector3D(q.ImagX, q.ImagY, q.ImagZ).CrossProduct(v));
        return v + q.Real * tVal + qVec.CrossProduct(tVal);
    }
}
