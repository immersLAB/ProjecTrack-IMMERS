using MathNet.Spatial.Euclidean;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Xml;
using System.Xml.Linq;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.UtilsModule;
using UnityEngine;
using System.Collections.ObjectModel;
using MathNet.Numerics.Integration;
using OpenCVForUnity.Calib3dModule;
using static UnityEngine.EventSystems.EventTrigger;
using OpenCVForUnity.UnityUtils;

public class CameraIntrinsicsLoader
{

    private Mat _cameraIntrinsicMat;
    public Mat CameraIntrinsicMat
    {
        get { return _cameraIntrinsicMat;  }
    }

    private Mat _distortionCoefs;
    public Mat DistortionCoefs
    {
        get { return _distortionCoefs; }
    }


    public CameraIntrinsicsLoader(TextAsset xmlFile)
    {
        //TextAsset textAsset = (TextAsset)Resources.Load(path);
        XElement intrinsics = XElement.Parse(xmlFile.text);


        IEnumerable<string> test = from item in intrinsics.Elements("Transform").Elements("double") select (string)item.Value;

        IEnumerable<double> intrinMatrix = from item in intrinsics.Elements("Transform").Elements("double") select Convert.ToDouble((string)item.Value);

        IEnumerable<double> rDCoefs = from item in intrinsics.Elements("RadialDistortion").Elements("double") select Convert.ToDouble((string)item.Value);

        IEnumerable<double> tDCoefs = from item in intrinsics.Elements("TangentialDistortion").Elements("double") select Convert.ToDouble((string)item.Value);

        _cameraIntrinsicMat = new Mat(3, 3, CvType.CV_64F);
        //double[] camParams = new double[9];

        //int i = 0;
        //foreach (double item in intrinMatrix)
        //{
        //    camParams[i++] = item;
        //}

        MatUtils.copyToMat(intrinMatrix.ToArray(), _cameraIntrinsicMat);
        _cameraIntrinsicMat = _cameraIntrinsicMat.t();
        double[] distortArray = new double[] {rDCoefs.ElementAt(0), rDCoefs.ElementAt(1), tDCoefs.ElementAt(0), tDCoefs.ElementAt(1), rDCoefs.ElementAt(2), rDCoefs.ElementAt(3), rDCoefs.ElementAt(4), rDCoefs.ElementAt(5)};
        _distortionCoefs = new Mat(8, 1, CvType.CV_64F);
        MatUtils.copyToMat(distortArray, _distortionCoefs);
    }

    public Vector3D Unproject(Point2D point, double depth, bool depthIsDistance)
    {

        double normalizedX = (point.X - _cameraIntrinsicMat.get(0, 2)[0]) / _cameraIntrinsicMat.get(0, 0)[0];
        double normalizedY = (point.Y - _cameraIntrinsicMat.get(1, 2)[0]) / _cameraIntrinsicMat.get(1, 1)[0];

        //Point centroid = new Point(normalizedX, normalizedY);
        Point centroid = new Point(point.X, point.Y);
        MatOfPoint2f distortedPt = new MatOfPoint2f(centroid);
        MatOfPoint2f undistortedPt = new MatOfPoint2f(new Point(0, 0));

        Calib3d.undistortPoints(distortedPt, undistortedPt, CameraIntrinsicMat, DistortionCoefs);
        //Debug.Log(_cameraIntrinsicMat.ToString());

        //Debug.Log(undistortedPt.);
        centroid = undistortedPt.toArray()[0];

        Vector3D normalizedPoint = new Vector3D(centroid.x, centroid.y, 1);

        if (depthIsDistance)
        {
            return normalizedPoint.Normalize().ScaleBy(depth);
        }

        return normalizedPoint.ScaleBy(depth);

    }
}
