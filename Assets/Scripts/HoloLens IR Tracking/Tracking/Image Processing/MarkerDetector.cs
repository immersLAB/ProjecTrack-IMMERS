using MathNet.Numerics.Differentiation;
using MathNet.Spatial.Euclidean;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ImgprocModule;
using OpenCVForUnity.UnityUtils;
using OpenCVForUnity.UnityUtils.Helper;

using UnityEngine.Playables;
using TMPro;
using OpenCVForUnity.UtilsModule;
using OpenCVForUnity.Calib3dModule;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine.Events;

public class MarkerDetector : MonoBehaviour
{
    //public event EventHandler<(PointCloud, Envelope)> MarkersProcessed;

    // double markerRadius = 0.0127 / 2.0;
    double markerRadius = 0.014 / 2.0;

    [SerializeField]
    bool enableDebugPlanes;

    [SerializeField]

    float areaThresh = 6, circularityThresh = 0.6f;

    private Material debugMediaMaterial = null;
    private Texture2D debugMediaTexture = null;
    public GameObject debugPlane;
   
    private Material debug2MediaMaterial = null;
    private Texture2D debug2MediaTexture = null;
    private Mat img, imgRGB;
    public GameObject debug2Plane;
    [SerializeField]
    public TextAsset cameraXML;
    public Vector3D test = new Vector3D();
     UnityEngine.Quaternion testQ = new UnityEngine.Quaternion();
        private CameraIntrinsicsLoader AHATCamParams;

    object pointlock = new object();
    
    public bool displayImage = false;
    List<Vector3D> _markers;
    private bool texturesUpdated = false;


    public UnityEvent<PointCloud, long> OnMarkersProcessed;

    public List<Vector3D> Markers
    {
        get
        {
            lock (pointlock)
            {
                List<Vector3D> clone = new List<Vector3D>();
                foreach (Vector3D c in _markers)
                {
                    clone.Add(c);
                }
                return clone;
            }
        }
    }

    public K3DTree MarkerTree
    {
        get
        {
            lock (pointlock)
            {
                return new K3DTree(_markers);
            }
        }
    }

    private void Awake()
    {

        AHATCamParams = new CameraIntrinsicsLoader(cameraXML);
    }
    void Start()
    {

        if (debugPlane != null)
        {
            debugPlane.SetActive(enableDebugPlanes);
            debugMediaMaterial = debugPlane.GetComponent<MeshRenderer>().material;
            debugMediaTexture = new Texture2D(512, 512, TextureFormat.Alpha8, false);
            debugMediaMaterial.mainTexture = debugMediaTexture;
        }

        if (debug2Plane != null)
        {
            debug2Plane.SetActive(enableDebugPlanes);
            debug2MediaMaterial = debug2Plane.GetComponent<MeshRenderer>().material;
            debug2MediaTexture = new Texture2D(512, 512, TextureFormat.RGBA32, false);
            debug2MediaMaterial.mainTexture = debug2MediaTexture;

        }
    }

    void Update()
    {
        if (enableDebugPlanes && texturesUpdated)
        {
            lock (pointlock)
            {

                Utils.matToTexture2D(img, debugMediaTexture);
                ////debugMediaTexture.LoadRawTextureData(img.);
                //debugMediaTexture.Apply();


                Utils.matToTexture2D(imgRGB, debug2MediaTexture);
            }

            //Utils.matToTexture2D(img, debugMediaTexture);
            ////debugMediaTexture.LoadRawTextureData(img.);
            debugMediaTexture.Apply();


                //Utils.matToTexture2D(imgRGB, debug2MediaTexture);
                ////debugMediaTexture.LoadRawTextureData(img.);
                debug2MediaTexture.Apply();
                texturesUpdated = false;
             
        }
    }

    public void ReadCamera(AHATImageData imageData)
    {
        //debugTxt.text = path;// AHATCamParams.DistortionCoefs.dump() ;
        int _imgHeight = 512;
        int _imgWidth = 512;
        if (imageData.image == null)
            return;

        Matrix<double> extrinRot = imageData.extrinT;// LAExtensions.RotationFromQuat(imageData.extrinRot) ;// Matrix<double>.Build.DenseIdentity(3, 3);
        Vector3D extrinTra = imageData.extrinTra;

        lock (pointlock)
        {

            img = ProcessImage(_imgHeight, _imgWidth, imageData.image, AHATCamParams);
            imgRGB = new Mat(_imgHeight, _imgWidth, CvType.CV_8UC4);

            Imgproc.cvtColor(img, imgRGB, Imgproc.COLOR_GRAY2BGRA);

            List<MatOfPoint> p = new List<MatOfPoint>();
            Mat H = new Mat();
            Imgproc.findContours(img, p, H, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
            List<Mat> circContours = new List<Mat>();
            List<Vector3D> detectedMarkers = new List<Vector3D>();
            _markers = new List<Vector3D>();

            int idx = 0;
            for (int i = 0; i < p.Count; i++)
            {
                Moments m = Imgproc.moments(p[i]);
                if (m.get_m00() == 0)
                    continue;
                double centX = m.get_m10() / m.get_m00();
                double centY = m.get_m01() / m.get_m00();
                MatOfPoint2f p2F = new MatOfPoint2f();
                p[i].convertTo(p2F, CvType.CV_32F);
                double arcLength = Imgproc.arcLength(p2F, true);
                double area = Imgproc.contourArea(p[i]);
                double circularity = 4 * Math.PI * area / Math.Pow(arcLength, 2);
                double rContour = Math.Sqrt(area / Math.PI);
                if (area < areaThresh || circularity < circularityThresh)
                {
                    continue;
                }
                
                Vector3D camPt = GetCameraSpace3DPoint(_imgWidth, _imgHeight, imageData.depth, AHATCamParams, 0.001, new Point2D(centX, centY), markerRadius); ; // camPose.Transform(center3D.ToVec3());

                Vector3D wPt = camPt.TransformBy(extrinRot) + extrinTra;
                _markers.Add(wPt);

                Imgproc.circle(imgRGB, new Point(centX, centY), (int)rContour, new Scalar(255, 0, 0), 2);
            }

            PointCloud markerCloud = new PointCloud(_markers);
            OnMarkersProcessed.Invoke(markerCloud, imageData.ts);

            ////debugMediaTexture.LoadRawTextureData(img.);
            //debug2MediaTexture.Apply();
            texturesUpdated = true;
        }
    }

    //protected void ReadCamera((DepthImageCameraView, Shared<Image> i) camInfo, Envelope e)
    //{

    //    DepthImage depthImg = camInfo.Item1.ViewedObject.Resource;
    //    byte[] _depthBuf = depthImg.ReadBytes(depthImg.Size);
    //    double _depthFactor = depthImg.DepthValueToMetersScaleFactor;
    //    DepthImageCameraView d = camInfo.Item1;

    //    Matrix camPose;
    //    //Debug.WriteLine(camInfo.Item1.CameraPose);
    //    //if (camInfo.Item1.CameraPose != null)
    //    //{
    //    camPose = StereoKitTransforms.WorldHierarchy.Value.Transform(camInfo.Item1.CameraPose.ToStereoKitPose()).ToMatrix();
    //    //}
    //    //else
    //    //{
    //    //    camPose = Matrix.Identity;
    //    //}
    //    ICameraIntrinsics camInt = camInfo.Item1.CameraIntrinsics;
    //    DepthValueSemantics sem = depthImg.DepthValueSemantics;

    //    Image irImg = camInfo.i.Resource;

    //    byte[] _irBuf = irImg.ReadBytes(irImg.Size);
    //    int _imgHeight = irImg.Height;
    //    int _imgWidth = irImg.Width;
    //    using Mat img = ProcessImage(_imgHeight, _imgWidth, _irBuf);

    //    //using var src = new Mat(_imgHeight, _imgWidth, MatType.CV_16UC1, _irBuf);
    //    //using var src8 = new Mat(_imgHeight, _imgWidth, MatType.CV_8UC1);
    //    //src.ConvertTo(src8, MatType.CV_8UC1, 1.0 / 256);
    //    //Cv2.Normalize(src8, src8, 0, 255, NormTypes.MinMax);

    //    using var srcCol = new Mat(_imgHeight, _imgWidth, MatType.CV_8UC4);
    //    Cv2.CvtColor(img, srcCol, ColorConversionCodes.GRAY2BGRA);
    //    Mat[] p = Cv2.FindContoursAsMat(img, RetrievalModes.External, ContourApproximationModes.ApproxNone);
    //    List<Mat> circContours = new List<Mat>();
    //    List<Vector3D> detectedMarkers = new List<Vector3D>();
    //    _markers = new List<Vector3D>();
    //    for (int i = 0; i < p.Length; i++)
    //    {
    //        Moments m = Cv2.Moments(p[i]);
    //        if (m.M00 == 0)
    //            continue;
    //        double centX = m.M10 / m.M00;
    //        double centY = m.M01 / m.M00;
    //        double arcLength = Cv2.ArcLength(p[i], true);
    //        double area = Cv2.ContourArea(p[i]);
    //        double circularity = 4 * Math.PI * area / Math.Pow(arcLength, 2);
    //        double rContour = Math.Sqrt(area / Math.PI);
    //        if (area < 10)
    //        {
    //            continue;
    //        }
    //        Point3D center3D = GetCameraSpace3DPoint(_imgWidth, _imgHeight, _depthBuf, camInt, camPose, _depthFactor, sem, new Point2D(centX, centY), this.markerRadius);
    //        double distanceToCenter = Pixel3DDistance(_imgWidth, _imgHeight, _depthBuf, _depthFactor, new Point2D(centX, centY), this.markerRadius);
    //        double angle = Math.Acos(markerRadius / distanceToCenter);
    //        UnitVector3D rShift = (-center3D.ToVector3D()).Normalize();
    //        UnitVector3D rRotAxis = rShift.Orthogonal;
    //        rShift = rShift.Rotate(rRotAxis, MathNet.Spatial.Units.Angle.FromRadians(angle));
    //        Point2D? r = camInt.GetPixelPosition(center3D + rShift.ScaleBy(markerRadius), true);
    //        //Point2D? reprojCent = camInt.GetPixelPosition(center3D, true);

    //        Point3D sphereEdge = center3D + rShift.ScaleBy(markerRadius);
    //        double rX = centX;
    //        double rY = centY;
    //        if (r.HasValue)
    //        {
    //            rX = r.Value.X;
    //            rY = r.Value.Y;
    //        }

    //        double rReproj = Math.Sqrt((rX - centX) * (rX - centX) + (rY - centY) * (rY - centY));
    //        //double rReproj = Math.Sqrt((rX - reprojCent.Value.X) * (rX - reprojCent.Value.X) + (rY - reprojCent.Value.Y) * (rY - reprojCent.Value.Y));

    //        double estimatedArea = Math.PI * rContour * rContour;
    //        double areaRatio = area / estimatedArea;

    //        if (circularity < 0.75 || areaRatio < 0.9 || (rReproj - rContour) * (rReproj - rContour) > 9)
    //            continue;

    //        Vec3 pt = camPose.Transform(center3D.ToVec3());
    //        _markers.Add(new Vector3D(pt.x, pt.y, pt.z));

    //        Cv2.Circle(srcCol, (int)centX, (int)centY, (int)rContour, Scalar.Red, 2);
    //    }
    //    //for (int i = 0; i < _markers.Count; i++)
    //    //{
    //    //    testSpheres[i].transform.position = new Vec3((float)_markers[i].X, (float)_markers[i].Y, (float)_markers[i].Z);
    //    //}

    //    PointCloud pc = new PointCloud(Markers);
    //    MarkersProcessed?.Invoke(this, (pc, e));
    //    //Debug.WriteLine("Marker Centroid: " + pc.Centroid.ToString());
    //    //Text.Add(pc.Centroid.ToString(), new Pose(new Vec3((float) pc.Centroid.X, (float)pc.Centroid.Z, (float) pc.Centroid.Z), Quat.LookDir(Input.Head.Forward * -1)).ToMatrix(0.5f));
    //    tex.SetColors(_imgWidth, _imgHeight, srcCol.Data);
    //    if (displayImage)
    //        mat[MatParamName.DiffuseTex] = tex;
    //    //Debug.WriteLine(_markers.Count);

    //}

    private Mat ProcessImage(int height, int width, byte[] imgBuf, CameraIntrinsicsLoader camIntrinsics)
    {
        //Texture2D imgTex = new Texture2D(width, height, TextureFormat.R16, false);
        //imgTex.LoadRawTextureData(imgBuf);
        //imgTex.Apply();

        Mat imgMat = new Mat(height, width, CvType.CV_16UC1);
        MatUtils.copyToMat(imgBuf, imgMat);
        //Mat imgMatUndistorted = new Mat(height, width, CvType.CV_16UC1);
        //Calib3d.undistort(imgMat, imgMatUndistorted, camIntrinsics.CameraIntrinsicMat, camIntrinsics.DistortionCoefs);
        //using var src = new Mat(height, width, Imgproc, imgBuf);
        Imgproc.threshold(imgMat, imgMat, 1920, 65535, Imgproc.THRESH_BINARY);
        var srcThres = new Mat(height, width, CvType.CV_8UC1);
        imgMat.convertTo(srcThres, CvType.CV_8UC1);
        //Cv2.Threshold(srcThres, srcThres, 7.5, 255, ThresholdTypes.Binary);
        return srcThres;
    }

    private double GetDepthValueInterpolated(int width, int height, byte[] img, Point2D p)
    {
        double xEdit = p.X - 0.5;
        double yEdit = p.Y - 0.5;

        int p1X = (int)Math.Clamp(Math.Floor(xEdit), 0, width);
        int p2X = (int)Math.Clamp(Math.Ceiling(xEdit), 0, width);
        int p1Y = (int)Math.Clamp(Math.Floor(yEdit), 0, height);
        int p2Y = (int)Math.Clamp(Math.Ceiling(yEdit), 0, height);

        ushort fP11 = BitConverter.ToUInt16(img, 2 * (p1X + p1Y * width));
        ushort fP12 = BitConverter.ToUInt16(img, 2 * (p1X + p2Y * width));
        ushort fP21 = BitConverter.ToUInt16(img, 2 * (p2X + p1Y * width));
        ushort fP22 = BitConverter.ToUInt16(img, 2 * (p2X + p2Y * width));

        double fXY1 = (p2X - xEdit) * fP11 + (xEdit - p1X) * fP21;
        double fXY2 = (p2X - xEdit) * fP12 + (xEdit - p1X) * fP22;
        double rawDepth = (p2Y - yEdit) * fXY1 + (yEdit - p1Y) * fXY2;
        return rawDepth;
    }

    //private Vec3 GetStereoKit3DPoint(int width, int height, byte[] img, ICameraIntrinsics c, Matrix ext, double df, DepthValueSemantics s, Point2D p, double dOffset = 0)
    //{
    //    //double rawDepth = GetDepthValueInterpolated(width, height, img, p);
    //    ushort rawDepth = BitConverter.ToUInt16(img, 2 * ((int)p.X + (int)p.Y * width));
    //    double depth = rawDepth * df + dOffset;
    //    Point3D cPoint = c.GetCameraSpacePosition(p, depth, s, true);
    //    return ext.Transform(cPoint.ToVec3());
    //}


    private Vector3D GetCameraSpace3DPoint(int width, int height, byte[] depthImg, CameraIntrinsicsLoader c, double df, Point2D p, double dOffset = 0)
    {
        double interpRawDepth = GetDepthValueInterpolated(width, height, depthImg, p.X, p.Y);
        ushort rawDepth = BitConverter.ToUInt16(depthImg, 2 * ((int)p.X + (int)p.Y * width));
        double depth = interpRawDepth * df + dOffset;

        //Debug.WriteLine(interpRawDepth + ", " + rawDepth);

        Vector3D cPoint = c.Unproject(p, depth, true);

        return cPoint;
    }

    private Point3D ComputeCameraSpacePosition(Point2D point, double depth, CameraIntrinsicsLoader camIntrinics)
    {

        return Point3D.NaN;
    }


    private double Pixel3DDistance(int width, int height, byte[] img, double df, Point2D p, double dOffset = 0)
    {
        //double rawDepth = GetDepthValueInterpolated(width, height, img, p);
        ushort rawDepth = BitConverter.ToUInt16(img, 2 * ((int)p.X + (int)p.Y * width));
        double depth = rawDepth * df + dOffset;
        return depth;
    }

    private double GetDepthValueInterpolated(int width, int height, byte[] img, double x, double y)
    {
        int x_f = (int)x;
        int x_c = (x_f + 1 < width) ? x_f + 1 : x_f;

        int y_f = (int)y;
        int y_c = (y_f + 1 < height) ? y_f + 1 : y_f;

        ushort rawDepth1 = BitConverter.ToUInt16(img, 2 * ((int)x_f + (int)y_f * width));
        ushort rawDepth2 = BitConverter.ToUInt16(img, 2 * ((int)x_c + (int)y_f * width));
        ushort rawDepth3 = BitConverter.ToUInt16(img, 2 * ((int)x_f + (int)y_c * width));
        ushort rawDepth4 = BitConverter.ToUInt16(img, 2 * ((int)x_c + (int)y_c * width));

        double raw1 = (x_f != x_c) ? (x_c - x) * rawDepth1 + (x - x_f) * rawDepth2 : rawDepth1;
        double raw2 = (x_f != x_c) ? (x_c - x) * rawDepth3 + (x - x_f) * rawDepth4 : rawDepth3;

        return (y_f != y_c) ? (y_c - y) * raw1 + (y - y_f) * raw2 : raw1;
    }

    //public int RegisterToolID()
    //{
    //    return _markers.AddToolSet();
    //}
    //public List<Marker> GetToolPoints(int id)
    //{
    //    return _markers.GetToolPoints(id);
    //}

}
