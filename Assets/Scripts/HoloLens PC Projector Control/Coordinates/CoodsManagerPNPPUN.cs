using TMPro;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MRTKExtensions.QRCodes;
using System.Linq;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.Calib3dModule;
using OpenCVForUnity.UnityUtils;
using System.Collections.Generic;

public class CoodsManagerPNPPUN : MonoBehaviour
{

    public GameObject target1;
    public GameObject target2; // QR codes to detect 

    private Vector3[] points1; // first set of codes
    private Vector3[] points2; // second set of codes
    private Vector3 camPosition;
    private Quaternion camRotation;

    private GameObject camLoc;
    public GameObject camLocIni;

    public float width = 33.87f;
    
    public float xyratio = 1.787771f;
    public float size = 10f;
    public float ratio = 1.5933f;

    private float height;

    void Start()
    {
        points1 = new Vector3[4];
        points2 = new Vector3[4];
        camPosition = new Vector3();
        camRotation = Quaternion.identity;
        height = width / xyratio;


    }

    // set all the feedback text
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
        {
            CalibrateCam();
        }
        if (Input.GetKeyDown(KeyCode.P))
        {
            CalibrateCamPPNP();
        }
    }

    private void Record1() { DoRecord(ref points1, target1); }
    private void Record2() { DoRecord(ref points2, target2); }

    // record the QR code locations
    private void DoRecord(ref Vector3[] points, GameObject trackers)
    {

        if (trackers != null)
        {
            int nChild = 4; // trackers.transform.childCount;

            if (nChild == 4)
            {

                for (int i = 0; i < nChild; i++)
                {
                    Transform marker = trackers.transform.GetChild(i).GetChild(1).GetChild(0);

                    if (marker.gameObject.activeSelf) { points[i] = marker.TransformPoint(Vector3.zero); }
                    // else { Debug.LogError($"Code {i} not found!"); }

                }
            }
            // else { Debug.LogError("4 codes needed!"); }
        }
        else { Debug.LogError("Tracker unassigned!"); }
    }

    // calibrate the projector using a physical code + 4 codes
    public void CalibrateCamPPNP()
    {
        Debug.Log("5 codes PNP called!");
        Record1();
        Record2();

        // LogPoints();
        // Define the image points array
        Vector2[] imagePoints = new Vector2[]
        {
            new Vector2(-(width - size) / 2f, (height - size) / 2f),
            new Vector2((width - size) / 2f, (height - size) / 2f),
            new Vector2(-(width - size) / 2f, -(height - size) / 2f),
            new Vector2((width - size) / 2f, -(height - size) / 2f),
        };

        // Perform division by width * ratio
        float factor = width * ratio;
        for (int i = 0; i < imagePoints.Length; i++)
        {
            imagePoints[i] /= factor;
        }

        Vector3[] objectPoints = new Vector3[]
        {
            points1[0],
            points2[1],
            points2[2],
            points1[3]
        };

        // Convert object points to Mat (OpenCV format)
        MatOfPoint3f objectPointsMat = new MatOfPoint3f();
        objectPointsMat.fromArray(ConvertVector3ArrayToMat(objectPoints));

        // Convert image points to Mat (OpenCV format)
        MatOfPoint2f imagePointsMat = new MatOfPoint2f();
        imagePointsMat.fromArray(ConvertVector2ArrayToMat(imagePoints));

        // Camera intrinsic parameters (if known)
        Mat cameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
        MatOfDouble distCoeffs = new MatOfDouble(); // Empty matrix (no distortion assumed)

        // Output rotation and translation vectors
        Mat rvec = new Mat(3, 1, CvType.CV_64F);
        Mat tvec = new Mat(3, 1, CvType.CV_64F);

        double[] cameraPositionArray = new double[] { camLocIni.transform.position.x, camLocIni.transform.position.y, camLocIni.transform.position.z };
        // Convert position to a float array (assuming 3D position)
        Mat cameraPositionMat = new Mat(3, 1, CvType.CV_64F);

        cameraPositionMat.put(0, 0, cameraPositionArray);

        Quaternion invertedRot = Quaternion.Inverse(camLocIni.transform.rotation);
        Mat rmat = QuaternionToMat(invertedRot);
        Calib3d.Rodrigues(rmat, rvec);

        Core.gemm(rmat, cameraPositionMat, -1.0, new Mat(), 0, tvec);

        // Estimate pose using solvePnP
        Calib3d.solvePnP(objectPointsMat, imagePointsMat, cameraMatrix, distCoeffs, rvec, tvec, true, Calib3d.SOLVEPNP_ITERATIVE);

        // EPNP

        // Convert rotation vector (rvec) to rotation matrix (rmat)
        Calib3d.Rodrigues(rvec, rmat);

        // Transpose the matrix
        Mat rmatinv = new Mat();
        Core.transpose(rmat, rmatinv);

        Core.gemm(rmatinv, tvec, -1.0, new Mat(), 0, cameraPositionMat);

        cameraPositionMat.get(0, 0, cameraPositionArray);

        camPosition = new Vector3((float)cameraPositionArray[0], (float)cameraPositionArray[1], (float)cameraPositionArray[2]);
        camLoc.transform.position = camPosition;

        camRotation = MatToQuaternion(rmatinv);
        camLoc.transform.rotation = camRotation;

        camLoc.SetActive(true);

        // Release Mats
        objectPointsMat.release();
        imagePointsMat.release();
        cameraMatrix.release();
        cameraPositionMat.release();
        rvec.release();
        tvec.release();
        rmatinv.release();

        // Output pose
        Debug.Log("Estimated Translation: " + camPosition);
        Debug.Log("Estimated Rotation: " + camRotation);

    }

    // calibrate the projector using recorded locations
    public void CalibrateCam()
    {
        SyncManager syncM = FindObjectOfType<SyncManager>();
        if (syncM != null)
        {
            camLoc = syncM.GetCamObject();
            if (camLoc != null)
            {
                Debug.Log("Camera location object found!");
            }
        }
        else
        {
            Debug.Log("Camera location object missing!");
            return;
        }

        camLoc.SetActive(false);

        Debug.Log("8 codes PNP called!");
        Record1();
        Record2();

        // LogPoints();
        // Define the image points array
        Vector2[] imagePoints = new Vector2[]
        {
            new Vector2(-(width - size) / 2f, (height - size) / 2f),
            new Vector2((width - size) / 2f, (height - size) / 2f),
            new Vector2(-(width - size) / 2f, -(height - size) / 2f),
            new Vector2((width - size) / 2f, -(height - size) / 2f),
            new Vector2(-(width - size) / 2f, (height - size) / 2f),
            new Vector2((width - size) / 2f, (height - size) / 2f),
            new Vector2(-(width - size) / 2f, -(height - size) / 2f),
            new Vector2((width - size) / 2f, -(height - size) / 2f)
        };

        // Perform division by width * ratio
        float factor = width * ratio;
        for (int i = 0; i < imagePoints.Length; i++)
        {
            imagePoints[i] /= factor;
        }

        Vector3[] objectPoints = points1.Concat(points2).ToArray();

        // Convert object points to Mat (OpenCV format)
        MatOfPoint3f objectPointsMat = new MatOfPoint3f();
        objectPointsMat.fromArray(ConvertVector3ArrayToMat(objectPoints));

        // Convert image points to Mat (OpenCV format)
        MatOfPoint2f imagePointsMat = new MatOfPoint2f();
        imagePointsMat.fromArray(ConvertVector2ArrayToMat(imagePoints));

        // Camera intrinsic parameters (if known)
        Mat cameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
        MatOfDouble distCoeffs = new MatOfDouble(); // Empty matrix (no distortion assumed)

        // Output rotation and translation vectors
        Mat rvec = new Mat();
        Mat tvec = new Mat();

        // Estimate pose using solvePnP
        Calib3d.solvePnP(objectPointsMat, imagePointsMat, cameraMatrix, distCoeffs, rvec, tvec);

        // Convert rotation vector (rvec) to rotation matrix (rmat)
        Mat rmat = new Mat();
        Calib3d.Rodrigues(rvec, rmat);

        // Compute camera position
        Mat cameraPositionMat = new Mat(3, 1, CvType.CV_64F);
        // Core.multiply(rmat, -tvec, cameraPositionMat);

        // Transpose the matrix
        Mat rmatinv = new Mat();
        Core.transpose(rmat, rmatinv);

        Core.gemm(rmatinv, tvec, -1.0, new Mat(), 0, cameraPositionMat);

        // Log rmat, rvec, tvec
        // Debug.Log("rmat:");
        // Debug.Log(rmat.dump()); // Log the contents of rmat
        //Debug.Log("rvec:");
        //Debug.Log(rvec.dump()); // Log the contents of rvec
        //Debug.Log("tvec:");
        //Debug.Log(tvec.dump()); // Log the contents of tvec


        double[] cameraPositionArray = new double[3];
        cameraPositionMat.get(0, 0, cameraPositionArray);

        //// Log cameraPositionArray and cameraPositionMat
        //Debug.Log("cameraPositionMat:");
        //Debug.Log(cameraPositionMat.dump()); // Log the contents of cameraPositionMat
        //Debug.Log("cameraPositionArray: " + string.Join(", ", cameraPositionArray));


        camPosition = new Vector3((float)cameraPositionArray[0], (float)cameraPositionArray[1], (float)cameraPositionArray[2]);
        camLoc.transform.position = camPosition;

        camRotation = MatToQuaternion(rmatinv);
        camLoc.transform.rotation = camRotation;

        camLoc.SetActive(true);
        camLocIni.transform.parent.gameObject.SetActive(false);

        // Release Mats
        objectPointsMat.release();
        imagePointsMat.release();
        cameraMatrix.release();
        cameraPositionMat.release();
        rvec.release();
        tvec.release();
        rmatinv.release();

        // Output pose
        Debug.Log("Estimated Translation: " + camPosition);
        Debug.Log("Estimated Rotation: " + camRotation);

    }

    Quaternion MatToQuaternion(Mat rotationMatrix)
    {
        // Convert Mat to Matrix4x4
        Matrix4x4 matrix4x4 = new Matrix4x4();
        matrix4x4.SetRow(0, new Vector4((float)rotationMatrix.get(0, 0)[0], (float)rotationMatrix.get(0, 1)[0], (float)rotationMatrix.get(0, 2)[0], 0));
        matrix4x4.SetRow(1, new Vector4((float)rotationMatrix.get(1, 0)[0], (float)rotationMatrix.get(1, 1)[0], (float)rotationMatrix.get(1, 2)[0], 0));
        matrix4x4.SetRow(2, new Vector4((float)rotationMatrix.get(2, 0)[0], (float)rotationMatrix.get(2, 1)[0], (float)rotationMatrix.get(2, 2)[0], 0));
        matrix4x4.SetRow(3, new Vector4(0, 0, 0, 1));

        // Convert Matrix4x4 to Quaternion
        Quaternion quaternion = Quaternion.LookRotation(matrix4x4.GetColumn(2), matrix4x4.GetColumn(1));

        return quaternion;
    }

    // Function to convert Quaternion to Mat (rotation matrix)
    public Mat QuaternionToMat(Quaternion quaternion)
    {
        // Create a Matrix4x4 from the quaternion
        Matrix4x4 matrix4x4 = Matrix4x4.TRS(Vector3.zero, quaternion, Vector3.one);

        // Create a Mat object (3x3 matrix) for rotation
        Mat rotationMat = new Mat(3, 3, CvType.CV_64F); // Assuming CV_64FC1 for double precision

        // Extract rotation elements from Matrix4x4 to Mat
        rotationMat.put(0, 0, new double[] {
            matrix4x4[0, 0], matrix4x4[0, 1], matrix4x4[0, 2],
            matrix4x4[1, 0], matrix4x4[1, 1], matrix4x4[1, 2],
            matrix4x4[2, 0], matrix4x4[2, 1], matrix4x4[2, 2]
        });

        return rotationMat;
    }

    // Helper function to convert Vector3[] to MatOfPoint3f
    private Point3[] ConvertVector3ArrayToMat(Vector3[] array)
    {
        Point3[] result = new Point3[array.Length];
        for (int i = 0; i < array.Length; i++)
        {
            result[i] = new Point3(array[i].x, array[i].y, array[i].z);
        }
        return result;
    }

    // Helper function to convert Vector2[] to MatOfPoint2f
    private Point[] ConvertVector2ArrayToMat(Vector2[] array)
    {
        Point[] result = new Point[array.Length];
        for (int i = 0; i < array.Length; i++)
        {
            result[i] = new Point(array[i].x, array[i].y);
        }
        return result;
    }

    public void LogPoints()
    {
        for (int i = 0; i < points1.Length; i++) { Debug.Log(points1[i]); }
        for (int i = 0; i < points2.Length; i++) { Debug.Log(points2[i]); }

        Debug.Log(camPosition);
    }

    private Vector3 AvgPoints(Vector3[] points)
    {

        Vector3 averagePoint = Vector3.zero;
        if (points.Length > 0)
        {
            foreach (Vector3 point in points)
            {
                averagePoint += point;
            }
            averagePoint /= points.Length;
        }

        return averagePoint;

    }

    // reset all the detected QR codes
    public void ResetAll()
    {
        DoReset(target1);
        DoReset(target2);
        if (camLoc != null) { camLoc.SetActive(false); }

        for (int i = 0; i < points1.Length; i++) { points1[i] = Vector3.zero; }
        for (int i = 0; i < points2.Length; i++) { points2[i] = Vector3.zero; }
        camPosition = Vector3.zero;
        camRotation = Quaternion.identity;

    }

    // whether to show the sample object to visualize where the projector is
    public void HideShowCam()
    {
        if (camLoc == null) { return; }
        if (camLoc.activeSelf)
        {
            camLoc.SetActive(false);
        }
        else
        {
            camLoc.SetActive(true);
        }

        if (target1.activeSelf)
        {
            target1.SetActive(false);
        }
        else
        {
            target1.SetActive(true);
        }

        if (target2.activeSelf)
        {
            target2.SetActive(false);
        }
        else
        {
            target2.SetActive(true);
        }
    }

    // reset one set of QR codes

    private void DoReset(GameObject trackers)
    {

        if (trackers != null)
        {
            int nChild = trackers.transform.childCount;

            for (int i = 0; i < nChild; i++)
            {
                Transform displayer = trackers.transform.GetChild(i).GetChild(1);
                QRPoseTrackController controller = displayer.GetComponent<QRPoseTrackController>();
                controller.Reset();
            }
        }
    }



}
