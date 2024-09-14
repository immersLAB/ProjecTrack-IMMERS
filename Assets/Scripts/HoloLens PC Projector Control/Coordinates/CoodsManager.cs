using TMPro;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MRTKExtensions.QRCodes;


public class CoodsManager : MonoBehaviour
{

    public GameObject target1;
    public GameObject target2; // QR codes to detect 

    private Vector3[] points1; // first set of codes
    private Vector3[] points2; // second set of codes
    private Vector3 camPosition;
    private Quaternion camRotation;
    private float radCircle;

    public GameObject camLoc;
    public float rCircle;
    public float halfDiag;

    void Start()
    {
        points1 = new Vector3[4];
        points2 = new Vector3[4];
        camPosition = new Vector3();
        camRotation = Quaternion.identity;
        camLoc.SetActive(false);
        radCircle = rCircle / halfDiag;

    }

    // set all the feedback text
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
        {
            CalibrateCam();
        }
    }

    private void Record1() { DoRecord(ref points1, target1); }
    private void Record2() { DoRecord(ref points2, target2); }

    // record the QR code locations
    private void DoRecord(ref Vector3[] points, GameObject trackers)
    {

        if (trackers != null)
        {
            int nChild = trackers.transform.childCount;

            if (nChild == 4) { 

                for (int i = 0;i<nChild; i++) {
                    Transform marker = trackers.transform.GetChild(i).GetChild(0).GetChild(0);

                    if (marker.gameObject.activeSelf) { points[i] = marker.TransformPoint(Vector3.zero); }
                    else { Debug.LogError($"Code {i} not found!"); }
                    
                }
            }
            else { Debug.LogError("4 codes needed!"); }
        }
        else { Debug.LogError("Tracker unassigned!"); }
    }
    
    // calibrate the projector using recorded locations
    public void CalibrateCam()
    {
        Record1();
        Record2();


        // Position

        Matrix<double> matA = Matrix<double>.Build.Dense(12, 7);
        Matrix<double> vecB = Matrix<double>.Build.Dense(12,1);

        Matrix<double> eye3 = Matrix<double>.Build.DenseIdentity(3);

        for (int i = 0; i < 4; i++)
        {
            Matrix<double> vecX1 = Vec2Mat(points1[i]);
            Matrix<double> vecX2 = Vec2Mat(points2[i]);
            matA.SetSubMatrix(3 * i, 0, eye3);
            matA.SetSubMatrix(3 * i, 3 + i, vecX1 - vecX2);
            vecB.SetSubMatrix(3 * i, 0, vecX1);

        }

        Matrix<double> result = matA.PseudoInverse() * vecB;

        camPosition.x = (float)result[0, 0];
        camPosition.y = (float)result[1, 0];
        camPosition.z = (float)result[2, 0];

        camLoc.SetActive(true);
        camLoc.transform.position = camPosition;

        LogPoints();

        // Rotation

        Vector3 camLookAt = Vector3.zero;
        Vector3 camUpper = Vector3.zero;

        for (int i = 0; i < 2; i++)
        {
            Vector3 direction = ((points2[i] + points1[i]) / 2-camPosition).normalized;
            camLookAt += direction;
            camUpper += direction;
        }
        for (int i = 2; i < 4; i++)
        {
            Vector3 direction = ((points2[i] + points1[i]) / 2 - camPosition).normalized;
            camLookAt += direction;
            camUpper -= direction;
        }

        camRotation = Quaternion.LookRotation(camLookAt, camUpper);
        camLoc.transform.rotation = camRotation;

        // Scale

        float angDiag = 0.0f;
        float tanDiag;
        for (int i = 0; i < 4; i++)
        {
            angDiag += Vector3.Angle((points2[i] + points1[i]) / 2 - camPosition, camLookAt) / 4.0f;
        }

        tanDiag = Mathf.Tan(angDiag * Mathf.Deg2Rad);

        Transform coneTransform = camLoc.transform.Find("Cone"); //
        if (coneTransform != null)
        {
            Vector3 currentScale = coneTransform.localScale;
            currentScale.x = 2 * currentScale.z * tanDiag * radCircle;
            currentScale.y = 2 * currentScale.z * tanDiag * radCircle;
            coneTransform.localScale = currentScale;
        }


    }

    // helper functions
    private Matrix<double> Vec2Mat(Vector3 point)
    {
        double[,] Vec = new double[3,1] { { point.x }, {point.y }, { point.z } };
        return Matrix<double>.Build.DenseOfArray(Vec);
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
