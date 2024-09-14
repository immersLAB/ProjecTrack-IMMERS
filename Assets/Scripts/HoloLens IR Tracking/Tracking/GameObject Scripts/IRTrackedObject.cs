using MathNet.Numerics.LinearAlgebra;
using MathNet.Spatial.Euclidean;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit.Filtering;
using MathNet.Numerics.LinearAlgebra.Factorization;


public class IRTrackedObject : MonoBehaviour
{

    public enum PoseDetectionType {GEOMETRIC, ICP, GO_ICP, VOTING};

    [SerializeField]
    PoseDetectionType poseMatchingMode = PoseDetectionType.GEOMETRIC;

    private PointCloud _markerCloud = new PointCloud();
    private int _toolID;
    // TODO: ask wally what these are for -> delete: Sphere c1, c2, c3, c4;
    private RegistrationData _regData;
    private bool _poseUpdated;
    private Queue<MathNet.Spatial.Euclidean.Quaternion> pastOrientations = new Queue<MathNet.Spatial.Euclidean.Quaternion>();
    private Queue<Vector3> pastPositions = new Queue<Vector3>();
    private bool _trackingInitialized = false;
    [SerializeField]
    int numAverage = 4;

    [SerializeField]
    double threshold = 0.0075;

    [SerializeField]
    private bool _trackingEnabled = true;

    // Initializing variables for the tool tip's pose transform (aka position and rotation):
    [SerializeField]
    private Transform toolTipTransform; // Initializing private variable to represent ToolTip empty GameObject's transform (position and rotation)

    public Vector3 ToolTipPosition { get; private set; } // Initializing 3D vector for position of ToolTip origin
    public UnityEngine.Quaternion ToolTipRotation { get; private set; } // Initializing angle for rotation of ToolTip origin 
        // Using 'UnityEngine.Quaternion' rather than 'MathNet.Spatial.Euclidean.Quaternion' because I'm applying calculations to a Unity GameObject, so this is more efficient


    void Start()
    {
        _markerCloud = GetChildObjectsAsPointCloud();

        // Initializing ToolTip origin transform, based off the ToolTip empty GameObject:
        if (toolTipTransform == null)
        {
            Debug.LogError("ToolTip transform is not assigned!"); // Debugging log if ToolTip GameObject isn't assigned
        }
    }
    private void Update()
    {
        if (_poseUpdated)
        {
            MathNet.Spatial.Euclidean.Quaternion q = LAExtensions.QuatFromRotation(_regData.R);
            UnityEngine.Quaternion qUnity = new UnityEngine.Quaternion((float)q.ImagX, (float)q.ImagY, (float)q.ImagZ, (float)q.Real);

            this.transform.rotation = qUnity * this.transform.rotation;
            this.transform.position = qUnity * this.transform.position + (new Vector3((float)_regData.t.X, (float)_regData.t.Y, (float)_regData.t.Z));

            pastOrientations.Enqueue(new MathNet.Spatial.Euclidean.Quaternion(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z));
            pastPositions.Enqueue(transform.position);

            if (pastOrientations.Count > numAverage)
            {
                pastOrientations.Dequeue();
                pastPositions.Dequeue();
                MathNet.Spatial.Euclidean.Quaternion qAvg = AverageQuaternion(pastOrientations);
                this.transform.rotation = new UnityEngine.Quaternion((float)qAvg.ImagX, (float)qAvg.ImagY, (float)qAvg.ImagZ, (float)qAvg.Real);
                this.transform.position = AveragePosition(pastPositions);
            }
            _markerCloud = GetChildObjectsAsPointCloud();
            _poseUpdated = false;

            // Updating the ToolTip transform (position and rotation)
            if (toolTipTransform != null) // Checking that the toolTipTransform exists
            {
                ToolTipPosition = toolTipTransform.position; // Assign new position of ToolTip, from the transform's position
                ToolTipRotation = toolTipTransform.rotation; // Assign new rotation of ToolTip, form the trans
            }
        }
    }

    public void UpdatePose(PointCloud d, long ts)
        // ts refers to timestamp
    {
        if (!_trackingEnabled) // If tracking is disabled, return (exit the UpdatePose function)
        {
            return;
        }

        PointCloud detectedCloud = d; // Initialize the point cloud, detected by the HoloLens 2 from the tool's IR markers
        if (detectedCloud.Count < _markerCloud.Count-1) // If total amount of IR markers is ?? TODO: what exactly does this line verify?
        {
            return;
        }

        RegistrationData result;

        // removed unecessary, old code comments from debugging

        switch (poseMatchingMode)
        {
            case PoseDetectionType.ICP:
                result = ICPSolver.Register(_markerCloud, detectedCloud);
                break;
            case PoseDetectionType.GO_ICP:
                result = GoICPSolver.Register(_markerCloud, detectedCloud, 100);
                break;

            case PoseDetectionType.VOTING:
                result = GeometryVotingSolver.Register(_markerCloud, detectedCloud, threshold);
                break;
            case PoseDetectionType.GEOMETRIC:
            default:
                result = GeometrySolver.Register(_markerCloud, detectedCloud, threshold);
                break;
        }

        if (result.error > 0.003 * _markerCloud.Count)
            {
                return;
            }

        _regData = result;
        _poseUpdated = true;
    }
    protected PointCloud GetChildObjectsAsPointCloud()
    {
        List<Vector3D> c = new List<Vector3D>();
        for (int i = 0; i < this.transform.childCount; i++)
        {
            if (this.transform.GetChild(i).tag != "marker")
                continue;
            Transform t = this.transform.GetChild(i);
            c.Add(new Vector3D(t.transform.position.x, t.transform.position.y, t.transform.position.z));
        }

        return new PointCloud(c);
    }


    MathNet.Spatial.Euclidean.Quaternion AverageQuaternion(Queue<MathNet.Spatial.Euclidean.Quaternion> qList) 
    {
        Matrix<double> sumQuatOuterProd = Matrix<double>.Build.Dense(4, 4);
        for (int i = 0; i < qList.Count; i++)
        {
            Vector<double> qVec = Vector<double>.Build.DenseOfArray(new double[] { qList.ElementAt(i).Real, qList.ElementAt(i).ImagX, qList.ElementAt(i).ImagY, qList.ElementAt(i).ImagZ });
            sumQuatOuterProd += qVec.OuterProduct(qVec);
        }
        sumQuatOuterProd /= qList.Count;
        Svd<double> decomposition = sumQuatOuterProd.Svd();
        Vector<double> qAsVec = decomposition.U.Column(0);
        MathNet.Spatial.Euclidean.Quaternion q = new MathNet.Spatial.Euclidean.Quaternion(qAsVec[0], qAsVec[1], qAsVec[2], qAsVec[3]);
        //Debug.WriteLine(q);
        return q;
 
        }
    Vector3 AveragePosition(Queue<Vector3> qList)
    {
        Vector3 avg = new Vector3();
        for (int i = 0; i < qList.Count; i++)
        {
            avg += qList.ElementAt(i);
        }
        return avg / qList.Count;
    }

    public void EnableTracking()
    {
        _trackingEnabled = true;
    }
}

// Overall, this script calculates the tool (OptiTrack Probe) pose (position and rotation) in global coordinate space (i.e. entire Unity scene)
    // After calculating and updating the tool pose, it track the tip (ToolTip) pose - where we want to later draw virtual content

    // Script stores the toolTipTransform (pose) along with toolTipPosition (position) and toolTipRotation (rotation):
        // toolTipTransform - position + rotation (overall tip pose)
        // ToolTipPosition - position (3D vector)
        // ToolTipRotation - rotation (quaternion)

    // Pose, position, and rotation are updated (overwritten) for each new calculated tool pose, so:
        // TODO - test 1 (sphere): Send over toolTip info to PUN for one calculated pose (triggered by voice command)
            // Yuxuan's project:
                // SyncManager.cs
                    // Start() - start PUN; objects being shared between hololens and PC
                    // Connect() - joins the room (AshyRoom), which is created by the hololens
                    // OnCreatedRoom() - join the NEW room created (if the intiial one failed)
                    // OnJoinedRoom() - automatically called when player (PC) joins room
                    // InstantiateObject() - instantiates all the shared objects
                        // Shared objects must be in the Resources folder
                        //  Each object needs: PShoton Transform View (synchronize position between different players) and Photon View (PUN to know)
        // TODO - test 2 (line drawing): Send over toolTip info to PUN for multiple calculated poses overtime (triggered to start / stop w/ 2 voice comannds)


// TODO: continuing editing my code on 08/08, implementing drawing on a 2D plane in the PC scene, while using the tracked probe
    // 08/08: Integrate w/ another script to send PUN to PC -> then use location to place a sphere
    // 08/08: Have a voice command to 'place sphere'

    // 08/09: Have a voice command to 'start drawing' and 'stop drawing' -> use linerenderer -> extrude -> shader for 3D model instead of placing spheres