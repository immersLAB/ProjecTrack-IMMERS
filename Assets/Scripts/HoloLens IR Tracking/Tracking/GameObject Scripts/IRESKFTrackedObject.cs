using MathNet.Numerics.LinearAlgebra;
using MathNet.Spatial.Euclidean;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;
using StereoSensors.Tracking;
using System.Threading.Tasks;
#if WINDOWS_UWP 
using Windows.Devices.Bluetooth.GenericAttributeProfile;
using Windows.Storage.Streams;

#endif

public class IRESKFTrackedObject : MonoBehaviour
{

    private PointCloud _markerCloud;
    private int _toolID;
    //Sphere c1, c2, c3, c4;
    private RegistrationData _regData;
    private UnityEngine.Quaternion _filteredOrientation;
    private bool _poseUpdated;
    private ESP32Client btC = new ESP32Client();

    System.Diagnostics.Stopwatch watch = new System.Diagnostics.Stopwatch();

    QuatESKF filter;
    long lastDetectedTime;

    void Start()
    {
        _markerCloud = GetChildObjectsAsPointCloud();
    }
    private async void Awake()
    {

#if WINDOWS_UWP
        Debug.Log("Attempting to connect");
        Task<GattCharacteristic> ch = btC.Connect("e8:9f:6d:30:e4:c2", "6e400001-b5a3-f393-e0a9-e50e24dcca9e");
        await ch;
        ch.Result.ValueChanged += Notify;
#endif
    }
    private void Update()
    {
        if (_poseUpdated)
        {
            MathNet.Spatial.Euclidean.Quaternion q = LAExtensions.QuatFromRotation(_regData.R);
            UnityEngine.Quaternion qUnity = new UnityEngine.Quaternion((float)q.ImagX, (float)q.ImagY, (float)q.ImagZ, (float)q.Real);
            this.transform.rotation = qUnity * this.transform.rotation;
            this.transform.position = qUnity * this.transform.position + (new Vector3((float)_regData.t.X, (float)_regData.t.Y, (float)_regData.t.Z));
            //_markerCloud = GetChildObjectsAsPointCloud();
            _poseUpdated = false;
        }

    }
    public void UpdatePose(PointCloud d, long ts)
    {
        PointCloud detectedCloud = d;
        if (detectedCloud.Count < 2)
        {
            return;
        }

        RegistrationData result = ICPSolver.Register(_markerCloud, detectedCloud);
        if (result.errorSq > 0.0008)
        {
            return;
        }

        K3DTree tree = new K3DTree(detectedCloud.GetPointsAsList());
        //K3DTree tree = new K3DTree(d.Item1.ZeroCenteredPoints());
        //Vector<double> obs = Vector<double>.Build.Dense((currentCloud.Count) * 3 + 3);
        List<Vector3D> currentPoints = _markerCloud.GetPointsAsList();
        Vector<double> obs = Vector<double>.Build.Dense((_markerCloud.Count) * 3);

        for (int i = 0; i < currentPoints.Count; i++)
        {
            Vector3D closestPoint = tree.FindClosest(currentPoints[i]);
            obs[i * 3] = closestPoint.X;
            obs[i * 3 + 1] = closestPoint.Y;
            obs[i * 3 + 2] = closestPoint.Z;
        }

        double dt = (ts - lastDetectedTime) / ((double)System.TimeSpan.TicksPerSecond);

        filter.UpdatePrior(dt);
        filter.UpdatePosterior(obs, dt);
        filter.InjectError();
        lastDetectedTime = ts;

        //_regData = result;
        //_poseUpdated = true;
    }

#if WINDOWS_UWP 
    protected virtual void Notify(GattCharacteristic c, GattValueChangedEventArgs a)
    {
        watch.Restart();
        var reader = DataReader.FromBuffer(a.CharacteristicValue);
        reader.ByteOrder = ByteOrder.LittleEndian;
        double dt = reader.ReadDouble();

        //Debug.WriteLine("Arduino " + dt);
        //Debug.WriteLine("Program " + elapsedTime);

        double ax = reader.ReadDouble();
        double ay = reader.ReadDouble();
        double az = reader.ReadDouble();

        double gx = reader.ReadDouble();
        double gy = reader.ReadDouble();
        double gz = reader.ReadDouble();

        double mx = reader.ReadDouble();
        double my = reader.ReadDouble();
        double mz = reader.ReadDouble();

        //// Adafruit Config
        Vector<double> accel = Vector<double>.Build.DenseOfArray(new double[] { ax, -az, ay });
        Vector<double> gyro = Vector<double>.Build.DenseOfArray(new double[] { -gx, gz, -gy });
        Vector<double> mag = Vector<double>.Build.DenseOfArray(new double[] { mx, my, mz });

        Debug.Log(gyro);

        // Sparkfun Config
        //Vector<double> accel = Vector<double>.Build.DenseOfArray(new double[] { ax, -az, ay });
        //Vector<double> gyro = Vector<double>.Build.DenseOfArray(new double[] { gx, -gz, gy });
        //Vector<double> accel = Vector<double>.Build.DenseOfArray(new double[] { 0, 0, 0.001 });
        //Vector<double> gyro = Vector<double>.Build.DenseOfArray(new double[] { 0, 0, 0 });

        //Vector<double> stateUpdate = Vector<double>.Build.DenseOfArray(new double[] { dt, ax, -az, ay, gx, -gz, gy });
        Vector<double> stateUpdate = Vector<double>.Build.DenseOfArray(new double[] { dt, gy, gx, -gz });
        //stateUpdate2 = Vector<double>.Build.DenseOfArray(new double[] { dt, ax, -az, ay });

        filter.UpdateNominal(stateUpdate.SubVector(1, stateUpdate.Count - 1), stateUpdate[0]);

        double elapsedTime = watch.ElapsedTicks / ((double)System.TimeSpan.TicksPerSecond);
    }
#endif

    protected PointCloud GetChildObjectsAsPointCloud()
    {
        List<Vector3D> c = new List<Vector3D>();
        for (int i = 0; i < this.transform.childCount; i++)
        {
            Transform t = this.transform.GetChild(i);
            c.Add(new Vector3D(t.transform.position.x, t.transform.position.y, t.transform.position.z));
        }

        return new PointCloud(c);
    }

}
