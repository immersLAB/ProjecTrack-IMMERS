using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;
using TMPro;
using Microsoft.MixedReality.OpenXR;
using System.Net.Sockets;
using UnityEngine.Events;
using System.Security.Cryptography;
using MathNet.Spatial.Euclidean;
using QuatNet = MathNet.Spatial.Euclidean.Quaternion;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine.XR;

#if ENABLE_WINMD_SUPPORT
using Windows.Perception.Spatial;
using HL2UnityPlugin;
using Windows.Storage.Pickers;
#endif

public struct AHATImageData
{
    public byte[] depth;
    public byte[] image;
    public float[] extrinsics;
    public float[] devicePose;
    public QuatNet extrinRot;
    public Matrix<double> extrinT;
    public Vector3D extrinTra;

    //#if ENABLE_WINMD_SUPPORT

    //    //public Windows.Perception.PerceptionTimestamp ts;
    //#endif
    public long ts;
}

public class AHATCamera : MonoBehaviour
{
#if ENABLE_WINMD_SUPPORT
    HL2ResearchMode researchMode;
#endif

    public UnityEvent<AHATImageData> FrameReceived;
    enum DepthSensorMode
    {
        ShortThrow,
        LongThrow,
        None
    };
    [SerializeField] DepthSensorMode depthSensorMode = DepthSensorMode.ShortThrow;

    private Material depthMediaMaterial = null;
    private Texture2D depthMediaTexture = null;
    private byte[] depthFrameData = null;

    bool startRealtimePreview = true;
    private Material imageAHATMediaMaterial = null;
    private Texture2D imageAHATMediaTexture = null;
    private byte[] imageAHATFrameData = null;
    bool enablePointCloud = false;
    long lastTS;
    //private object imagelock = new object();
#if ENABLE_WINMD_SUPPORT
    SpatialCoordinateSystem unityWorldOrigin;
    SpatialLocatorAttachedFrameOfReference attachedRef;
    SpatialLocator devicePose = SpatialLocator.GetDefault();
#endif

    private void Awake()
    {
#if ENABLE_WINMD_SUPPORT
        unityWorldOrigin = PerceptionInterop.GetSceneCoordinateSystem(UnityEngine.Pose.identity) as SpatialCoordinateSystem;
        devicePose = SpatialLocator.GetDefault();
        //attachedRef = devicePose.CreateAttachedFrameOfReferenceAtCurrentHeading();
#endif
    }
    private void Update()
    {
        //Debug.Log("Unity Cam: " + Camera.main.transform.position.x + ", " + Camera.main.transform.position.y + ", " + Camera.main.transform.position.z);
    }

    public void StartDepthCamera()
    {
#if ENABLE_WINMD_SUPPORT
        unityWorldOrigin = PerceptionInterop.GetSceneCoordinateSystem(UnityEngine.Pose.identity) as SpatialCoordinateSystem;
        researchMode.SetReferenceCoordinateSystem(unityWorldOrigin);
        researchMode.SetPointCloudDepthOffset(0);

        // Depth sensor should be initialized in only one mode
        if (depthSensorMode == DepthSensorMode.LongThrow) researchMode.InitializeLongDepthSensor();
        else if (depthSensorMode == DepthSensorMode.ShortThrow) researchMode.InitializeDepthSensor();

        // Depth sensor should be initialized in only one mode
        if (depthSensorMode == DepthSensorMode.LongThrow) researchMode.StartLongDepthSensorLoop(enablePointCloud);
        else if (depthSensorMode == DepthSensorMode.ShortThrow) researchMode.StartDepthSensorLoop(enablePointCloud);
        

        researchMode.OutputIRDepth += ReceiveThingy;
#endif
    }

    public void StopDepthCamera()
    {
#if ENABLE_WINMD_SUPPORT
        researchMode.OutputIRDepth -= ReceiveThingy;
        researchMode.StopAllSensorDevice();
        while(!researchMode.DepthSensorStreamClosed()){continue;}
#endif
    }

    void Start()
    {
#if ENABLE_WINMD_SUPPORT
        researchMode = new HL2ResearchMode();
        StartDepthCamera();
#endif
    }



#if ENABLE_WINMD_SUPPORT
    void ReceiveThingy(object e, AHATCamEventArgs s)
    {

        AHATImageData imgDat = new AHATImageData();
        imgDat.ts = (long) s.GetTimeStamp();
        // update depth map texture
        if ((depthSensorMode == DepthSensorMode.ShortThrow && startRealtimePreview && researchMode.DepthMapTextureUpdated()) &&             
            depthSensorMode == DepthSensorMode.ShortThrow && startRealtimePreview && researchMode.ShortAbImageTextureUpdated())
        {
            ushort[] frameBufferD = s.GetDepthBuffer();
            ushort[] frameBufferIR = s.GetImageBuffer();
            float[] poseMat = s.GetExtrinsics();

            if (frameBufferD.Length > 0)
            {
                if (depthFrameData == null)
                {
                    depthFrameData = new byte[frameBufferD.Length * 2];

                }
                System.Buffer.BlockCopy(frameBufferD, 0, depthFrameData, 0, depthFrameData.Length);
                imgDat.depth = depthFrameData;
            }
            
            if (frameBufferIR.Length > 0)
            {
                if (imageAHATFrameData == null)
                {
                    imageAHATFrameData = new byte[frameBufferIR.Length*2];
                }

                System.Buffer.BlockCopy(frameBufferIR, 0, imageAHATFrameData, 0, imageAHATFrameData.Length);
                
                imgDat.image = imageAHATFrameData;

            }

            imgDat.extrinsics = poseMat;
            Matrix<double> extrinRot = Matrix<double>.Build.DenseIdentity(3, 3);
            Matrix<double> extrinT = Matrix<double>.Build.DenseIdentity(3, 3);

            //string poseString = "";
            //for (int r = 0; r < 4; r++)
            //{
            //    for (int c = 0; c < 4; c++)
            //    {
            //        poseString += poseMat[r * 4 + c] + ", ";
            //    }
            //}

            //Debug.Log(poseString);
            
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    extrinRot[c, r] = poseMat[r * 4 + c];
                    if (c == 2)
                        extrinT[c, r] = -poseMat[r * 4 + c];
                    else
                        extrinT[c, r] = poseMat[r * 4 + c];
                }
            }


            QuatNet extrinTra = new QuatNet(0, poseMat[12], poseMat[13], poseMat[14]);
            QuatNet extrinRotQ = LAExtensions.QuatFromRotation(extrinRot);

            ////QuatNet extrinRotInv = LAExtensions.QuatFromRotation(extrinRot).Inversed;
            ////QuatNet extrinTraInv = -(extrinRotInv * extrinTra * LAExtensions.QuatFromRotation(extrinRot));

            //SpatialLocation devicePoseAtCapture = devicePose.TryLocateAtTimestamp(Windows.Perception.PerceptionTimestampHelper.FromSystemRelativeTargetTime(TimeSpan.FromTicks(imgDat.ts)), unityWorldOrigin);

            //QuatNet deviceTra = new QuatNet(0, devicePoseAtCapture.Position.X, devicePoseAtCapture.Position.Y, devicePoseAtCapture.Position.Z);
            //QuatNet deviceRot = new QuatNet(devicePoseAtCapture.Orientation.W, devicePoseAtCapture.Orientation.X, devicePoseAtCapture.Orientation.Y, devicePoseAtCapture.Orientation.Z);

            //QuatNet fullExtrinRot = deviceRot * extrinRotQ;
            //QuatNet fullExtrinTra = deviceRot * extrinTra * deviceRot.Inversed + deviceTra;

            //imgDat.extrinRot = new QuatNet(fullExtrinRot.Real, -fullExtrinRot.ImagX, -fullExtrinRot.ImagY, fullExtrinRot.ImagZ);
            //imgDat.extrinTra = new Vector3D(fullExtrinTra.ImagX, fullExtrinTra.ImagY, -fullExtrinTra.ImagZ);

            imgDat.extrinRot = new QuatNet(extrinRotQ.Real, extrinRotQ.ImagX, extrinRotQ.ImagY, extrinRotQ.ImagZ);
            imgDat.extrinT = extrinT;//new QuatNet(extrinRotQ.Real, extrinRotQ.ImagX, extrinRotQ.ImagY, extrinRotQ.ImagZ);
            imgDat.extrinTra = new Vector3D(extrinTra.ImagX, extrinTra.ImagY, -extrinTra.ImagZ);

            //imgDat.extrinRot = new QuatNet(-devicePoseAtCapture.Orientation.W, devicePoseAtCapture.Orientation.X, devicePoseAtCapture.Orientation.Y, -devicePoseAtCapture.Orientation.Z);
            //imgDat.extrinTra = new Vector3D(devicePoseAtCapture.Position.X, devicePoseAtCapture.Position.Y, -devicePoseAtCapture.Position.Z);
            
            //imgDat.devicePose = new float[] {devicePoseAtCapture.Position.X, devicePoseAtCapture.Position.Y, -devicePoseAtCapture.Position.Z,
            //    -devicePoseAtCapture.Orientation.X, -devicePoseAtCapture.Orientation.Y, devicePoseAtCapture.Orientation.Z, devicePoseAtCapture.Orientation.W};
            //Debug.Log(extrinTra);
            //Debug.Log(deviceTra);

            //SpatialLocation devicePoseAtCapture = devicePose.TryLocateAtTimestamp(GetCurrentTimestamp(), unityWorldOrigin);

            //Debug.Log(poseMat.Length);

            //Debug.Log(devicePoseAtCapture);
            //Debug.Log("Device Timestamp 1: " + TimeSpan.FromTicks(imgDat.ts).Ticks);
            //Debug.Log("Device Timestamp 2: " + GetCurrentTimestamp().SystemRelativeTargetTime.Ticks);

            //if(devicePoseAtCapture != null)
            //Debug.Log("Device Pose: " + devicePoseAtCapture.Position.ToString());
            //else
            //{
            //Debug.Log("Device extrinsic: " + imgDat.extrinsics[12] + ", " + imgDat.extrinsics[13] + ", " + imgDat.extrinsics[14] );
            //}

        }

        FrameReceived.Invoke(imgDat);
}
#endif

    void LateUpdate()
    {
        //AHATImageData imgDat = new AHATImageData();
#if ENABLE_WINMD_SUPPORT
        //// update depth map texture
        //if ((depthSensorMode == DepthSensorMode.ShortThrow && startRealtimePreview && 
        //    depthPreviewPlane != null && researchMode.DepthMapTextureUpdated()) &&             
        //    depthSensorMode == DepthSensorMode.ShortThrow && startRealtimePreview && 
        //    imageAHATPreviewPlane != null && researchMode.ShortAbImageTextureUpdated())
        //{

        //    //byte[] frameTexture = researchMode.GetDepthMapTextureBuffer();
        //    //ushort[] frameBuffer = researchMode.GetDepthMapBuffer();

        //    //ushort[] frameBuffer = researchMode.GetDepthMapBuffer();
        //    ushort[] frameBufferD = new ushort[researchMode.GetDepthBufferSize()];
        //    ushort[] frameBufferIR = new ushort[researchMode.GetDepthBufferSize()];
        //    float[] poseMat = new float[16];
            
        //    Windows.Globalization.Calendar c = new Windows.Globalization.Calendar();

        //    Windows.Perception.PerceptionTimestamp ts;// = Windows.Perception.PerceptionTimestampHelper.FromHistoricalTargetTime(c.GetDateTime()); 

        //    researchMode.GetIRDepthMapBuffers(out frameBufferD, out frameBufferIR, out poseMat, out ts);
            
        //    c = new Windows.Globalization.Calendar();
        //    Windows.Perception.PerceptionTimestamp tst = Windows.Perception.PerceptionTimestampHelper.FromHistoricalTargetTime(c.GetDateTime()); 
        //    //Debug.Log(ts.SystemRelativeTargetTime.ToString());
        //    //Debug.Log(tst.SystemRelativeTargetTime.ToString());

        //    if (frameBufferD.Length > 0)
        //    {
        //        if (depthFrameData == null)
        //        {
        //            depthFrameData = new byte[frameBufferD.Length * 2];

        //        }
        //        System.Buffer.BlockCopy(frameBufferD, 0, depthFrameData, 0, depthFrameData.Length);
        //        imgDat.depth = depthFrameData;
        //    }
            
        //    if (frameBufferIR.Length > 0)
        //    {
        //        if (imageAHATFrameData == null)
        //        {
        //            imageAHATFrameData = new byte[frameBufferIR.Length*2];
        //        }

        //        System.Buffer.BlockCopy(frameBufferIR, 0, imageAHATFrameData, 0, imageAHATFrameData.Length);

        //        imgDat.image = imageAHATFrameData;

        //    }
        //    imgDat.extrinsics = poseMat;
        //    //Debug.Log(poseMat[12] + ", " + poseMat[13] + ", " + poseMat[14]);

        //    //imgDat.ts = GetCurrentTimeStamp();
        //    //Debug.Log(researchMode.PrintDepthExtrinsics());
        //}

        ////// update short-throw AbImage texture
        ////if (depthSensorMode == DepthSensorMode.ShortThrow && startRealtimePreview && 
        ////    imageAHATPreviewPlane != null && researchMode.ShortAbImageTextureUpdated())
        ////{
        ////    byte[] frameTexture = researchMode.GetShortAbImageTextureBuffer();
        ////    ushort[] frameBuffer = researchMode.GetShortAbImageBuffer();

        ////    if (frameBuffer.Length > 0)
        ////    {
        ////        if (imageAHATFrameData == null)
        ////        {
        ////            imageAHATFrameData = new byte[frameBuffer.Length*2];
        ////        }

        ////        System.Buffer.BlockCopy(frameBuffer, 0, imageAHATFrameData, 0, imageAHATFrameData.Length);

        ////        imgDat.image = imageAHATFrameData;

        ////    }
        ////}
#endif
        //FrameReceived.Invoke(imgDat);
    }

#if ENABLE_WINMD_SUPPORT
    public Windows.Perception.PerceptionTimestamp GetCurrentTimestamp()
    {
        // Get the current time, in order to create a PerceptionTimestamp. 
        Windows.Globalization.Calendar c = new Windows.Globalization.Calendar();
        return Windows.Perception.PerceptionTimestampHelper.FromHistoricalTargetTime(c.GetDateTime());
    }
#endif
    public void StopSensorsEvent()
    {
#if ENABLE_WINMD_SUPPORT
        researchMode.StopAllSensorDevice();
#endif
    }

    private void OnApplicationFocus(bool focus)
    {
        if (!focus) StopSensorsEvent();
    }

#if ENABLE_WINMD_SUPPORT
    long GetCurrentTimestampUnix()
    {
        // Get the current time, in order to create a PerceptionTimestamp. 
        Windows.Globalization.Calendar c = new Windows.Globalization.Calendar();
        Windows.Perception.PerceptionTimestamp ts = Windows.Perception.PerceptionTimestampHelper.FromHistoricalTargetTime(c.GetDateTime());
        return ts.TargetTime.ToUnixTimeMilliseconds();
        //return ts.SystemRelativeTargetTime.Ticks;
    }

#endif
}