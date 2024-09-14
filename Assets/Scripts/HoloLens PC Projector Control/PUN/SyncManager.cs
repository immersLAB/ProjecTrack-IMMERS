using UnityEngine;
using Photon.Pun;
using Photon.Realtime;
using TMPro;
using System.Collections.Generic;
using UnityEditor.XR;
using System.Dynamic;
// TODO: confirm that it's okay to not use System.Numerics

public class SyncManager : MonoBehaviourPunCallbacks
{
    public TextMeshPro displayText; // feedback text
    public string roomName; // room to join
    public List<GameObject> prefabShare; // prefabs to instantiate across network
    private List<GameObject> objectShare; // objects instantiated across network
    private List<bool> first;
    public string layerName;
    public Camera targetCamera;

    // Initializing variables (intially from SpeechListener.cs)
    public GameObject placedSphere; // Sphere prefab
    private IRTrackedObject _trackedObject; // IR tracked object reference
    private bool isDrawing = false; // Boolean to designate start vs. stop drawing

    // Initializing variables (for 3D line drawing functionality)
    public GameObject drawingLine; // LineRenderer prefab (aka MarkerLine currently)
    private LineRenderer currentLineRenderer; // LineRenderer effect for current 3D line drawing
    private List<Vector3> linePoints = new List<Vector3>(); // List of 3D vectors, to store the 3D points along the 3D line (lineRenderer)


    void Start()
    {
        PhotonNetwork.ConnectUsingSettings();
        objectShare = new List<GameObject>();
        first = new List<bool>();
        for (int i = 0; i < prefabShare.Count; i++)
        {
            first.Add(true);
        }

        _trackedObject = FindObjectOfType<IRTrackedObject>(); // intially from SpeechListener.cs
    }

    // Join the room
    public void Connect()
    {
        Debug.Log("Connecting");
        PhotonNetwork.JoinRoom(roomName);
    }

    // If no room exists, notify user and create a room
    public override void OnJoinRoomFailed(short returnCode, string message)
    {
        Debug.Log("Tried to join the room and failed.");
        displayText.text = "Failed";
        PhotonNetwork.CreateRoom(roomName, new RoomOptions { MaxPlayers = 2 });
    }

    // Automatically join room when new room is created
    public override void OnCreatedRoom()
    {
        Debug.Log("Room created successfully: " + roomName);
        displayText.text = "Created " + roomName;

        // After creating the room, optionally join it immediately
        PhotonNetwork.JoinRoom(roomName);
    }

    // Instantiate shared Photon Unity Network
    public override void OnJoinedRoom()
    {
        Debug.Log("Joined the room!");
        displayText.text = "Joined  " + roomName;
        InstantiateObject();
    }


    // instantiate all shared objects

    [PunRPC]
    void InstantiateObject()
    {
        Vector3 spawnPosition = new Vector3(0, 0, 0.55f);
        // changed y from 1.55f -> 0 (so that it spawns in front of the user / MRTK Rig, which is now @ Y = 0)
        Quaternion spawnRotation = Quaternion.identity;

        // Instantiate prefabs across the network
        for (int i = 0; i < prefabShare.Count; i++)
        {
            GameObject pfb = prefabShare[i];

            if ((pfb != null) && (first[i] == true))
            {
                // Instantiate prefab using PhotonNetwork
                GameObject obj = PhotonNetwork.Instantiate(pfb.name, spawnPosition, spawnRotation);

                // Add instantiated object to objectShare list
                objectShare.Add(obj);
                first[i] = false;
            }
            else
            {
                Debug.LogWarning("Prefab at index " + i + " is null.");
            }
        }
    }


    public GameObject GetCamObject()
    {
        return objectShare[0];
    }

    private LayerMask GetLayerMask(string layerName)
    {
        int layer = LayerMask.NameToLayer(layerName);
        return 1 << layer;
    }

    private bool IsLayerVisible(Camera cm, LayerMask lm)
    {
        // Check if the layer is included in the camera's culling mask
        return (cm.cullingMask & lm) != 0;
    }

    public void HideShowHand()
    {
        if (targetCamera == null) { return; }
        if (layerName == null) { return; }

        LayerMask lm = GetLayerMask(layerName);

        bool isVisible = IsLayerVisible(targetCamera, lm);

        if (isVisible)
        {
            targetCamera.cullingMask &= ~lm;
        }
        else
        {
            targetCamera.cullingMask |= lm;
        }

    }

    // Function for a placing a single sphere object at once (On Click):
        // Previously triggered with 'place object' -> now triggered with 'mark'
    // TODO:
    // 1. Edit statefulInteractable -> SyncManager.onPlacedObject (aka using SyncManager.cs instead of SpeechListener.cs) - DONE
    // 2. test if this works w/ "drawingSphere" - DONE
    // 3. test if this works with placedSphere.name - TODO
    // 4. Experiment w/ adding the startDrawing and stopDrawing - DOING
    [PunRPC]
    public void onPlacedObject()
    {
        if (_trackedObject != null)
        {
            Vector3 spherePosition = _trackedObject.ToolTipPosition;
            Quaternion sphereRotation = _trackedObject.ToolTipRotation;

            // original version:
            PhotonNetwork.Instantiate("drawingSphere", spherePosition, sphereRotation); // Place a new instantiation of the sphere prefab
        }
    }

    // Function for 'draw' -> to run Update()'s body code and continuously place sphere prefabs
        // Previously triggered with 'start drawing' -> now triggered with 'draw'
    [PunRPC]
    public void StartDrawing()
    {
        isDrawing = true;

        // new code for 3D line drawing:

        linePoints.Clear(); // Clear all previous 3D points

        // Instantiate MarkerLine (LineRenderer prefab), which will be used to draw in 3D
        GameObject overallLine = PhotonNetwork.Instantiate("MarkerLine", _trackedObject.ToolTipPosition, Quaternion.identity);
        currentLineRenderer = overallLine.GetComponent<LineRenderer>();
        // TODO: check code -> then, can I just use markerLine prefab instead of overallLine?
        // TODO: verify that using linerenderer prefab works
        // TODO: use drawingLine.name instead of "MarkerLine"

        if (currentLineRenderer != null) // Ensures that only one lineRenderer exists, so we don't reinitialize every time the user StartDrawing() runs

        {
            currentLineRenderer.positionCount = 1; // Initialize current LineRenderer with no 3D points
            currentLineRenderer.SetPosition(0, _trackedObject.ToolTipPosition); // Initial position set to the tool tip position when StartDrawing() runs
            linePoints.Add(_trackedObject.ToolTipPosition); // Add initial position to linePoints (list of 3D points for line)
        }
    }

    // Function for 'stop' -> to NOT run Update()'s body code and stop placing sphere prefabs
    // Previously triggered with 'stop drawing' -> now triggered with 'stop'
    [PunRPC]
    public void StopDrawing()
    {
        isDrawing = false;

        // new code for 3D line drawing:

        currentLineRenderer = null; // Reset LineRenderer reference
        linePoints.Clear(); // Clear list of 3D points
    }

    // Function for 'clear' -> to destory all instances of the drawingSphere prefab (aka anything with the DrawingObject tag) 
        // Previously triggered with 'clear drawing' -> now triggered with 'clear'
    [PunRPC]
    public void ClearDrawing()
    {
        // Find all instances of the drawingSphere in the scene
        GameObject[] spheres = GameObject.FindGameObjectsWithTag("DrawingObject");

        foreach (GameObject sphere in spheres)
        {
            PhotonNetwork.Destroy(sphere); // Destroy each sphere over the network
        }

        // TODO: add functionality to remove 3D line drawing
    }

    // Update is called once per frame
        // Don't need to call [PunRPC] because Update() is automatically running for *each* user (HoloLens 2, PC/projector) every frame
        // TODO: move onto testing w/ placedSphere.name instead of drawingSphere
    void Update()
    {
        // original version:
        // Continuously place sphere prefabs w/ ToolTip's current transform (position and rotation):
        //if (isDrawing && _trackedObject != null) // If isDrawing is true (triggered in inspector settings via 'start drawing') and tracked object exists:
        //{
            // original version:
            //Vector3 spherePosition = _trackedObject.ToolTipPosition;
            //Quaternion sphereRotation = _trackedObject.ToolTipRotation;

            //PhotonNetwork.Instantiate("drawingSphere", spherePosition, sphereRotation); // Place a new instantiation of the sphere prefab
        //}

        // new version for 3D line drawing:
 
        if (isDrawing && _trackedObject != null && currentLineRenderer != null) // If isDrawing is true (triggered in inspector settings via 'start drawing') and tracked object exists:
        {
            Vector3 linePointPosition = _trackedObject.ToolTipPosition;

            if (linePoints.Count == 0 || Vector3.Distance(linePoints[linePoints.Count - 1], linePointPosition) > 0.005f) 
            {
                linePoints.Add(linePointPosition);
                currentLineRenderer.positionCount = linePoints.Count;
                currentLineRenderer.SetPositions(linePoints.ToArray());

                // Move currentLineRenderer to correct position:
                // TODO: test if this changed anything
                //currentLineRenderer.transform.position = _trackedObject.ToolTipPosition;
            }
        }
    }
}
