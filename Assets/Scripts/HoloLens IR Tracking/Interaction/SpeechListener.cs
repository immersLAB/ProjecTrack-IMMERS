using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpeechListener : MonoBehaviour
{
    // Initializing variables
    public GameObject placedSphere; // Sphere prefab
    public GameObject placedTumor; // Tumor (or any 3D object for a given demo) to place in the 'hole'
    private IRTrackedObject _trackedObject;
    private bool isDrawing = false; // Boolean to designate start vs. stop drawing


    // Start is called before the first frame update
    void Start()
    {
        _trackedObject = FindObjectOfType<IRTrackedObject>();
    }

    // Function for a placing a single sphere object at once (On Click):
    public void onPlacedObject()
    {
        if (_trackedObject != null)
        {
            // Store pose (position and rotation) of IR probe tool tip
            Vector3 spherePosition = _trackedObject.ToolTipPosition; // tool tip position as 3D vector
            Quaternion sphereRotation = _trackedObject.ToolTipRotation; // tool tip rotation as quaternion

            Instantiate(placedSphere, spherePosition, sphereRotation); // Place a new instantiation of the sphere prefab
        }
    }

    public void placeAtToolTip()
    {
        if (_trackedObject && placedTumor != null)
        {
            // Store pose (position and rotation) of IR probe tool tip
            Vector3 objectPosition = _trackedObject.ToolTipPosition; // tool tip position as 3D vector
            Quaternion objectRotation = _trackedObject.ToolTipRotation; // tool tip rotation as quaternion

            placedTumor.transform.position = objectPosition;
            placedTumor.transform.rotation = objectRotation;
        }
            // TEST VERSION: Update the position and rotation of the HoleHandle based on the Tool Tip
            // TODO: adapt this ^ to place tumor where tool tip is
            //GameObject holeHandle = objectShare[2]; // Assuming HoleHandle is objectShare[2] 
            //if (holeHandle != null) 
            //{
            //    holeHandle.transform.position = _trackedObject.ToolTipPosition; 
            //    holeHandle.transform.rotation = _trackedObject.ToolTipRotation;
            //}

            // TODO: add function to make parent of all the 'startDrawing' markings -> then be able to move the hole
    }

    // Function for 'start drawing' -> to run Update()'s body code and continuously place sphere prefabs
    public void StartDrawing()
    {
        isDrawing = true;
    }

    // Function for 'stop drawing' -> to NOT run Update()'s body code and stop placing sphere prefabs
    public void StopDrawing()
    {
        isDrawing = false;
    }

    // Update is called once per frame
    void Update()
    {
        // Continuously place sphere prefabs w/ ToolTip's current transform (position and rotation):
        if (isDrawing && _trackedObject != null) // If isDrawing is true (triggered in inspector settings via 'start drawing') and tracked object exists:
        {
            Vector3 spherePosition = _trackedObject.ToolTipPosition;
            Quaternion sphereRotation = _trackedObject.ToolTipRotation;

            Instantiate(placedSphere, spherePosition, sphereRotation); // Place a new instantiation of the sphere prefab
        }
    }
}
// TODO 08/09: add things during update, once changing to a start / stop drawing functionality rather than an On Click()
// 08/09:
    // 1. Test new build w/ shorter voice command time (.3 -> .1 -> .05)
        // Implemented, but could be improved
    // 2. Multiple voice commands (start drawing vs stop drawing)
        // Implemented with MULTIPLE stateful interactable scripts on the SpeechController empty
    // 3. Test Yuxuan's projector control versions / scenes
        // Done

// 08/12:
    // 1. Test Yuxuan's projector control versions / scenes + troubleshoot issues
        // Aperture-PC
        // Aperture-HL
    // 2. Integrate w/ Yuxuan's project -> send sphere prefabs synchronized to PC
        // ^^ work w/ Wally and Yuxuan to integrate
    // 3. Work w/ Yuxuan for remaining next steps to control projector
        // spheres or 2D shapes?
        // linerenderer
        // Add wally and yuxuan and repo -> commit to IMMERS