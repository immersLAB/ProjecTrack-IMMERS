using UnityEngine;
using TMPro;


public class SetCamera : MonoBehaviour
{
    public Camera cam; // the projector camera
    public GameObject camPrefab; // the sample object prefab
    public GameObject displayText; // feedback text

    public float AspectRatio = 16f / 9f;
    public float ThrowRatio = 1.5f; // Example distance:width ratio

    private GameObject camLoc; // the sample object

    // Set the Fov and aspect ratio of the projector
    void Start()
    {
        if (cam != null)
        {
            float verticalFOV = 2f * Mathf.Atan(0.5f / (AspectRatio * ThrowRatio)) * Mathf.Rad2Deg;

            cam.aspect = AspectRatio;
            cam.fieldOfView = verticalFOV;
            
        }

    }

    // Align the projector camera with the sample object
    void Update()
    {

        if (camLoc != null)
        {
            cam.transform.position = camLoc.transform.position;
            cam.transform.rotation = camLoc.transform.rotation;
        }
        else
        {
            GameObject[] objects = FindObjectsOfType<GameObject>();
            foreach (GameObject obj in objects)
            {
                if (obj.name.StartsWith(camPrefab.name))
                {
                    camLoc = obj;
                    Debug.Log("Camera location found!");
                    displayText.GetComponent<TMP_Text>().text = "Camera location found!";
                }
            }
        }

    }


}
