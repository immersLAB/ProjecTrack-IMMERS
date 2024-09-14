using MRTKExtensions.QRCodes;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaceRelative : MonoBehaviour
{
    public GameObject objectA; // Reference to object A
    public Vector3 bPositionRelativeToA; // B's position relative to A
    public Quaternion bRotationRelativeToA; // B's rotation relative to A

    // [SerializeField]
    private ContinuousQRTrackerController trackerController;

    // Start is called before the first frame update
    void Start()
    {
        trackerController.PositionSet.AddListener(PoseFound);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void PoseFound(Pose pose)
    {
        float delta = (objectA.transform.localScale.x+ objectA.transform.localScale.y)*0.25f;

        transform.localPosition = bPositionRelativeToA + delta * Vector3.right + delta * Vector3.forward;

        //// Calculate B's position and rotation in world space
        //Vector3 bPosition = objectA.transform.TransformPoint(bPositionRelativeToA + delta * Vector3.right + delta * Vector3.forward);
        //Quaternion bRotation = objectA.transform.rotation * bRotationRelativeToA;

        //transform.position = bPosition;
        //transform.rotation = bRotation;
    }
}



