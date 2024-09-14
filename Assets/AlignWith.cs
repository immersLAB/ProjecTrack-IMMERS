using UnityEngine;

public class AlignWith : MonoBehaviour
{
    public Transform target; // Reference to the other sprite's transform

    void Update()
    {
        if (target != null)
        {
            // Match position
            transform.position = target.position;

            transform.rotation = target.rotation;

            // Match scale
            transform.localScale = target.localScale;
        }
    }
}
