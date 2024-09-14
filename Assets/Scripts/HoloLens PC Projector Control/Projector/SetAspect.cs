using UnityEngine;

public class SetAspect : MonoBehaviour
{
    public Camera cam;

    public float AspectRatio;

    // Start is called before the first frame update
    void Start()
    {
        if (cam != null)
        {
            cam.aspect = AspectRatio;
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (cam != null)
        {
            cam.aspect = AspectRatio;
        }
    }
}
