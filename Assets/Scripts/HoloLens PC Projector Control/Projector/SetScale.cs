using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetScale : MonoBehaviour
{
    public float diameter = 5f;
    public float width = 33.87f;
    public float ratio = 1.5933f;
    // Start is called before the first frame update
    void Start()
    {
        transform.localScale = new Vector3(diameter/width, diameter/width, ratio);
        
    }

    // Update is called once per frame
    void Update()
    {
        // transform.localScale = new Vector3(0.2094679f, 0.2094679f, 1.5f);
    }
}
