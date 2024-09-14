using UnityEngine;

public class HoleController : MonoBehaviour
{
    public Material planeMaterial;
    public Vector2 holePosition;
    public Vector2 holeSize;

    void Update()
    {
        if (planeMaterial != null)
        {
            planeMaterial.SetVector("_HolePosition", new Vector4(holePosition.x, holePosition.y, 0, 0));
            planeMaterial.SetVector("_HoleSize", new Vector4(holeSize.x, holeSize.y, 0, 0));
        }
    }
}
