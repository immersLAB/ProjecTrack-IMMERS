using UnityEngine;
// using UnityEngine.UIElements;
using UnityEngine.UI;

public class CameraManager : MonoBehaviour
{
    public Camera[] cameras;
    private int currentCameraIndex = 0; // 0 for the main camera, 1 for the projector

    public GameObject canvas; // the UI
    private Image targetImage;
    private Sprite newSprite; // Holds the sprite to be loaded

    // Enable the main camera
    void Start()
    {
        EnableCamera(currentCameraIndex);
        DisableCamera((currentCameraIndex + 1) % cameras.Length);
        canvas.SetActive(true);
        targetImage = canvas.GetComponentInChildren<Image>();
    }

    void Update()
    {
        // Check for input to switch cameras
        if (Input.GetKeyDown(KeyCode.C))
        {
            // Disable the current camera
            DisableCamera(currentCameraIndex);

            // Switch to the next camera
            currentCameraIndex = (currentCameraIndex + 1) % cameras.Length;

            // Enable the new current camera
            EnableCamera(currentCameraIndex);

            // Check if the game is in full-screen mode
            bool isFullScreen = Screen.fullScreen;

        }

        if (Input.GetKeyDown(KeyCode.F))
        {
            Debug.Log("F key pressed");
            // Toggle between full-screen and windowed mode
            Screen.fullScreen = !Screen.fullScreen;
            Debug.Log(Screen.fullScreen.ToString());
        }

        // Check if the projector is active
        if (cameras[1] != null && cameras[1].enabled)
        {
            // Hide the UI
            canvas.SetActive(false);
        }
        else
        {
            // Show the UI
            canvas.SetActive(true);
        }

        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            newSprite = Resources.Load<Sprite>("Image/Slide1");
            if (newSprite == null)
            {
                Debug.LogError("Sprite not found at path: Resources/Image/Slide1");
            }

            if (targetImage != null && newSprite != null)
            {
                targetImage.sprite = newSprite;
            }
            else
            {
                Debug.LogError("Target Image or Sprite is not assigned.");
            }
        }

        if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            newSprite = Resources.Load<Sprite>("Image/Slide2");
            if (newSprite == null)
            {
                Debug.LogError("Sprite not found at path: Resources/Image/Slide1");
            }

            if (targetImage != null && newSprite != null)
            {
                targetImage.sprite = newSprite;
            }
            else
            {
                Debug.LogError("Target Image or Sprite is not assigned.");
            }
        }
    }

    void EnableCamera(int index)
    {
        cameras[index].enabled = true;
        cameras[index].gameObject.SetActive(true);
    }

    void DisableCamera(int index)
    {
        cameras[index].enabled = false;
        cameras[index].gameObject.SetActive(false);
    }


}
