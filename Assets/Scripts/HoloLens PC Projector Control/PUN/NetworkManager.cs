using UnityEngine;
using Photon.Pun;
using TMPro;

public class NetworkManager : MonoBehaviourPunCallbacks
{
    public GameObject displayText; // feedback text
    public string roomName; // the room to join
    private GameObject camLoc; // the sample object to show the estimated position of the camera

    // Start is called before the first frame update
    void Start()
    {
        PhotonNetwork.ConnectUsingSettings();
    }

    // Join a room specified by the user
    public void Connect() {

        PhotonNetwork.JoinRoom(roomName);

    }
    // Provide feedback when failed to join a room
    public override void OnJoinRoomFailed(short returnCode, string message)
    {
        Debug.Log("Tried to join the room and failed.");
        displayText.GetComponent<TMP_Text>().text = "Tried to join the room and failed.";
    }
    // Provide feedback when joined a room
    public override void OnJoinedRoom()
    {
        Debug.Log("Joined a room!");
        displayText.GetComponent<TMP_Text>().text = "Joined  " + roomName;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    // Get the sample object
    public GameObject GetCamObject()
    {
        return camLoc;
    }
}
