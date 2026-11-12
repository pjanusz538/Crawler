using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform player;
    private Vector3 offset;

    void Start()
    {
        // Calculate the initial distance between the camera and the player
        offset = transform.position - player.position;
    }

    void LateUpdate()
    {
        // Only update position, do not touch transform.rotation
        transform.position = player.position + offset;
    }
}