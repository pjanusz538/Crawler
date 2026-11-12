using UnityEngine;

public class TargetPickup : MonoBehaviour
{
    public CrawlerAgent agent;

    private void OnTriggerEnter(Collider other)
    {
        // Wykryj robota lub jedn¹ z jego czêœci.
        CrawlerAgent foundAgent =
            other.GetComponentInParent<CrawlerAgent>();

        if (foundAgent != null)
        {
            foundAgent.CollectTarget();
        }
    }
}
