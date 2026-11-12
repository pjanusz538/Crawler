using UnityEngine;

public class CrawlerFootTrigger : MonoBehaviour
{
    public CrawlerAgent agent;
    [Range(0, 3)]
    public int footIndex;

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Ground"))
        {
            agent.FootEnter(footIndex);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Ground"))
        {
            agent.FootExit(footIndex);
        }
    }
}
