using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArmCollider : MonoBehaviour
{
    public GameObject arm;
    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.name == "Cube")
        {
            Debug.Log("Cube entered");            
            ArmGrabAgent armAgent = arm.GetComponent<ArmGrabAgent>();
            armAgent.ColliderEnter(other);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.gameObject.name == "Cube")
        {
            Debug.Log("Cube exited");
            ArmGrabAgent armAgent = arm.GetComponent<ArmGrabAgent>();
            armAgent.ColliderExit(other);
        }
    }
}
