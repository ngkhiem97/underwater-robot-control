using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArmCollision : MonoBehaviour
{
    public GameObject arm;
    private void OnCollisionEnter(Collision collision)
    {
        // Debug.Log("Collision detected!!!!!!!!");
        // if (collision.gameObject.tag == "Box")
        // {
        //     Debug.Log("Box collision detected");
        // }
        ArmAgent armAgent = arm.GetComponent<ArmAgent>();
        armAgent.OnCollisionEnter(collision);
    }
}
