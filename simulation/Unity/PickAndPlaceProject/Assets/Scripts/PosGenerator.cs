using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PosGenerator : MonoBehaviour
{
    // Start is called before the first frame update
    public Transform shoulderLink;
    public float maxDistance;
    private Transform tf;
    private int count = 0;
    void Start()
    {
        tf = GetComponent<Transform>();
    }

    public void GenerateRandomPos()
    {
        // generate a random position around the shoulder link of max distance 1
        Vector3 randomPos = Random.insideUnitSphere;
        randomPos = randomPos.normalized;
        randomPos *= maxDistance;
        randomPos.y = Mathf.Abs(randomPos.y);
        randomPos.z = Mathf.Abs(randomPos.z);
        Vector3 combinedPos = shoulderLink.position + randomPos;
        if ((combinedPos.x >= -0.11 && combinedPos.x <= 0.11 && combinedPos.y >= 0.98 && combinedPos.y <= 1.11 && combinedPos.z <= 0.36) || 
            (combinedPos.x >= -0.11 && combinedPos.x <= 0.11 && randomPos.y >= 0 && combinedPos.y <= 1.11 && combinedPos.z <= 0.11))
        {
            this.GenerateRandomPos();
            return;
        }
        tf.position = combinedPos;
    }

    public void OnCollisionEnter(Collision collision)
    {
        Debug.Log("Collision");
        Debug.Log(collision.gameObject.name);
        Debug.Log(tf.position);
    }

    // Update is called once per frame
    // void Update()
    // {
    //     if (count < 2)
    //     {
    //         count++;
    //     }
    //     else
    //     {
    //         count = 0;
    //         // Debug.Log("Resetting position");
    //         GenerateRandomPos();
    //     }
        
    // }
}
