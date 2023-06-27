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
        // generate a random position around the shoulder link of max distance 1
        Vector3 randomPos = Random.insideUnitSphere;
        randomPos = randomPos.normalized;
        randomPos *= maxDistance;
        randomPos.y = Mathf.Abs(randomPos.y);
        randomPos.z = Mathf.Abs(randomPos.z);
        if ((randomPos.z > 0 && randomPos.z < 0.34 && randomPos.y > 1 && randomPos.y < 1.1 && randomPos.x > -0.1 && randomPos.x < 0.1) 
            || (randomPos.z > 0 && randomPos.z < 0.1 && randomPos.y > 0 && randomPos.y < 1.11 && randomPos.x > -0.1 && randomPos.x < 0.1))
        {
            this.Start();
        }
        tf.position = shoulderLink.position + randomPos;

    }

    // Update is called once per frame
    void Update()
    {
        if (count < 100)
        {
            count++;
        }
        else
        {
            count = 0;
            Debug.Log("Resetting position");
            this.Start();
        }
        
    }
}
