using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Randomization : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        // Randomize the position of the object around the original position
        transform.position = new Vector3(transform.position.x + Random.Range(-0.1f, 0.1f), transform.position.y, transform.position.z + Random.Range(-0.1f, 0.1f));
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
