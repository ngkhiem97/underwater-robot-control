using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Follow : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void FixedUpdate(){
        Vector3 velocity = new Vector3(0, 2f, 0);
        transform.position += velocity * Time.deltaTime;
    }
}
