using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PrintPos : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        Debug.Log(Time.time);
        Debug.Log("PrintPos: " + this.name);
        Debug.Log(this.transform.position);
    }

    // Update is called once per frame
    void Update()
    {
        // print time
        Debug.Log(Time.time);
        Debug.Log("PrintPos: " + this.name);
        Debug.Log(this.transform.position);
    }
}
