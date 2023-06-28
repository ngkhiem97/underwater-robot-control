using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class GripperLogger : MonoBehaviour
{
    // Start is called before the first frame update
    public string logfile;
    void Start()
    {
        // print position and velocity with 4 floating point precision
        System.IO.File.AppendAllText(logfile, "Gripper position: (" + transform.position.x + ", " + transform.position.y + ", " + transform.position.z + ")"
                                                + " with velocity: " + GetComponent<ArticulationBody>().velocity.ToString("F4")
                                                + " at time: " + Time.time.ToString() + "\n");
    }

    // Update is called once per frame
    void Update()
    {
        // if the gripper is moving, log the position
        if (GetComponent<ArticulationBody>().velocity.magnitude > 0.01)
        {
            System.IO.File.AppendAllText(logfile, "Gripper position: (" + transform.position.x + ", " + transform.position.y + ", " + transform.position.z + ")"
                                                    + " with velocity: " + GetComponent<ArticulationBody>().velocity.ToString("F4")
                                                    + " at time: " + Time.time.ToString() + "\n");
        }
    }
}
