using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class TestAgent : Agent
{
    int numSteps = 0;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    public override void OnEpisodeBegin()
    {
        Debug.Log("Episode Begin");
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Debug.Log("Collecting Observations");
        sensor.AddObservation(numSteps);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        Debug.Log("Action Received, numSteps: " + numSteps);
    }

    // Update is called once per frame
    void Update()
    {
        numSteps++;
        if (numSteps == 10)
        {
            numSteps = 0;
            EndEpisode();
        }
    }
}
