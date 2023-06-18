using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine.SceneManagement;
using System.Collections.ObjectModel;

// public enum ControlType { PositionControl };

public class ArmFollowAgent : Agent
{
    private ArticulationBody[] articulationChain;
    private bool firstEpisode = true;
    private float startingDistanceToTarget;

    public Unity.Robotics.UrdfImporter.Control.ControlType control = Unity.Robotics.UrdfImporter.Control.ControlType.PositionControl;
    public float stiffness;
    public float damping;
    public float forceLimit;

    public Transform targetTransform;

    // Start is called before the first frame update
    void Start()
    {
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChain)
        {
            joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
        }
        startingDistanceToTarget = Vector3.Distance(targetTransform.position, articulationChain[9].transform.position);
    }

    public override void OnEpisodeBegin()
    {
        // Debug.Log("OnEpisodeBegin");
        if (!firstEpisode)
        {
            // Debug.Log("Resetting the Scene");
            SceneManager.LoadScene( SceneManager.GetActiveScene().buildIndex ) ;
        }
        firstEpisode = false;
    }
    

    // Update is called once per frame
    void Update()
    {
        float scaleFactor = .1f;
        float distanceChange = Vector3.Distance(targetTransform.position, articulationChain[9].transform.position) - startingDistanceToTarget;
        distanceChange = distanceChange / scaleFactor;
        distanceChange = Mathf.Abs(distanceChange);
        distanceChange = Mathf.Exp(distanceChange) - 1;
        SetReward(-distanceChange);
        // Debug.Log("distanceChange: " + distanceChange);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(targetTransform.localPosition[1]);
        sensor.AddObservation(articulationChain[1].transform.localRotation); // shoulder
        sensor.AddObservation(articulationChain[2].transform.localRotation); // arm
        sensor.AddObservation(articulationChain[3].transform.localRotation); // elbow
        sensor.AddObservation(articulationChain[4].transform.localRotation); // forearm
        sensor.AddObservation(articulationChain[5].transform.localRotation); // wrist
        sensor.AddObservation(articulationChain[6].transform.localRotation); // hand
        // sensor.AddObservation(articulationChain[9].transform.localRotation); // servo head

        // Debug.Log("targetTransform.localPosition[1]: " + targetTransform.localPosition[1]);
        // Debug.Log("articulationChain[1].transform.localRotation: " + articulationChain[1].transform.localRotation);
        // Debug.Log("articulationChain[2].transform.localRotation: " + articulationChain[2].transform.localRotation);
        // Debug.Log("articulationChain[3].transform.localRotation: " + articulationChain[3].transform.localRotation);
        // Debug.Log("articulationChain[4].transform.localRotation: " + articulationChain[4].transform.localRotation);
        // Debug.Log("articulationChain[5].transform.localRotation: " + articulationChain[5].transform.localRotation);
        // Debug.Log("articulationChain[6].transform.localRotation: " + articulationChain[6].transform.localRotation);
        // Debug.Log("articulationChain[9].transform.localRotation: " + articulationChain[9].transform.localRotation);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        UpdateDirections(actions);
    }

    private void UpdateDirections(ActionBuffers actions)
    {
        int index = 0;
        foreach (ArticulationBody joint in articulationChain)
        {

            if (joint.index != 1 && joint.index != 2 && joint.index != 3 && joint.index != 4 && joint.index != 5 && joint.index != 6)
            {
                continue;
            }

            JointControl jointControl = joint.gameObject.GetComponent<JointControl>();

            if (jointControl.controltype != (Unity.Robotics.UrdfImporter.Control.ControlType)control)
            {
                UpdateControlType(jointControl);
            }

            int direction = actions.DiscreteActions[index];

            if (direction == 1)
            {
                jointControl.direction = Unity.Robotics.UrdfImporter.Control.RotationDirection.Positive;
            }
            else if (direction == 2)
            {
                jointControl.direction = Unity.Robotics.UrdfImporter.Control.RotationDirection.Negative;
            }
            else
            {
                jointControl.direction = Unity.Robotics.UrdfImporter.Control.RotationDirection.None;
            }

            index++;
        }
    }

    public void UpdateControlType(JointControl joint)
    {
        joint.controltype = (Unity.Robotics.UrdfImporter.Control.ControlType)control;
        if (control == Unity.Robotics.UrdfImporter.Control.ControlType.PositionControl)
        {
            ArticulationDrive drive = joint.joint.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            joint.joint.xDrive = drive;
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // base.Heuristic(actionsOut);
    }

    public void OnCollisionEnter(Collision collision)
    {
        // Debug.Log("OnCollisionEnter Robot Side!!!!");
        SetReward(-100f);
        // EndEpisode();
    }
}
