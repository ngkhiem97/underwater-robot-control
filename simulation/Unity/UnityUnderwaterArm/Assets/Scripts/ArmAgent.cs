using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine.SceneManagement;
using System.Collections.ObjectModel;

// public enum ControlType { PositionControl };

public class ArmAgent : Agent
{
    private ArticulationBody[] articulationChain;
    private bool firstEpisode = true;
    private int stepCount = 0;

    public Unity.Robotics.UrdfImporter.Control.ControlType control = Unity.Robotics.UrdfImporter.Control.ControlType.PositionControl;
    public float stiffness;
    public float damping;
    public float forceLimit;

    // public float speed = 5f; // Units: degree/s
    // public float torque = 100f; // Units: Nm or N
    // public float acceleration = 5f;// Units: m/s^2 / degree/s^2

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
    }

    public override void OnEpisodeBegin()
    {
        stepCount = 0;
        // SceneManager.LoadScene(SceneManager.GetActiveScene().name);
        if (!firstEpisode)
        {
            Debug.Log("Resetting the Scene");
            SceneManager.LoadScene( SceneManager.GetActiveScene().buildIndex ) ;
        }
        firstEpisode = false;
    }
    

    // Update is called once per frame
    void Update()
    {
        // Print Observations
        // ReadOnlyCollection<float> observations = GetObservations();
        // string obsStr = "";
        // foreach (float obs in observations)
        // {
        //     obsStr += obs.ToString() + ", ";
        // }
        // Debug.Log("Observations: " + obsStr);

        float scaleFactor = 1f;
        float distanceToTarget = Vector3.Distance(targetTransform.localPosition, articulationChain[9].transform.localPosition);
        float scaledReward = distanceToTarget / scaleFactor;
        SetReward(-scaledReward);

        // Print Reward
        if (stepCount % 10 == 0)
        {
            Debug.Log("Reward: " + GetCumulativeReward());
        }
        stepCount++;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(targetTransform.localPosition);
        sensor.AddObservation(articulationChain[1].transform.localPosition); // shoulder
        sensor.AddObservation(articulationChain[2].transform.localPosition); // arm
        sensor.AddObservation(articulationChain[3].transform.localPosition); // elbow
        sensor.AddObservation(articulationChain[4].transform.localPosition); // forearm
        sensor.AddObservation(articulationChain[5].transform.localPosition); // wrist
        sensor.AddObservation(articulationChain[6].transform.localPosition); // hand
        sensor.AddObservation(articulationChain[9].transform.localPosition); // servo head
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        Debug.Log("OnActionReceived");
        Debug.Log("actions.DiscreteActions[0]: " + actions.DiscreteActions[0]); // shoulder
        Debug.Log("actions.DiscreteActions[1]: " + actions.DiscreteActions[1]); // arm
        Debug.Log("actions.DiscreteActions[2]: " + actions.DiscreteActions[2]); // elbow
        Debug.Log("actions.DiscreteActions[3]: " + actions.DiscreteActions[3]); // forearm
        Debug.Log("actions.DiscreteActions[4]: " + actions.DiscreteActions[4]); // wrist
        Debug.Log("actions.DiscreteActions[5]: " + actions.DiscreteActions[5]); // hand
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

    private void OnTriggerEnter(Collider other)
    {
        Debug.Log("OnTriggerEnter!!!!");
        SetReward(1.0f);
        EndEpisode();
    }
}
