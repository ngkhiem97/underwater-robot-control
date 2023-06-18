using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine.SceneManagement;
using System.Collections.ObjectModel;

// public enum ControlType { PositionControl };

public class ArmGrabAgent : Agent
{
    private ArticulationBody[] articulationChain;
    private bool firstEpisode = true;
    private int collisionCount = 0;
    private int gripperState = 0;

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
        float scaleFactor = 0.1f;
        float distance = Vector3.Distance(targetTransform.position, articulationChain[9].transform.position);
        distance = distance * scaleFactor;
        distance = Mathf.Pow(distance, 2);
        SetReward(-distance);
        // Debug.Log("distance: " + -distance);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // check if scene is loaded
        while (articulationChain == null)
        {
            articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        }
        sensor.AddObservation(articulationChain[1].transform.position - targetTransform.position);
        sensor.AddObservation(articulationChain[2].transform.position - targetTransform.position);
        sensor.AddObservation(articulationChain[3].transform.position - targetTransform.position);
        sensor.AddObservation(articulationChain[4].transform.position - targetTransform.position);
        sensor.AddObservation(articulationChain[5].transform.position - targetTransform.position);
        sensor.AddObservation(articulationChain[6].transform.position - targetTransform.position);
        sensor.AddObservation(articulationChain[8].transform.position - targetTransform.position);
        sensor.AddObservation(articulationChain[11].transform.position - targetTransform.position);
        sensor.AddObservation(articulationChain[13].transform.position - targetTransform.position);
        sensor.AddObservation(targetTransform.gameObject.GetComponent<Rigidbody>().velocity);

        sensor.AddObservation(articulationChain[1].xDrive.target); // shoulder
        sensor.AddObservation(articulationChain[2].xDrive.target); // arm
        sensor.AddObservation(articulationChain[3].xDrive.target); // elbow
        sensor.AddObservation(articulationChain[4].xDrive.target); // forearm
        sensor.AddObservation(articulationChain[5].xDrive.target); // wrist
        sensor.AddObservation(articulationChain[6].xDrive.target); // hand
        sensor.AddObservation(articulationChain[11].xDrive.target); // right gripper
        sensor.AddObservation(articulationChain[13].xDrive.target); // left gripper

        sensor.AddObservation(articulationChain[1].velocity); // shoulder
        sensor.AddObservation(articulationChain[2].velocity); // arm
        sensor.AddObservation(articulationChain[3].velocity); // elbow
        sensor.AddObservation(articulationChain[4].velocity); // forearm
        sensor.AddObservation(articulationChain[5].velocity); // wrist
        sensor.AddObservation(articulationChain[6].velocity); // hand
        sensor.AddObservation(articulationChain[8].velocity); // base

        sensor.AddObservation(articulationChain[1].angularVelocity); // shoulder
        sensor.AddObservation(articulationChain[2].angularVelocity); // arm
        sensor.AddObservation(articulationChain[3].angularVelocity); // elbow
        sensor.AddObservation(articulationChain[4].angularVelocity); // forearm
        sensor.AddObservation(articulationChain[5].angularVelocity); // wrist
        sensor.AddObservation(articulationChain[6].angularVelocity); // hand
        sensor.AddObservation(articulationChain[8].angularVelocity); // base

        sensor.AddObservation(articulationChain[0].transform.position - articulationChain[1].transform.position); // shoulder
        sensor.AddObservation(articulationChain[0].transform.position - articulationChain[2].transform.position); // arm
        sensor.AddObservation(articulationChain[0].transform.position - articulationChain[3].transform.position); // elbow
        sensor.AddObservation(articulationChain[0].transform.position - articulationChain[4].transform.position); // forearm
        sensor.AddObservation(articulationChain[0].transform.position - articulationChain[5].transform.position); // wrist
        sensor.AddObservation(articulationChain[0].transform.position - articulationChain[6].transform.position); // hand
        sensor.AddObservation(articulationChain[0].transform.position - articulationChain[8].transform.position); // base
        sensor.AddObservation(articulationChain[0].transform.position - articulationChain[11].transform.position); // right gripper
        sensor.AddObservation(articulationChain[0].transform.position - articulationChain[13].transform.position); // left gripper
        sensor.AddObservation(articulationChain[8].transform.position - articulationChain[11].transform.position); // right gripper
        sensor.AddObservation(articulationChain[8].transform.position - articulationChain[13].transform.position); // left gripper

        sensor.AddObservation(gripperState); // gripper

        // print joint velocities
        // Debug.Log("joint velocities: ");
        // Debug.Log(articulationChain[1].angularVelocity); // shoulder
        // Debug.Log(articulationChain[2].angularVelocity); // arm
        // Debug.Log(articulationChain[3].angularVelocity); // elbow
        // Debug.Log(articulationChain[4].angularVelocity); // forearm
        // Debug.Log(articulationChain[5].angularVelocity); // wrist
        // Debug.Log(articulationChain[6].angularVelocity); // hand
        // Debug.Log(articulationChain[8].angularVelocity); // base
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

        int gripper = actions.DiscreteActions[6];
        if (gripper == 0)
        {
            ArticulationBody rightGripper = articulationChain[11];
            ArticulationBody leftGripper = articulationChain[13];
            JointControl leftGripperControl = leftGripper.gameObject.GetComponent<JointControl>();
            JointControl rightGripperControl = rightGripper.gameObject.GetComponent<JointControl>();
            leftGripperControl.direction = Unity.Robotics.UrdfImporter.Control.RotationDirection.Positive;
            rightGripperControl.direction = Unity.Robotics.UrdfImporter.Control.RotationDirection.Negative;
            gripperState = 0;
        }
        else
        {
            ArticulationBody rightGripper = articulationChain[11];
            ArticulationBody leftGripper = articulationChain[13];
            JointControl leftGripperControl = leftGripper.gameObject.GetComponent<JointControl>();
            JointControl rightGripperControl = rightGripper.gameObject.GetComponent<JointControl>();
            leftGripperControl.direction = Unity.Robotics.UrdfImporter.Control.RotationDirection.Negative;
            rightGripperControl.direction = Unity.Robotics.UrdfImporter.Control.RotationDirection.Positive;
            gripperState = 1;
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

    public void ColliderEnter(Collider collider)
    {
        collisionCount++;
        SetReward(0.25f);
        if (collisionCount == 4)
        {
            SetReward(1f);
            EndEpisode();
        }
    }

    public void ColliderExit(Collider collider)
    {
        collisionCount--;
    }
}
