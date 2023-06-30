using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class ArmAgent : Agent
{
    // Start is called before the first frame update
    public float jointFriction = 10f;
    public float angularDamping = 10f;
    public float stiffness = 100000f;
    public float damping = 1000f;
    public float forceLimit = 10000f;
    public Rigidbody target;
    private const int k_NumRobotJoints = 6;
    private ArticulationBody[] articulationChain;
    private float[] initialJointTargets = new float[14];
    private List<GameObject> contacts = new List<GameObject>();
    private GameObject gripperBase;
    private int gripperState = 1;
    void Start()
    {
        // configuring the joints
        articulationChain = GetComponentsInChildren<ArticulationBody>();
        ConfigJoints();

        // getting the gripper's contacts
        string[] contactsNames = {"contact_left_1", "contact_left_2", "contact_right_1", "contact_right_2"};
        foreach (string contactName in contactsNames)
        {
            contacts.Add(GameObject.Find(contactName));
        }

        // getting the gripper's base
        gripperBase = GameObject.Find("gripper_base");
    }

    public override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(GetGripperPosition());
        sensor.AddObservation(GetGripperOrientation());
        sensor.AddObservation(GetTargetPosition());
        sensor.AddObservation(GetTargetOrientation());
        sensor.AddObservation(GetDeltaPosition());
        sensor.AddObservation(GetDeltaOrientation());
        sensor.AddObservation(GetTargetRelativeVelocity());
        sensor.AddObservation(GetTargetRelativeAngularVelocity());
    }

    public override void OnActionReceived(ActionBuffers vectorAction)
    {
        Vector3 gripperPosition = new Vector3(vectorAction.ContinuousActions[0], 
                                              vectorAction.ContinuousActions[1], 
                                              vectorAction.ContinuousActions[2]);
        bool gripperOpen = vectorAction.DiscreteActions[0] == 1;
        ControlGripper(gripperPosition, gripperOpen);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // base.Heuristic(actionsOut);
    }

    // Update is called once per frame
    void Update()
    {
        // Print gripper position and orientation
        // Debug.Log("Gripper position: " + GetGripperPosition());
        // Debug.Log("Gripper orientation: " + GetGripperOrientation());
    }

    private void ControlGripper(Vector3 position, bool open)
    {
        
    }

    private void ConfigJoints()
    {
        int jointTargetIndex = 0;
        for (int i = 0; i < k_NumRobotJoints; i++)
        {
            ArticulationBody joint = articulationChain[i];

            // adjusting joint control parameters
            joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = jointFriction;
            joint.angularDamping = angularDamping;

            // adjusting joint drive parameters
            ArticulationDrive drive = joint.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            joint.xDrive = drive;

            // saving initial joint targets
            initialJointTargets[jointTargetIndex] = joint.xDrive.target;
            jointTargetIndex++;
        }
    }

    private Vector3 GetGripperPosition()
    {
        Vector3 gripperPosition = new Vector3();
        foreach (GameObject contact in contacts)
        {
            gripperPosition += contact.transform.position;
        }
        gripperPosition /= contacts.Count;
        return gripperPosition;
    }

    private Vector3 GetGripperOrientation()
    {
        return Utils.ConvertRotation(gripperBase.transform.rotation.eulerAngles);
    }

    private double[] GetCurrentJointConfig()
    {
        double[] joints = new double[k_NumRobotJoints];
        int jointIndex = 0;
        for (int i = 0; i < k_NumRobotJoints; i++)
        {
            ArticulationBody joint = articulationChain[i];
            joints[jointIndex] = joint.jointPosition[0];
            jointIndex++;
        }
        return joints;
    }

    private Vector3 GetTargetPosition()
    {
        return target.transform.position;
    }

    private Vector3 GetTargetOrientation()
    {
        return Utils.ConvertRotation(target.transform.rotation.eulerAngles);
    }

    private Vector3 GetDeltaPosition()
    {
        return GetTargetPosition() - GetGripperPosition();
    }

    private Vector3 GetDeltaOrientation()
    {
        return GetTargetOrientation() - GetGripperOrientation();
    }

    private Vector3 GetGripperVelocity()
    {
        Vector3 gripperVelocity = new Vector3();
        foreach (GameObject contact in contacts)
        {
            gripperVelocity += contact.GetComponent<ArticulationBody>().velocity;
        }
        gripperVelocity /= contacts.Count;
        return gripperVelocity;
    }

    private Vector3 GetGripperAngularVelocity()
    {
        Vector3 gripperAngularVelocity = new Vector3();
        foreach (GameObject contact in contacts)
        {
            gripperAngularVelocity += contact.GetComponent<ArticulationBody>().angularVelocity;
        }
        gripperAngularVelocity /= contacts.Count;
        return gripperAngularVelocity;
    }

    private Vector3 GetTargetRelativeVelocity()
    {
        return target.velocity - GetGripperVelocity();
    }

    private Vector3 GetTargetRelativeAngularVelocity()
    {
        return target.angularVelocity -  GetGripperAngularVelocity();
    }
}
