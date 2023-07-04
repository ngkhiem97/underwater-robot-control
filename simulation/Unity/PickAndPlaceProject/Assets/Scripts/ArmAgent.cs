using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Linq;

public class ArmAgent : Agent
{
    // Start is called before the first frame update
    public float jointFriction = 10f;
    public float angularDamping = 10f;
    public float stiffness = 100000f;
    public float damping = 1000f;
    public float forceLimit = 10000f;
    public Rigidbody target;
    public Vector3 testPosition;
    public Vector3 testOrientation;
    private const int k_NumRobotJoints = 6;
    private ArticulationBody[] articulationChain;
    private float[] initialJointTargets = new float[6];
    private List<GameObject> contacts = new List<GameObject>();
    private GameObject gripperBase;
    private int gripperState = 1;
    private int gripperStateCmd = 1;

    private ROSConnection Ros;
    private const string RosServiceName = "niryo_moveit";
    private const float JointAssignmentWait = 0.025f;
    private const float armReach = 0.54f;
    private ArticulationBody LeftGripper;
    private ArticulationBody RightGripper;
    private bool gripperControlInAction = false;

    private string[] controlLinkNames =
        { "world/base_link/shoulder_link", "/arm_link", "/elbow_link", "/forearm_link", "/wrist_link", "/hand_link" };
    private ArticulationBody[] controlLinks = new ArticulationBody[k_NumRobotJoints];
    private float[] LastContinuousAction = null;
    private int LastDiscreteAction = -1;
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

        // Get ROS connection static instance
        Ros = ROSConnection.GetOrCreateInstance();
        Ros.RegisterRosService<PointToPointServiceRequest, PointToPointServiceResponse>(RosServiceName);

        // Find all robot joints in Awake() and add them to the jointArticulationBodies array.
        string linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            Debug.Log("Link name: " + linkName);
            linkName += controlLinkNames[i];
            controlLinks[i] = this.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // Find left and right fingers
        var rightGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        var leftGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";
        RightGripper = this.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        LeftGripper = this.transform.Find(leftGripper).GetComponent<ArticulationBody>();
    }

    public override void OnEpisodeBegin()
    {
        target.GetComponent<PosGenerator>().GenerateRandomPos();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(GetGripperPosition());
        sensor.AddObservation(GetGripperOrientation());
        sensor.AddObservation(GetCurrentGripperState());
        sensor.AddObservation(GetGripperControlStatus());
        sensor.AddObservation(GetTargetPosition());
        sensor.AddObservation(GetTargetOrientation());
        sensor.AddObservation(GetDeltaPosition());
        sensor.AddObservation(GetDeltaOrientation());
        sensor.AddObservation(GetTargetRelativeVelocity());
        sensor.AddObservation(GetTargetRelativeAngularVelocity());
    }

    public override void OnActionReceived(ActionBuffers vectorAction)
    {
        if (CheckLastAction(vectorAction))
        {
            return;
        }
        if (LastContinuousAction == null)
        {
            LastContinuousAction = new float[5];
        }
        LastContinuousAction[0] = vectorAction.ContinuousActions[0];
        LastContinuousAction[1] = vectorAction.ContinuousActions[1];
        LastContinuousAction[2] = vectorAction.ContinuousActions[2];
        LastContinuousAction[3] = vectorAction.ContinuousActions[3];
        LastContinuousAction[4] = vectorAction.ContinuousActions[4];
        LastDiscreteAction = vectorAction.DiscreteActions[0];
        float x_new = Utils.ConvertRange(vectorAction.ContinuousActions[0], -1, 1, -1, 1);
        float y_new = Utils.ConvertRange(vectorAction.ContinuousActions[1], -1, 1, 0, 1);
        float z_new = Utils.ConvertRange(vectorAction.ContinuousActions[2], -1, 1, 0, 1);
        Vector3 gripperPosition = Utils.ScaleToLength(new Vector3(x_new, y_new, z_new), 1);
        gripperPosition = Utils.ScaleToScale(gripperPosition, 1, armReach * 0.75f);
        gripperPosition = Utils.ShiftVector(gripperPosition, controlLinks[1].transform.position.x, 
                                                             controlLinks[1].transform.position.y, 
                                                             controlLinks[1].transform.position.z);
        Vector3 gripperOrientation = new Vector3(vectorAction.ContinuousActions[3],
                                                 Utils.GetYRotationFromXZ(gripperPosition.x,
                                                                          gripperPosition.z),
                                                 vectorAction.ContinuousActions[4]);
        int gripperOpen = vectorAction.DiscreteActions[0];
        ControlGripper(gripperPosition, gripperOrientation, gripperOpen);
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // only set if current position is far from test position
        if (Vector3.Distance(GetGripperPosition(), testPosition) > 0.02f)
        {
            actionsOut.ContinuousActions.Array[0] = testPosition.x;
            actionsOut.ContinuousActions.Array[1] = testPosition.y;
            actionsOut.ContinuousActions.Array[2] = testPosition.z;
            actionsOut.ContinuousActions.Array[3] = testOrientation.x * Mathf.Deg2Rad;
            // actionsOut.ContinuousActions.Array[4] = testOrientation.y * Mathf.Deg2Rad;
            actionsOut.ContinuousActions.Array[4] = testOrientation.z * Mathf.Deg2Rad;
            actionsOut.DiscreteActions.Array[0] = 0;
        }
    }

    // Update is called once per frame
    void Update()
    {
        // Print gripper position and orientation
        // Debug.Log("Gripper position: " + GetGripperPosition());
        // Debug.Log("Gripper orientation: " + GetGripperOrientation());
        // Debug.Log("arm_link position: " + controlLinks[1].transform.position);
        // print out the step count
        // Debug.Log("Step count: " + StepCount);
        // print gripper orientation
        // Debug.Log("Gripper orientation: " + GetGripperOrientation());
    }

    private bool CheckLastAction(ActionBuffers vectorAction)
    {
        if (LastContinuousAction == null || LastDiscreteAction == -1)
        {
            return false;
        }
        if (LastContinuousAction[0] == vectorAction.ContinuousActions[0] &&
            LastContinuousAction[1] == vectorAction.ContinuousActions[1] &&
            LastContinuousAction[2] == vectorAction.ContinuousActions[2] &&
            LastContinuousAction[3] == vectorAction.ContinuousActions[3] &&
            LastContinuousAction[4] == vectorAction.ContinuousActions[4] &&
            LastDiscreteAction == vectorAction.DiscreteActions[0])
        {
            return true;
        }
        return false;
    }

    private void ControlGripper(Vector3 position, Vector3 orientation_rad, int open)
    {
        if (gripperControlInAction)
        {
            return;
        }
        Debug.Log("Control gripper with position: " + position + " and orientation: " + orientation_rad);
        PublishJoints(position, orientation_rad);
        gripperStateCmd = open;
    }

    private void ConfigJoints()
    {
        int jointTargetIndex = 0;
        foreach (ArticulationBody joint in articulationChain)
        {

            // adjusting joint control parameters
            joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = jointFriction;
            joint.angularDamping = angularDamping;

            if (!controlLinkNames.Contains(joint.name))
            {
                continue;
            }

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
            ArticulationBody joint = controlLinks[i];
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

    private float GetCurrentGripperState()
    {
        return gripperState;
    }

    private int GetGripperControlStatus()
    {
        return gripperControlInAction ? 1 : 0;
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

    private NiryoMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new NiryoMoveitJointsMsg();
        joints.joints = new double[k_NumRobotJoints];
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = controlLinks[i].xDrive.target * Mathf.Deg2Rad;
        }

        return joints;
    }

    private void PublishJoints(Vector3 destination, Vector3 orientation_rad)
    {
        Debug.Log("Publishing joints.");
        gripperControlInAction = true;
        var request = new PointToPointServiceRequest();
        request.joints_input = CurrentJointConfig();

        Vector3 offset = new Vector3(0, 0, 0);

        // Finish Pose
        request.finish_pose = new PoseMsg
        {
            position = (destination + offset).To<FLU>(),
            orientation = Quaternion.Euler(orientation_rad.x * Mathf.Rad2Deg, 
                                           orientation_rad.y * Mathf.Rad2Deg, 
                                           orientation_rad.z * Mathf.Rad2Deg).To<FLU>()
        };

        // Freeze current step

        Ros.SendServiceMessage<PointToPointServiceResponse>(RosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(PointToPointServiceResponse response)
    {
        if (response.trajectory != null)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            gripperControlInAction = false;
            Debug.LogError("No trajectory returned from PointToPointService.");
        }
    }

    IEnumerator ExecuteTrajectories(PointToPointServiceResponse response)
    {
        if (response.trajectory != null)
        {
            // For every robot pose in trajectory plan
            foreach (var t in response.trajectory.joint_trajectory.points)
            {
                var jointPositions = t.positions;
                var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                // Set the joint values for every joint
                for (var joint = 0; joint < k_NumRobotJoints; joint++)
                {
                    var joint1XDrive = controlLinks[joint].xDrive;
                    joint1XDrive.target = result[joint];
                    controlLinks[joint].xDrive = joint1XDrive;
                }

                // Wait for robot to achieve pose for all joint assignments
                yield return new WaitForSeconds(JointAssignmentWait);
            }
            if (gripperStateCmd == 0)
            {
                OpenGripper();
            }
            else
            {
                CloseGripper();
            }
            gripperState = gripperStateCmd;
        }
        gripperControlInAction = false;
    }

    void CloseGripper()
    {
        var leftDrive = LeftGripper.xDrive;
        var rightDrive = RightGripper.xDrive;

        leftDrive.target = -0.01f;
        rightDrive.target = 0.01f;

        LeftGripper.xDrive = leftDrive;
        RightGripper.xDrive = rightDrive;
    }

    void OpenGripper()
    {
        var leftDrive = LeftGripper.xDrive;
        var rightDrive = RightGripper.xDrive;

        leftDrive.target = 0.01f;
        rightDrive.target = -0.01f;

        LeftGripper.xDrive = leftDrive;
        RightGripper.xDrive = rightDrive;
    }
}
