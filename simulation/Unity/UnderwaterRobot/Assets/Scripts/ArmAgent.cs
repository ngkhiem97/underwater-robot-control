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
using UnityEngine.SceneManagement;
using Unity.MLAgents.SideChannels;

public class ArmAgent : Agent
{
    // Start is called before the first frame update
    public float jointFriction = 10f;
    public float angularDamping = 10f;
    public float stiffness = 10000f;
    public float damping = 100f;
    public float forceLimit = 1000f;
    public Rigidbody target;
    public Vector3 testPosition;
    public Vector3 testOrientation;
    private const int k_NumRobotJoints = 6;
    private ArticulationBody[] articulationChain;
    private float[] initialJointTargets = new float[k_NumRobotJoints];
    private List<GameObject> contacts = new List<GameObject>();
    private GameObject gripperBase;
    private int gripperState = 1;
    private int gripperStateCmd = 1;

    private ROSConnection Ros;
    private const string RosServiceName = "niryo_moveit";
    private const float JointAssignmentWait = 0.001f;
    private const float GripperControlWait = 0.002f;
    private const float armReach = 0.54f;
    private ArticulationBody LeftGripper;
    private ArticulationBody RightGripper;
    private bool gripperControlInAction = false;
    private int gripperControlForTimeSteps = 0;

    private string[] controlLinkNames =
        { "world/base_link/shoulder_link", "/arm_link", "/elbow_link", "/forearm_link", "/wrist_link", "/hand_link" };
    private ArticulationBody[] controlLinks = new ArticulationBody[k_NumRobotJoints];
    private float[] LastContinuousAction = null;
    private int numContactEntered = 0;
    private bool started = false;
    FloatLogSideChannel floatChannel;
    private float lastZOrientation = 0f;
    private bool stop = false;
    void Start()
    {
        // configuring the joints
        articulationChain = GetComponentsInChildren<ArticulationBody>();

        // getting the gripper's contacts
        string[] contactsNames = {"contact_left", "contact_right"};
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
            linkName += controlLinkNames[i];
            controlLinks[i] = this.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // ConfigJoints();
        for (var joint = 0; joint < k_NumRobotJoints; joint++)
        {
            var jointXDrive = controlLinks[joint].xDrive;
            initialJointTargets[joint] = jointXDrive.target;
        }

        // Find left and right fingers
        var rightGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        var leftGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";
        RightGripper = this.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        LeftGripper = this.transform.Find(leftGripper).GetComponent<ArticulationBody>();

        // Set initial target position
        started = true;
        
        // Add string side channel
        floatChannel = new FloatLogSideChannel();
        SideChannelManager.RegisterSideChannel(floatChannel);
    }

    public override void OnEpisodeBegin()
    {
        stop = false;
        target.GetComponent<PosGenerator>().ResetPos();
        for (var joint = 0; joint < k_NumRobotJoints; joint++)
        {
            var joint1XDrive = controlLinks[joint].xDrive;
            joint1XDrive.target = initialJointTargets[joint];
            controlLinks[joint].xDrive = joint1XDrive;
        }
        if (started)
        {
            return;
        }
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(GetGripperPosition());
        sensor.AddObservation(GetTargetPosition());
        sensor.AddObservation(GetDeltaPosition());
        sensor.AddObservation(GetTargetRelativeVelocity());
    }

    public override void OnActionReceived(ActionBuffers vectorAction)
    {
        if (CheckAction(vectorAction)) // this is to prevent the (0, 0, 0) action from being sent from the decision requester
        {
            return;
        }

        if (stop)
        {
            return;
        }

        // start grasping
        if (GetDeltaPosition().magnitude < 0.04f)
        {
            Debug.Log("Reached target at: " + GetTargetPosition());
            Vector3 offset = new Vector3(0.01f, -0.04f, 0.04f);
            Vector3 gripperPosition_ = GetTargetPosition() - offset;
            Vector3 gripperOrientation_ = ProcessGripperOrientation(gripperPosition_);
            gripperOrientation_.z = lastZOrientation;
            ControlGripper(gripperPosition_, gripperOrientation_, 1);
            stop = true;
            return;
        }

        // if not grasping, continue moving to target
        Vector3 gripperPosition = ProcessGripperPosition(vectorAction);
        Vector3 gripperOrientation = ProcessGripperOrientation(gripperPosition);
        ControlGripper(gripperPosition, gripperOrientation, 0);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // only set if current position is far from test position
        // if (Vector3.Distance(GetGripperPosition(), testPosition) > 0.02f)
        // {
        //     actionsOut.ContinuousActions.Array[0] = testPosition.x;
        //     actionsOut.ContinuousActions.Array[1] = testPosition.y;
        //     actionsOut.ContinuousActions.Array[2] = testPosition.z;
        // }
    }

    // Update is called once per frame
    void Update()
    {
        // Check for "Invalid worldAABB. Object is too large or too far away from the origin" error
        foreach (ArticulationBody joint in articulationChain) 
        {
            if (float.IsNaN(joint.transform.position.x) ||
                float.IsNaN(joint.transform.position.y) ||
                float.IsNaN(joint.transform.position.z))
            {
                Ros.Disconnect();
                SceneManager.LoadScene(SceneManager.GetActiveScene().name);
            }
        }
        if (gripperControlInAction) 
        {
            gripperControlForTimeSteps++;
            if (gripperControlForTimeSteps > 1000) {
                Ros.Disconnect();
                SceneManager.LoadScene(SceneManager.GetActiveScene().name);
            }
        }
    }

    private Vector3 ProcessGripperPosition(ActionBuffers vectorAction)
    {
        float x_new = (vectorAction.ContinuousActions[0]) * 0.02f;
        float y_new = (vectorAction.ContinuousActions[1]) * 0.02f;
        float z_new = (vectorAction.ContinuousActions[2]) * 0.02f;
        Vector3 gripperPosition = GetToolLinkPosition() + new Vector3(x_new, y_new, z_new);
        Debug.Log("Gripper position: " + gripperPosition);
        return gripperPosition;
    }

    private Vector3 ProcessGripperOrientation(Vector3 gripperPosition)
    {
        Transform shoulderLink = this.transform.Find("world/base_link/shoulder_link");
        Vector3 objectPosition = GetTargetPosition();
        Vector3 relativePosition =  shoulderLink.InverseTransformPoint(gripperPosition) -  shoulderLink.InverseTransformPoint(objectPosition);
        float x_angle = Mathf.Atan2(relativePosition.y, relativePosition.z);
        if (x_angle > 0)
        {
            x_angle = Mathf.PI - x_angle;
        } else
        {
            x_angle = - Mathf.PI - x_angle;
        }
        float z_angle = Mathf.Atan2(relativePosition.y, relativePosition.x);
        if (z_angle > 0)
        {
            z_angle = z_angle - Mathf.PI / 2;
        } else
        {
            z_angle = z_angle + Mathf.PI / 2;
        }
        Vector3 gripperOrientation = new Vector3(x_angle,
                                                 Utils.GetYRotationFromXZ(gripperPosition.x, gripperPosition.z),
                                                 z_angle);

        return gripperOrientation;
    }

    private bool CheckAction(ActionBuffers vectorAction)
    {
        if (vectorAction.ContinuousActions[0] < 0.001f &&
            vectorAction.ContinuousActions[1] < 0.001f &&
            vectorAction.ContinuousActions[2] < 0.001f)
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
        PublishJoints(position, orientation_rad);
        gripperStateCmd = open;
        lastZOrientation = orientation_rad.z;
    }

    private void ConfigJoints()
    {
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

    private Vector3 GetToolLinkPosition()
    {
        return this.transform.Find("world/base_link/shoulder_link/arm_link/elbow_link/forearm_link/wrist_link/hand_link/tool_link").position;
    }

    private Vector3 GetGripperBaseOrientation()
    {
        return Utils.ConvertRotation(this.transform.Find("world/base_link/shoulder_link/arm_link/elbow_link/forearm_link/wrist_link/hand_link/tool_link").rotation.eulerAngles);
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
        gripperControlInAction = true;
        var request = new PointToPointServiceRequest();
        request.joints_input = CurrentJointConfig();
        Vector3 offset = new Vector3(0.000012391f, 0.001206f, -0.0011351f);
        request.finish_pose = new PoseMsg
        {
            position = (destination + offset).To<FLU>(),
            orientation = Quaternion.Euler(orientation_rad.x * Mathf.Rad2Deg, 
                                           orientation_rad.y * Mathf.Rad2Deg, 
                                           orientation_rad.z * Mathf.Rad2Deg).To<FLU>()
        };
        
        Ros.SendServiceMessage<PointToPointServiceResponse>(RosServiceName, request, TrajectoryResponse);
    }

    private void TrajectoryResponse(PointToPointServiceResponse response)
    {
        if (response.trajectory != null)
        {
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            gripperControlInAction = false;
            gripperControlForTimeSteps = 0;
            numContactEntered = 0;
            EndEpisode();
        }
    }

    private IEnumerator ExecuteTrajectories(PointToPointServiceResponse response)
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
        gripperControlForTimeSteps = 0;
        yield return new WaitForSeconds(GripperControlWait);
    }

    private void CloseGripper()
    {
        var leftDrive = LeftGripper.xDrive;
        var rightDrive = RightGripper.xDrive;

        leftDrive.target = -0.01f;
        rightDrive.target = 0.01f;

        LeftGripper.xDrive = leftDrive;
        RightGripper.xDrive = rightDrive;
    }

    private void OpenGripper()
    {
        var leftDrive = LeftGripper.xDrive;
        var rightDrive = RightGripper.xDrive;

        leftDrive.target = 0.01f;
        rightDrive.target = -0.01f;

        LeftGripper.xDrive = leftDrive;
        RightGripper.xDrive = rightDrive;
    }

    internal void ContactEntered(Collider other)
    {
        numContactEntered++;
        if (numContactEntered == 2)
        {
            Debug.Log("======================== Contact entered ========================");
            numContactEntered = 0;
            EndEpisode();
        }
    }
}
