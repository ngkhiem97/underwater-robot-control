using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OpenGrippers : MonoBehaviour
{
    private ArticulationBody[] articulationChain;

    public Unity.Robotics.UrdfImporter.Control.ControlType control = Unity.Robotics.UrdfImporter.Control.ControlType.PositionControl;
    public float stiffness;
    public float damping;
    public float forceLimit;


    // Start is called before the first frame update
    void Start()
    {
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        ArticulationBody rightGripper = articulationChain[11];
        ArticulationBody leftGripper = articulationChain[13];

        int defDyanmicVal = 10;
        rightGripper.gameObject.AddComponent<JointControl>();
        rightGripper.jointFriction = defDyanmicVal;
        rightGripper.angularDamping = defDyanmicVal;
        ArticulationDrive currentDrive = rightGripper.xDrive;
        currentDrive.forceLimit = forceLimit;
        rightGripper.xDrive = currentDrive;

        leftGripper.gameObject.AddComponent<JointControl>();
        leftGripper.jointFriction = defDyanmicVal;
        leftGripper.angularDamping = defDyanmicVal;
        currentDrive = leftGripper.xDrive;
        currentDrive.forceLimit = forceLimit;
        leftGripper.xDrive = currentDrive;

    }

    // Update is called once per frame
    void Update()
    {
        ArticulationBody rightGripper = articulationChain[11];
        ArticulationBody leftGripper = articulationChain[13];

        JointControl leftGripperControl = leftGripper.gameObject.GetComponent<JointControl>();
        JointControl rightGripperControl = rightGripper.gameObject.GetComponent<JointControl>();

        if (leftGripperControl.controltype != (Unity.Robotics.UrdfImporter.Control.ControlType)control)
        {
            UpdateControlType(leftGripperControl);
        }

        if (rightGripperControl.controltype != (Unity.Robotics.UrdfImporter.Control.ControlType)control)
        {
            UpdateControlType(rightGripperControl);
        }
        
        leftGripperControl.direction = Unity.Robotics.UrdfImporter.Control.RotationDirection.Positive;
        rightGripperControl.direction = Unity.Robotics.UrdfImporter.Control.RotationDirection.Negative;
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
}
