using UnityEngine;

public class JointTest : MonoBehaviour
{
    public ConfigurableJoint joint;

    void Start()
    {
        joint.rotationDriveMode = RotationDriveMode.XYAndZ;

        JointDrive xDrive = joint.angularXDrive;
        xDrive.positionSpring = 500f;
        xDrive.positionDamper = 50f;
        xDrive.maximumForce = 1000f;
        joint.angularXDrive = xDrive;

        JointDrive yzDrive = joint.angularYZDrive;
        yzDrive.positionSpring = 500f;
        yzDrive.positionDamper = 50f;
        yzDrive.maximumForce = 1000f;
        joint.angularYZDrive = yzDrive;

        joint.targetRotation = Quaternion.Euler(30f, 0f, 0f);
    }
}