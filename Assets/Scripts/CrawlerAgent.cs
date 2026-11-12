
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class CrawlerAgent : Agent
{
    [Header("Joint Test")]
    public bool testJointMovement = false;

    [Header("Main Body")]
    public Rigidbody crawlerRigidbody;

    [Header("Leg Settings")]
    public ConfigurableJoint[] UpperJoint;
    public ConfigurableJoint[] LowerJoint;

    [Header("Feet Detection")]
    public Collider[] FeetTrigger;
    public int minimumFeetOnGround = 3;

    private bool[] feetOnGround;

    [Header("Target")]
    public Transform target;
    public float targetSpawnRange = 4f;
    public float collectionDistance = 0.7f;

    [Header("Episode Settings")]
    public int maxEpisodeSteps = 10000;
    public float fallHeight = 0.3f;

    [Header("Random Start Height")]
    [Range(0f, 1f)]
    public float raisedStartChance = 0.0f;

    public float minStartHeightOffset = 0.1f;
    public float maxStartHeightOffset = 0.3f;

    [Header("Standing Settings")]
    [Range(0.1f, 1.5f)]
    public float standingHeightMultiplier = 0.8f;

    public float standingTolerance = 0.15f;

    [Range(0f, 1f)]
    public float uprightDotThreshold = 0.75f;

    [Header("Rewards")]
    public float collectionReward = 1.0f;
    public float standingReward = 0.002f;
    public float uprightReward = 0f;
    public float feetReward = 0f;
    public float movementReward = 0f;
    public float fallPenalty = -1f;
    public float inactivityPenalty = -0.0005f;

    public float targetProgressReward = 0.5f;
    public float targetFacingReward = 0.001f;
    private float previousTargetDistance;

    public float minimumMovementSpeed = 0.1f;

    [Header("Debug Overlay")]
    public bool showDebugOverlay = true;

    private float currentUpright;
    private float calculatedStandingHeight;
    private float groundHeight;

    private int episodeCount = 0;
    private int totalSteps = 0;

    private string agentState = "Initializing";
    private string lastEvent = "None";

    private Rigidbody[] allRigidbodies;

    private Vector3[] initialLocalPositions;
    private Quaternion[] initialLocalRotations;

    private Vector3 initialRootPosition;
    private Quaternion initialRootRotation;

    private bool initialPoseSaved = false;

    private float episodeStartHeight;

    private Vector3 startPosition;
    private Quaternion startRotation;

    private Quaternion[] startUpperRotations;
    private Quaternion[] startLowerRotations;

    private Vector3 previousPosition;

    private GUIStyle overlayStyle;
    private GUIStyle titleStyle;

    // =========================================
    // ROBOT COORDINATE SYSTEM
    // =========================================

    // World Y is the gravity-up direction.
    private Vector3 WorldUp => Vector3.up;

    // Local +Z of the body is robot-up.
    private Vector3 BodyUp =>
        crawlerRigidbody.transform.forward;

    // Local -X of the body is robot-forward.
    private Vector3 BodyForward =>
        -crawlerRigidbody.transform.right;

    // =========================================
    // JOINT CONFIGURATION
    // =========================================

    private void ConfigureJoint(ConfigurableJoint joint)
    {
        joint.rotationDriveMode = RotationDriveMode.XYAndZ;

        JointDrive xDrive = joint.angularXDrive;
        xDrive.positionSpring = 400f;
        xDrive.positionDamper = 50f;
        xDrive.maximumForce = 1000f;
        joint.angularXDrive = xDrive;

        JointDrive yzDrive = joint.angularYZDrive;
        yzDrive.positionSpring = 400f;
        yzDrive.positionDamper = 50f;
        yzDrive.maximumForce = 1000f;
        joint.angularYZDrive = yzDrive;
    }

    // =========================================
    // INITIALIZATION
    // =========================================

    public override void Initialize()
    {
        MaxStep = maxEpisodeSteps;

        if (crawlerRigidbody == null)
        {
            Debug.LogError("Crawler Rigidbody is not assigned!");
            return;
        }

        initialRootPosition = crawlerRigidbody.position;
        initialRootRotation = crawlerRigidbody.rotation;

        startPosition = initialRootPosition;
        startRotation = initialRootRotation;

        feetOnGround = new bool[FeetTrigger.Length];

        startUpperRotations = new Quaternion[UpperJoint.Length];
        startLowerRotations = new Quaternion[LowerJoint.Length];

        foreach (ConfigurableJoint joint in UpperJoint)
            ConfigureJoint(joint);

        foreach (ConfigurableJoint joint in LowerJoint)
            ConfigureJoint(joint);

        for (int i = 0; i < UpperJoint.Length; i++)
            startUpperRotations[i] = UpperJoint[i].transform.rotation;

        for (int i = 0; i < LowerJoint.Length; i++)
            startLowerRotations[i] = LowerJoint[i].transform.rotation;

        // Save all robot rigidbodies.
        allRigidbodies = GetComponentsInChildren<Rigidbody>();

        initialLocalPositions = new Vector3[allRigidbodies.Length];
        initialLocalRotations = new Quaternion[allRigidbodies.Length];

        for (int i = 0; i < allRigidbodies.Length; i++)
        {
            initialLocalPositions[i] =
                Quaternion.Inverse(initialRootRotation) *
                (allRigidbodies[i].position - initialRootPosition);

            initialLocalRotations[i] =
                Quaternion.Inverse(initialRootRotation) *
                allRigidbodies[i].rotation;
        }

        CalculateStandingHeight();

        initialPoseSaved = true;

        previousPosition = crawlerRigidbody.position;

        Debug.Log(
            "Crawler initialized. Rigidbody count: " +
            allRigidbodies.Length
        );

        Debug.Log(
            "Body Up: " + BodyUp +
            " | Body Forward: " + BodyForward +
            " | World Up: " + WorldUp
        );
    }

    // =========================================
    // STANDING HEIGHT
    // =========================================

    private void CalculateStandingHeight()
    {
        Collider bodyCollider =
            crawlerRigidbody.GetComponent<Collider>();

        if (bodyCollider == null)
        {
            Debug.LogError("Crawler Rigidbody has no Collider!");
            return;
        }

        // Assumes the robot is initially aligned so local +Z
        // corresponds to world +Y.
        float bodyHeight = bodyCollider.bounds.size.y;

        calculatedStandingHeight =
            bodyHeight * standingHeightMultiplier;

        // Estimate ground height from the lowest foot trigger.
        float lowestFootY = float.PositiveInfinity;

        foreach (Collider foot in FeetTrigger)
        {
            if (foot == null)
                continue;

            if (foot.bounds.min.y < lowestFootY)
                lowestFootY = foot.bounds.min.y;
        }

        if (float.IsPositiveInfinity(lowestFootY))
        {
            groundHeight = bodyCollider.bounds.min.y;
            Debug.LogWarning(
                "Feet colliders not found. Using body collider for ground estimate."
            );
        }
        else
        {
            groundHeight = lowestFootY;
        }

        Debug.Log(
            "Body height: " + bodyHeight +
            " | Standing threshold: " + calculatedStandingHeight +
            " | Ground Y: " + groundHeight
        );
    }

    // =========================================
    // RESET ROBOT
    // =========================================

    private void ResetRobot(Vector3 spawnPosition)
    {
        if (!initialPoseSaved)
            return;

        foreach (Rigidbody rb in allRigidbodies)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        crawlerRigidbody.position = spawnPosition;
        crawlerRigidbody.rotation = initialRootRotation;

        for (int i = 0; i < allRigidbodies.Length; i++)
        {
            Rigidbody rb = allRigidbodies[i];

            if (rb == crawlerRigidbody)
                continue;

            rb.position =
                spawnPosition +
                initialRootRotation * initialLocalPositions[i];

            rb.rotation =
                initialRootRotation * initialLocalRotations[i];

            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        // Restore initial joint targets.
        foreach (ConfigurableJoint joint in UpperJoint)
            joint.targetRotation = Quaternion.identity;

        foreach (ConfigurableJoint joint in LowerJoint)
            joint.targetRotation = Quaternion.identity;

        Physics.SyncTransforms();

        foreach (Rigidbody rb in allRigidbodies)
            rb.WakeUp();
    }

    // =========================================
    // FEET DETECTION
    // =========================================

    public void FootEnter(int footIndex)
    {
        if (footIndex >= 0 && footIndex < feetOnGround.Length)
            feetOnGround[footIndex] = true;
    }

    public void FootExit(int footIndex)
    {
        if (footIndex >= 0 && footIndex < feetOnGround.Length)
            feetOnGround[footIndex] = false;
    }

    private int GetTouchingFeetCount()
    {
        int count = 0;

        for (int i = 0; i < feetOnGround.Length; i++)
        {
            if (feetOnGround[i])
                count++;
        }

        return count;
    }

    // =========================================
    // EPISODE RESET
    // =========================================

    public override void OnEpisodeBegin()
    {
        episodeCount++;

        agentState = "Running";
        lastEvent = "Episode started";

        Vector3 spawnPosition = startPosition;

        // Random vertical offset in world Y.
        if (Random.value < raisedStartChance)
        {
            float heightOffset = Random.Range(
                minStartHeightOffset,
                maxStartHeightOffset
            );

            spawnPosition += WorldUp * heightOffset;
        }

        ResetRobot(spawnPosition);

        previousUprightDot = Vector3.Dot(BodyUp, WorldUp);

        previousBodyHeight =
            crawlerRigidbody.position.y - groundHeight;

        episodeStartHeight = spawnPosition.y;

        if (target != null)
        {
            Vector3 offset = new Vector3(
                Random.Range(-targetSpawnRange, targetSpawnRange),
                0f,
                Random.Range(-targetSpawnRange, targetSpawnRange)
            );

            target.position = new Vector3(
                spawnPosition.x + offset.x,
                0.8f,
                spawnPosition.z + offset.z
            );
        }

        if (target != null)
        {
            Vector3 direction = Vector3.ProjectOnPlane(
                target.position - crawlerRigidbody.position,
                WorldUp
            );

            previousTargetDistance = direction.magnitude;
        }
        else
        {
            previousTargetDistance = 0f;
        }

        for (int i = 0; i < feetOnGround.Length; i++)
            feetOnGround[i] = false;

        previousPosition = crawlerRigidbody.position;

        currentUpright = 0f;
    }

    // =========================================
    // OBSERVATIONS
    // =========================================

    public override void CollectObservations(VectorSensor sensor)
    {
        // Body linear velocity in agent-local coordinates.
        sensor.AddObservation(
            transform.InverseTransformDirection(
                crawlerRigidbody.linearVelocity
            )
        );

        // Body angular velocity in agent-local coordinates.
        sensor.AddObservation(
            transform.InverseTransformDirection(
                crawlerRigidbody.angularVelocity
            )
        );

        // Robot orientation.
        sensor.AddObservation(BodyUp);
        sensor.AddObservation(BodyForward);

        // How upright is the body?
        float uprightAlignment =
            Vector3.Dot(BodyUp, WorldUp);

        sensor.AddObservation(uprightAlignment);

        // Target direction.
        if (target != null)
        {
            Vector3 localTarget =
                crawlerRigidbody.transform.InverseTransformPoint(
                    target.position
                );

            sensor.AddObservation(localTarget);
            sensor.AddObservation(localTarget.magnitude);
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(0f);
        }

        // Joint rotations.
        foreach (ConfigurableJoint joint in UpperJoint)
            sensor.AddObservation(joint.transform.localRotation);

        foreach (ConfigurableJoint joint in LowerJoint)
            sensor.AddObservation(joint.transform.localRotation);

        // Foot contacts.
        for (int i = 0; i < feetOnGround.Length; i++)
        {
            sensor.AddObservation(
                feetOnGround[i] ? 1f : 0f
            );
        }
    }

    // =========================================
    // ACTIONS
    // =========================================

    public override void OnActionReceived(ActionBuffers actions)
    {
        totalSteps++;

        var continuousActions = actions.ContinuousActions;

        // Joint test mode.
        if (testJointMovement)
        {
            if (UpperJoint.Length > 0)
            {
                UpperJoint[0].targetRotation =
                    Quaternion.Euler(0f, 0f, 30f);
            }

            return;
        }

        int index = 0;

        // Upper joints: X and Z.
        foreach (ConfigurableJoint joint in UpperJoint)
        {
            float actionX = continuousActions[index++];
            float actionZ = continuousActions[index++];

            float angleX = MapToRange(
                actionX,
                joint.lowAngularXLimit.limit,
                joint.highAngularXLimit.limit
            );

            float angleZ = MapToRange(
                actionZ,
                -joint.angularZLimit.limit,
                joint.angularZLimit.limit
            );

            joint.targetRotation = Quaternion.Euler(
                angleX,
                0f,
                angleZ
            );
        }

        // Lower joints: X.
        foreach (ConfigurableJoint joint in LowerJoint)
        {
            float actionX = continuousActions[index++];

            float angleX = MapToRange(
                actionX,
                joint.lowAngularXLimit.limit,
                joint.highAngularXLimit.limit
            );

            joint.targetRotation = Quaternion.Euler(
                angleX,
                0f,
                0f
            );
        }

        CalculateReward();
    }

    // =========================================
    // ACTION MAPPING
    // =========================================

    private float MapToRange(float action, float min, float max)
    {
        action = Mathf.Clamp(action, -1f, 1f);

        return Mathf.Lerp(
            min,
            max,
            (action + 1f) * 0.5f
        );
    }

    // =========================================
    // REWARD SYSTEM - STANDING ONLY
    // =========================================



    [Header("Fall Detection")]
    [Range(0f, 1f)]
    public float fallAngleThreshold = -0.5f;


    private float previousUprightDot;
    private float previousBodyHeight;

    private void CalculateReward()
    {
        float uprightDot = Vector3.Dot(BodyUp, WorldUp);

        float bodyHeight =
            crawlerRigidbody.position.y - groundHeight;

        int touchingFeet = GetTouchingFeetCount();

        // Zbyt du¿e przewrócenie.
        if (uprightDot < fallAngleThreshold)
        {
            AddReward(fallPenalty);

            agentState = "FALLEN";
            lastEvent = "Excessive tilt";

            EndEpisode();
            return;
        }

        // =========================================
        // TARGET NAVIGATION REWARDS
        // =========================================

        if (target != null)
        {
            // Kierunek do celu na p³aszczyŸnie poziomej.
            Vector3 directionToTarget = Vector3.ProjectOnPlane(
                target.position - crawlerRigidbody.position,
                WorldUp
            );

            float currentTargetDistance = directionToTarget.magnitude;

            if (currentTargetDistance > 0.001f)
            {
                directionToTarget.Normalize();

                // 1. Nagroda za zmniejszanie odleg³oœci od celu.
                float distanceProgress =
                    previousTargetDistance - currentTargetDistance;

                if (distanceProgress > 0f)
                {
                    AddReward(distanceProgress * targetProgressReward);
                }

                // 2. Nagroda za skierowanie przodu robota w stronê celu.
                float facingDot = Vector3.Dot(
                    BodyForward.normalized,
                    directionToTarget
                );

                float facingReward = Mathf.Max(0f, facingDot);

                AddReward(facingReward * targetFacingReward);
            }

            previousTargetDistance = currentTargetDistance;
        }

        // Nagroda za poprawê orientacji.
        float uprightProgress =
            uprightDot - previousUprightDot;

        AddReward(uprightProgress * 0.05f);

        // Nagroda za podnoszenie korpusu.
        float heightProgress =
            bodyHeight - previousBodyHeight;

        AddReward(heightProgress * 0.1f);

        // Niewielka nagroda za kontakt stóp z pod³o¿em.
        AddReward(touchingFeet * 0.0005f);

        bool upright = uprightDot >= uprightDotThreshold;

        bool bodyRaised =
            bodyHeight >=
            calculatedStandingHeight - standingTolerance;

        bool feetStable =
            touchingFeet >= minimumFeetOnGround;

        if (upright && bodyRaised && feetStable)
        {
            AddReward(standingReward);
            agentState = "Standing";
        }
        else
        {
            AddReward(inactivityPenalty);
            agentState = "Trying to stand";
        }

        previousUprightDot = uprightDot;
        previousBodyHeight = bodyHeight;
        currentUpright = bodyHeight;
    }


    // =========================================
    // TARGET COLLECTION
    // =========================================

    public void CollectTarget()
    {
        AddReward(collectionReward);

        agentState = "Target collected";

        lastEvent = "Target collected! Reward +" +
                    collectionReward.ToString("F2");

        // Losowanie nowej pozycji punktu bez koñczenia epizodu.
        if (target != null)
        {
            Vector3 offset = new Vector3(
                Random.Range(-targetSpawnRange, targetSpawnRange),
                0f,
                Random.Range(-targetSpawnRange, targetSpawnRange)
            );

            target.position = new Vector3(
                crawlerRigidbody.position.x + offset.x,
                0.8f,
                crawlerRigidbody.position.z + offset.z
            );
        }
    }

    // =========================================
    // DEBUG OVERLAY
    // =========================================
    private Vector2 debugScrollPosition;
    private void OnGUI()
    {
        if (!showDebugOverlay || crawlerRigidbody == null)
            return;

        // Inicjalizacja stylów GUI.
        if (overlayStyle == null)
        {
            overlayStyle = new GUIStyle(GUI.skin.label);
            overlayStyle.fontSize = 13;
            overlayStyle.normal.textColor = Color.white;
            overlayStyle.wordWrap = true;

            titleStyle = new GUIStyle(GUI.skin.label);
            titleStyle.fontSize = 17;
            titleStyle.fontStyle = FontStyle.Bold;
            titleStyle.normal.textColor = Color.cyan;
        }

        // Odleg³oœæ od celu.
        float distance = target != null
            ? Vector3.Distance(crawlerRigidbody.position, target.position)
            : -1f;

        // K¹t przechylenia wzglêdem pionu.
        float uprightDot = Vector3.Dot(BodyUp, WorldUp);
        float tiltAngle = Vector3.Angle(BodyUp, WorldUp);

        // Próg upadku.
        float fallAngle = Mathf.Acos(
            Mathf.Clamp(fallAngleThreshold, -1f, 1f)
        ) * Mathf.Rad2Deg;

        int touchingFeet = GetTouchingFeetCount();

        float reward = GetCumulativeReward();

        // Kolor zale¿ny od przechylenia.
        Color tiltColor;

        if (uprightDot <= 0f)
            tiltColor = Color.red;
        else if (tiltAngle > 60f)
            tiltColor = new Color(1f, 0.5f, 0f);
        else
            tiltColor = Color.green;

        // Dopasowanie panelu do rozmiaru ekranu.
        float panelWidth = Mathf.Min(420f, Screen.width - 30f);
        float panelHeight = Screen.height - 30f;

        GUILayout.BeginArea(
            new Rect(15, 15, panelWidth, panelHeight)
        );

        GUI.Box(
            new Rect(0, 0, panelWidth, panelHeight),
            ""
        );

        // Przewijanie zawartoœci panelu.
        debugScrollPosition = GUILayout.BeginScrollView(
            debugScrollPosition,
            GUILayout.Width(panelWidth),
            GUILayout.Height(panelHeight)
        );

        GUILayout.BeginVertical();

        GUILayout.Space(5);

        GUILayout.Label("CRAWLER AGENT", titleStyle);

        GUILayout.Space(8);

        GUILayout.Label("Episode: " + episodeCount, overlayStyle);

        GUILayout.Label(
            "Step: " + StepCount + " / " + MaxStep,
            overlayStyle
        );

        GUILayout.Label("State: " + agentState, overlayStyle);

        GUILayout.Space(5);

        GUILayout.Label("=== BODY TILT ===", titleStyle);

        GUI.color = tiltColor;

        GUILayout.Label(
            "Tilt angle: " + tiltAngle.ToString("F1") + " degrees",
            overlayStyle
        );

        GUILayout.Label(
            "Upright Dot: " + uprightDot.ToString("F3"),
            overlayStyle
        );

        GUILayout.Label(
            "Fall threshold: " + fallAngle.ToString("F1") + " degrees",
            overlayStyle
        );

        GUILayout.Label(
            uprightDot <= 0f
                ? "DANGER: EPISODE WILL END"
                : "Tilt status: Recoverable",
            overlayStyle
        );

        GUI.color = Color.white;

        // Pasek przechylenia 0-180 stopni.
        Rect barRect = GUILayoutUtility.GetRect(300, 22);

        GUI.Box(barRect, "");

        float normalizedTilt = Mathf.Clamp01(tiltAngle / 180f);

        Rect fillRect = new Rect(
            barRect.x + 2,
            barRect.y + 2,
            (barRect.width - 4) * normalizedTilt,
            barRect.height - 4
        );

        GUI.color = tiltColor;
        GUI.DrawTexture(fillRect, Texture2D.whiteTexture);
        GUI.color = Color.white;

        GUILayout.Space(8);

        GUILayout.Label("=== STANDING ===", titleStyle);

        GUILayout.Label(
            "Body height: " + currentUpright.ToString("F2"),
            overlayStyle
        );

        GUILayout.Label(
            "Standing threshold: " +
            calculatedStandingHeight.ToString("F2"),
            overlayStyle
        );

        GUILayout.Label(
            "Feet touching ground: " +
            touchingFeet + " / " + feetOnGround.Length,
            overlayStyle
        );

        GUILayout.Space(8);

        GUILayout.Label("=== TRAINING ===", titleStyle);

        GUILayout.Label(
            "Distance to target: " +
            (distance >= 0 ? distance.ToString("F2") : "No target"),
            overlayStyle
        );

        GUILayout.Label(
            "Episode reward: " + reward.ToString("F3"),
            overlayStyle
        );

        GUILayout.Label("Last event: " + lastEvent, overlayStyle);

        GUILayout.EndVertical();

        GUILayout.EndScrollView();

        GUILayout.EndArea();
    }
}