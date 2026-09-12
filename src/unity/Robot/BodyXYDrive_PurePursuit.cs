using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class BodyXYDrive_PurePursuit : MonoBehaviour
{
    [Header("Reference")]
    public Transform Body;
    public FixedRouteReader routeReader;
    public UnityRouteExecutor routeExecutor;

    [Header("Route")]
    public bool autoStartOnPlay = false;
    public bool loopRoute = false;

    [Header("Axis Settings")]
    public bool invertTurnCommand = true;
    public bool allowReverse = false;

    [Header("Lookahead Target (PP geometry only)")]
    public float lookaheadDistance = 1.2f;
    public float waypointReachThreshold = 0.5f;
    public float slowDownDistance = 1.2f;

    [Header("Turn-Then-Go Control")]
    public float turnOnlyAngleDeg = 10f;
    public float goAngleDeg = 4f;
    public float maxForwardCommand = 0.25f;
    public float maxTurnCommand = 0.35f;
    public float turnGain = 0.02f;
    public float forwardGain = 0.10f;

    [Header("Tuning - Basic Speed")]
    public float moveSpeed = 1.0f;
    public float turnSpeed = 45f;
    public float maxHorizontalSpeed = 1.2f;

    [Header("Physics Tuning")]
    public float inputDeadZone = 0.01f;
    public float moveForceMultiplier = 180f;
    public float turnTorqueMultiplier = 25f;
    public float linearDamping = 0.18f;
    public float angularDamping = 0.2f;

    [Header("Pitch Settings")]
    public float maxPitchAngle = 50f;
    public bool freezeRoll = true;

    [Header("Stability")]
    public bool autoSetDamping = true;
    public float maxAngularVelocity = 20f;

    [Header("Unstick Override")]
    public bool enableTurnOnlyUnstick = true;
    public float turnOnlyStuckTime = 3f;
    public float forcedForwardCmd = 0.12f;
    public float forcedForwardDuration = 0.35f;

    [Header("Critical Waypoint Lock")]
    public bool enableCriticalWaypointLock = true;
    public float criticalWaypointReachThreshold = 0.3f;
    public float criticalWaypointActivationDistance = 2.0f;
    public float criticalWaypointMatchTolerance = 0.2f;
    public List<Vector3> criticalWaypointCenters = new List<Vector3>()
    {
        new Vector3(12.5f, 0.6f, 17.5f),
        new Vector3(12.5f, 4.0f, 17.5f),
    };

    [Header("Final Stop Debug")]
    public bool debugFinalStop = true;
    public float debugFinalStopInterval = 0.2f;

    [Header("Runtime Debug")]
    public bool isRunning = false;
    public bool finished = false;
    public int nearestIndex = 0;
    public int targetSegmentIndex = 0;
    public Vector3 currentTargetPoint;
    public float currentForwardCmd = 0f;
    public float currentTurnCmd = 0f;
    public List<Vector3> activeWaypoints = new List<Vector3>();

    private Rigidbody rb;
    private float maxPitchRad;
    public bool hardStopped;
    private float turnOnlyTimer = 0f;
    private float forcedForwardTimer = 0f;
    private float debugFinalStopTimer = 0f;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (Body == null) Body = transform;

        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        rb.maxAngularVelocity = maxAngularVelocity;

        rb.constraints = RigidbodyConstraints.None;
        if (freezeRoll)
            rb.constraints = RigidbodyConstraints.FreezeRotationZ;

        if (autoSetDamping)
        {
            rb.linearDamping = linearDamping;
            rb.angularDamping = angularDamping;
        }

        maxPitchRad = maxPitchAngle * Mathf.Deg2Rad;
    }

    void Start()
    {
        if (autoStartOnPlay && routeReader != null)
            StartFollowing();
    }

    [ContextMenu("Start Following Fixed Route")]
    public void StartFollowing()
    {
        if (routeReader == null)
        {
            Debug.LogWarning("[Follower] routeReader is not assigned.");
            return;
        }

        if (routeReader.combinedWaypoints == null || routeReader.combinedWaypoints.Count < 2)
        {
            Debug.LogWarning("[Follower] combinedWaypoints is empty. Run Build Combined Route first.");
            return;
        }

        activeWaypoints = new List<Vector3>(routeReader.combinedWaypoints);
        nearestIndex = FindNearestWaypointIndex(GetCurrentPosition());
        targetSegmentIndex = nearestIndex;
        currentTargetPoint = activeWaypoints[Mathf.Min(nearestIndex, activeWaypoints.Count - 1)];

        currentForwardCmd = 0f;
        currentTurnCmd = 0f;
        isRunning = true;
        finished = false;
        hardStopped = false;

        Debug.Log($"[Follower] Start following fixed route. waypointCount={activeWaypoints.Count}, nearestIndex={nearestIndex}");
    }

    [ContextMenu("Run CSV Route From Executor")]
    public void RunCsvRouteFromExecutor()
    {
        if (routeExecutor == null)
        {
            Debug.LogWarning("[Follower] routeExecutor is not assigned.");
            return;
        }

        routeExecutor.LoadAndRun();
    }

    public void SetRoute(List<Vector3> route, bool autoStart = true)
    {
        if (route == null || route.Count < 2)
        {
            Debug.LogWarning("[Follower] SetRoute received empty or too-short route.");
            return;
        }

        activeWaypoints = new List<Vector3>(route);

        nearestIndex = FindNearestWaypointIndex(GetCurrentPosition());
        targetSegmentIndex = nearestIndex;
        currentTargetPoint = activeWaypoints[Mathf.Min(nearestIndex, activeWaypoints.Count - 1)];

        currentForwardCmd = 0f;
        currentTurnCmd = 0f;

        finished = false;
        hardStopped = false;
        isRunning = autoStart;

        Debug.Log($"[Follower] SetRoute loaded {activeWaypoints.Count} waypoints. autoStart={autoStart}, nearestIndex={nearestIndex}");
    }

    [ContextMenu("Stop Following")]
    public void StopFollowing()
    {
        isRunning = false;
        finished = true;
        hardStopped = true;
        currentForwardCmd = 0f;
        currentTurnCmd = 0f;
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        Debug.Log("[Follower] Hard stopped.");
    }

    void FixedUpdate()
    {
        if (autoSetDamping)
        {
            rb.linearDamping = linearDamping;
            rb.angularDamping = angularDamping;
        }

        if (hardStopped)
        {
            currentForwardCmd = 0f;
            currentTurnCmd = 0f;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            return;
        }

        if (!isRunning || finished || activeWaypoints == null || activeWaypoints.Count < 2)
        {
            LimitPitchAngle();
            return;
        }

        Vector3 currentPos = GetCurrentPosition();

        nearestIndex = FindNearestWaypointIndex(currentPos);

        Vector3 goalPoint = activeWaypoints[activeWaypoints.Count - 1];
        int lastIdx = activeWaypoints.Count - 1;

        float distToGoal3D = Vector3.Distance(currentPos, goalPoint);
        float distToGoalXZ = PlanarDistanceXZ(currentPos, goalPoint);

        float distToPrev3D = float.PositiveInfinity;
        float distToPrevXZ = float.PositiveInfinity;
        if (lastIdx >= 1)
        {
            distToPrev3D = Vector3.Distance(currentPos, activeWaypoints[lastIdx - 1]);
            distToPrevXZ = PlanarDistanceXZ(currentPos, activeWaypoints[lastIdx - 1]);
        }

        float finalReachThreshold = waypointReachThreshold;

        bool closeToFinalWaypoint3D = distToGoal3D <= finalReachThreshold;
        bool closeToFinalWaypointXZ = distToGoalXZ <= finalReachThreshold;

        bool muchCloserToFinalThanPrev3D = lastIdx >= 1 && distToGoal3D * 8f <= distToPrev3D;
        bool muchCloserToFinalThanPrevXZ = lastIdx >= 1 && distToGoalXZ * 8f <= distToPrevXZ;

        DebugFinalStopState(
            currentPos,
            goalPoint,
            currentTargetPoint,
            nearestIndex,
            lastIdx,
            distToGoal3D,
            distToGoalXZ,
            distToPrev3D,
            distToPrevXZ,
            closeToFinalWaypoint3D,
            closeToFinalWaypointXZ,
            muchCloserToFinalThanPrev3D,
            muchCloserToFinalThanPrevXZ,
            finalReachThreshold
        );

        if (closeToFinalWaypointXZ || muchCloserToFinalThanPrevXZ)
        {
            Debug.Log(
                $"[FINAL STOP TRIGGERED] " +
                $"bodyPos={currentPos}, transformPos={transform.position}, goalPoint={goalPoint}, targetPoint={currentTargetPoint}, " +
                $"nearestIndex={nearestIndex}, lastIdx={lastIdx}, " +
                $"distToGoal3D={distToGoal3D:F3}, distToGoalXZ={distToGoalXZ:F3}, " +
                $"distToPrev3D={distToPrev3D:F3}, distToPrevXZ={distToPrevXZ:F3}, " +
                $"finalReachThreshold={finalReachThreshold:F3}, " +
                $"close3D={closeToFinalWaypoint3D}, closeXZ={closeToFinalWaypointXZ}, " +
                $"muchCloser3D={muchCloserToFinalThanPrev3D}, muchCloserXZ={muchCloserToFinalThanPrevXZ}"
            );

            finished = true;
            isRunning = false;
            hardStopped = true;
            currentTargetPoint = goalPoint;
            currentForwardCmd = 0f;
            currentTurnCmd = 0f;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            Debug.Log($"[Follower] Final waypoint reached. nearestIndex={nearestIndex}, lastIdx={lastIdx}, distToGoal3D={distToGoal3D:F3}, distToGoalXZ={distToGoalXZ:F3}. Hard stop engaged.");
            return;
        }

        UpdateLookaheadTarget();
        ComputeTurnThenGoCommands(currentTargetPoint, distToGoal3D);
        ApplyTurnOnlyUnstick();

        ApplyDriveLikeMinimal(currentForwardCmd, currentTurnCmd);
        LimitPitchAngle();
    }

    private Vector3 GetCurrentPosition()
    {
        return (Body != null) ? Body.position : transform.position;
    }

    private void DebugFinalStopState(
        Vector3 currentPos,
        Vector3 goalPoint,
        Vector3 targetPoint,
        int nearestIdx,
        int lastIdx,
        float distToGoal3D,
        float distToGoalXZ,
        float distToPrev3D,
        float distToPrevXZ,
        bool close3D,
        bool closeXZ,
        bool muchCloser3D,
        bool muchCloserXZ,
        float finalReachThreshold)
    {
        if (!debugFinalStop)
            return;

        debugFinalStopTimer -= Time.fixedDeltaTime;
        if (debugFinalStopTimer > 0f)
            return;

        debugFinalStopTimer = debugFinalStopInterval;

        float distToTarget3D = Vector3.Distance(currentPos, targetPoint);
        float distToTargetXZ = PlanarDistanceXZ(currentPos, targetPoint);
        float bodyVsTransform = Vector3.Distance(GetCurrentPosition(), transform.position);

        Debug.Log(
            "[FINAL STOP DEBUG] " +
            $"transformPos={transform.position} | " +
            $"bodyPos={currentPos} | " +
            $"goalPoint={goalPoint} | " +
            $"targetPoint={targetPoint} | " +
            $"nearestIndex={nearestIdx} | " +
            $"lastIdx={lastIdx} | " +
            $"distToGoal3D={distToGoal3D:F3} | " +
            $"distToGoalXZ={distToGoalXZ:F3} | " +
            $"distToPrev3D={distToPrev3D:F3} | " +
            $"distToPrevXZ={distToPrevXZ:F3} | " +
            $"distToTarget3D={distToTarget3D:F3} | " +
            $"distToTargetXZ={distToTargetXZ:F3} | " +
            $"bodyVsTransform={bodyVsTransform:F3} | " +
            $"finalReachThreshold={finalReachThreshold:F3} | " +
            $"close3D={close3D} | " +
            $"closeXZ={closeXZ} | " +
            $"muchCloser3D={muchCloser3D} | " +
            $"muchCloserXZ={muchCloserXZ}"
        );
    }

    private void UpdateLookaheadTarget()
    {
        Vector3 pos = GetCurrentPosition();

        int criticalIdx = FindLockedCriticalWaypointIndex(pos, nearestIndex);
        if (criticalIdx >= 0)
        {
            currentTargetPoint = activeWaypoints[criticalIdx];
            targetSegmentIndex = criticalIdx;
            return;
        }

        Vector3 bestPoint;
        int bestSeg;

        bool found = FindLookaheadIntersection(pos, lookaheadDistance, nearestIndex, out bestPoint, out bestSeg);

        if (found)
        {
            currentTargetPoint = bestPoint;
            targetSegmentIndex = bestSeg;
            return;
        }

        int fallbackIdx = FindFirstWaypointAhead(nearestIndex);
        if (fallbackIdx < 0)
            fallbackIdx = Mathf.Min(nearestIndex + 1, activeWaypoints.Count - 1);

        currentTargetPoint = activeWaypoints[fallbackIdx];
        targetSegmentIndex = fallbackIdx;
    }

    private bool FindLookaheadIntersection(Vector3 center, float radius, int startIdx, out Vector3 hitPoint, out int hitSegIdx)
    {
        hitPoint = Vector3.zero;
        hitSegIdx = -1;

        if (activeWaypoints == null || activeWaypoints.Count < 2)
            return false;

        for (int i = Mathf.Clamp(startIdx, 0, activeWaypoints.Count - 2); i < activeWaypoints.Count - 1; i++)
        {
            Vector3 p1 = activeWaypoints[i];
            Vector3 p2 = activeWaypoints[i + 1];

            Vector2 a = new Vector2(p1.x, p1.z);
            Vector2 b = new Vector2(p2.x, p2.z);
            Vector2 c = new Vector2(center.x, center.z);

            Vector2 d = b - a;
            Vector2 f = a - c;

            float A = Vector2.Dot(d, d);
            float B = 2f * Vector2.Dot(f, d);
            float C = Vector2.Dot(f, f) - radius * radius;

            float discriminant = B * B - 4f * A * C;
            if (discriminant < 0f)
                continue;

            discriminant = Mathf.Sqrt(discriminant);

            float t1 = (-B - discriminant) / (2f * A);
            float t2 = (-B + discriminant) / (2f * A);

            bool foundOnThisSeg = false;
            float chosenT = -1f;

            if (t1 >= 0f && t1 <= 1f)
            {
                chosenT = t1;
                foundOnThisSeg = true;
            }
            if (t2 >= 0f && t2 <= 1f)
            {
                if (!foundOnThisSeg || t2 > chosenT)
                {
                    chosenT = t2;
                    foundOnThisSeg = true;
                }
            }

            if (foundOnThisSeg)
            {
                Vector3 p = Vector3.Lerp(p1, p2, chosenT);

                Vector3 local = Body.InverseTransformPoint(p);
                float forwardDist = local.x;
                if (forwardDist <= 0f)
                    continue;

                hitPoint = p;
                hitSegIdx = i;
                return true;
            }
        }

        return false;
    }

    private int FindFirstWaypointAhead(int startIdx)
    {
        for (int i = Mathf.Clamp(startIdx, 0, activeWaypoints.Count - 1); i < activeWaypoints.Count; i++)
        {
            Vector3 local = Body.InverseTransformPoint(activeWaypoints[i]);
            if (local.x > 0f)
                return i;
        }
        return -1;
    }

    private int FindLockedCriticalWaypointIndex(Vector3 pos, int startIdx)
    {
        if (!enableCriticalWaypointLock || activeWaypoints == null || activeWaypoints.Count == 0 || criticalWaypointCenters == null || criticalWaypointCenters.Count == 0)
            return -1;

        for (int i = Mathf.Clamp(startIdx, 0, activeWaypoints.Count - 1); i < activeWaypoints.Count; i++)
        {
            if (!IsCriticalWaypoint(activeWaypoints[i]))
                continue;

            if (!ShouldLockCriticalWaypoint(i))
                continue;

            float planarDist = PlanarDistanceXZ(pos, activeWaypoints[i]);

            if (planarDist <= criticalWaypointReachThreshold)
                return -1;

            if (planarDist <= criticalWaypointActivationDistance)
                return i;

            return -1;
        }

        return -1;
    }

    private bool ShouldLockCriticalWaypoint(int waypointIndex)
    {
        if (!enableCriticalWaypointLock)
            return false;

        if (activeWaypoints == null)
            return false;

        if (waypointIndex < 0 || waypointIndex >= activeWaypoints.Count - 1)
            return false;

        Vector3 curr = activeWaypoints[waypointIndex];
        Vector3 next = activeWaypoints[waypointIndex + 1];

        float planarDist = PlanarDistanceXZ(curr, next);
        float dy = Mathf.Abs(next.y - curr.y);

        return planarDist <= criticalWaypointMatchTolerance && dy > 0.5f;
    }

    private bool IsCriticalWaypoint(Vector3 p)
    {
        for (int i = 0; i < criticalWaypointCenters.Count; i++)
        {
            if (PlanarDistanceXZ(p, criticalWaypointCenters[i]) <= criticalWaypointMatchTolerance)
                return true;
        }
        return false;
    }

    private float PlanarDistanceXZ(Vector3 a, Vector3 b)
    {
        float dx = a.x - b.x;
        float dz = a.z - b.z;
        return Mathf.Sqrt(dx * dx + dz * dz);
    }

    private void ComputeTurnThenGoCommands(Vector3 targetPoint, float distToGoal)
    {
        Vector3 localTarget = Body.InverseTransformPoint(targetPoint);

        float forwardDist = localTarget.x;
        float lateralError = localTarget.z;
        float angleDeg = Mathf.Atan2(lateralError, forwardDist) * Mathf.Rad2Deg;
        float absAngle = Mathf.Abs(angleDeg);

        float vCmd = 0f;
        float hCmd = 0f;

        if (!allowReverse && forwardDist < 0f)
        {
            vCmd = 0f;
            hCmd = Mathf.Clamp(angleDeg * turnGain, -maxTurnCommand, maxTurnCommand);
        }
        else if (absAngle > turnOnlyAngleDeg)
        {
            vCmd = 0f;
            hCmd = Mathf.Clamp(angleDeg * turnGain, -maxTurnCommand, maxTurnCommand);
        }
        else
        {
            hCmd = Mathf.Clamp(angleDeg * turnGain, -maxTurnCommand, maxTurnCommand);

            float speedScale = Mathf.Clamp01(distToGoal * forwardGain);
            vCmd = Mathf.Clamp(speedScale, 0f, maxForwardCommand);

            if (distToGoal < slowDownDistance)
            {
                float k = Mathf.Clamp01(distToGoal / slowDownDistance);
                vCmd *= Mathf.Lerp(0.25f, 1f, k);
            }

            vCmd = Mathf.Max(vCmd, 0.10f);

            if (absAngle > goAngleDeg)
                vCmd *= 0.6f;
        }

        if (invertTurnCommand)
            hCmd = -hCmd;

        if (Mathf.Abs(vCmd) < inputDeadZone) vCmd = 0f;
        if (Mathf.Abs(hCmd) < inputDeadZone) hCmd = 0f;

        currentForwardCmd = vCmd;
        currentTurnCmd = hCmd;

        Debug.Log(
            $"[FollowerDebug] nearestIndex={nearestIndex}, seg={targetSegmentIndex}, " +
            $"forwardDist={forwardDist:F3}, lateralError={lateralError:F3}, angleDeg={angleDeg:F2}, " +
            $"vCmd={vCmd:F3}, hCmd={hCmd:F3}, target={currentTargetPoint}"
        );
    }

    private void ApplyTurnOnlyUnstick()
    {
        if (!enableTurnOnlyUnstick)
            return;

        bool turnOnlyState = Mathf.Abs(currentTurnCmd) > inputDeadZone && Mathf.Abs(currentForwardCmd) <= inputDeadZone;

        if (forcedForwardTimer > 0f)
        {
            forcedForwardTimer -= Time.fixedDeltaTime;
            currentForwardCmd = Mathf.Max(currentForwardCmd, forcedForwardCmd);
            return;
        }

        if (turnOnlyState)
        {
            turnOnlyTimer += Time.fixedDeltaTime;
            if (turnOnlyTimer >= turnOnlyStuckTime)
            {
                forcedForwardTimer = forcedForwardDuration;
                turnOnlyTimer = 0f;
                currentForwardCmd = Mathf.Max(currentForwardCmd, forcedForwardCmd);
                Debug.Log($"[FollowerDebug] Turn-only stuck for {turnOnlyStuckTime:F1}s, forcing forward cmd={currentForwardCmd:F3} for {forcedForwardDuration:F2}s");
            }
        }
        else
        {
            turnOnlyTimer = 0f;
        }
    }

    private void ApplyDriveLikeMinimal(float v, float h)
    {
        v = Mathf.Abs(v) < inputDeadZone ? 0f : v;
        h = Mathf.Abs(h) < inputDeadZone ? 0f : h;

        if (v != 0f)
        {
            Vector3 stepForward = Vector3.ProjectOnPlane(Body.right, transform.up).normalized;
            Vector3 moveForce = stepForward * v * moveSpeed * moveForceMultiplier;
            rb.AddForce(moveForce, ForceMode.Force);

            Vector3 horizontalVel = new Vector3(rb.linearVelocity.x, 0f, rb.linearVelocity.z);
            if (horizontalVel.magnitude > maxHorizontalSpeed)
            {
                rb.linearVelocity = new Vector3(
                    horizontalVel.normalized.x * maxHorizontalSpeed,
                    rb.linearVelocity.y,
                    horizontalVel.normalized.z * maxHorizontalSpeed
                );
            }
        }

        if (h != 0f)
        {
            Vector3 turnAxis = Vector3.up;
            float turnTorque = h * turnSpeed * Mathf.Deg2Rad * turnTorqueMultiplier;
            rb.AddTorque(turnAxis * turnTorque, ForceMode.Force);
        }
    }

    private int FindNearestWaypointIndex(Vector3 pos)
    {
        int bestIdx = 0;
        float bestDist = float.PositiveInfinity;

        for (int i = 0; i < activeWaypoints.Count; i++)
        {
            float d = Vector3.Distance(pos, activeWaypoints[i]);
            if (d < bestDist)
            {
                bestDist = d;
                bestIdx = i;
            }
        }

        return bestIdx;
    }

    private void LimitPitchAngle()
    {
        Vector3 currentEuler = Body.rotation.eulerAngles;
        float currentPitch = Mathf.DeltaAngle(0, currentEuler.x);
        float currentPitchRad = currentPitch * Mathf.Deg2Rad;

        if (Mathf.Abs(currentPitchRad) > maxPitchRad)
        {
            float targetPitch = maxPitchRad * Mathf.Sign(currentPitchRad) * Mathf.Rad2Deg;
            Quaternion targetRot = Quaternion.Euler(targetPitch, currentEuler.y, currentEuler.z);
            Body.rotation = Quaternion.Lerp(Body.rotation, targetRot, Time.fixedDeltaTime * 1f);
        }
    }

    private void OnDrawGizmos()
    {
        if (activeWaypoints != null && activeWaypoints.Count > 0)
        {
            Gizmos.color = Color.cyan;
            for (int i = 0; i < activeWaypoints.Count; i++)
            {
                Gizmos.DrawSphere(activeWaypoints[i], 0.10f);
                if (i < activeWaypoints.Count - 1)
                    Gizmos.DrawLine(activeWaypoints[i], activeWaypoints[i + 1]);
            }
        }

        if (currentTargetPoint != Vector3.zero)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(currentTargetPoint, 0.18f);
            if (Body != null)
                Gizmos.DrawLine(Body.position, currentTargetPoint);
        }
    }
}