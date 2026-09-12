using UnityEngine;
using System.Collections;

public class ElevatorController : MonoBehaviour
{
    [Header("Plane")]
    public Transform Platform;

    [Header("Motion Settings")]
    public float HighPosY = 4f;
    public float LowPosY = 0.6f;
    public float MoveSpeed = 0.2f;
    public bool UseLocalSpace = true;
    public float TriggerDelay = 3f;

    private bool isAtLow = true;
    private bool isMoving = false;

    // After first use, require "exit -> re-enter" to trigger again
    private bool isTriggerUnlocked = false;

    // NEW: allow first enter to trigger immediately
    private bool hasEverEntered = false;

    private Vector3 targetPos;
    private Vector3 initPlatformPos;

    void Start()
    {
        if (Platform == null)
        {
            Debug.LogError("Please assign the Platform (Plane) to ElevatorController!");
            return;
        }

        initPlatformPos = UseLocalSpace ? Platform.localPosition : Platform.position;
        SetElevatorY(LowPosY);

        Debug.Log($"Elevator initialized successfully, current at floor 0 (Y={LowPosY}m). " +
                  $"First enter will trigger immediately; afterwards, leave the zone and re-enter to trigger again.");
    }

    void OnEnable()
    {
        ElevatorTriggerDetector.OnElevatorEntered += OnRobotEnterZone;
        ElevatorTriggerDetector.OnElevatorExited += OnRobotLeaveZone;
    }

    private void OnRobotEnterZone()
    {
        if (isMoving)
        {
            Debug.LogWarning("elevator is moving... cannot trigger now.");
            return;
        }

        // ✅ First time: trigger immediately (no need to exit/re-enter)
        if (!hasEverEntered)
        {
            hasEverEntered = true;
            Debug.Log("first enter detected -> triggered immediately (no need to exit/re-enter).");
            StartCoroutine(StartElevatorAfterDelay());
            return;
        }

        // ✅ After first time: keep your original lock/unlock behavior
        if (isTriggerUnlocked)
        {
            Debug.Log("triggered... elevator will start after delay.");
            StartCoroutine(StartElevatorAfterDelay());
        }
        else
        {
            Debug.LogWarning("trigger is locked... please leave the zone and re-enter to trigger.");
        }
    }

    private void OnRobotLeaveZone()
    {
        if (!isMoving)
        {
            isTriggerUnlocked = true;
            string currentFloor = isAtLow ? "floor 0" : "floor 1";
            Debug.Log($"Now in {currentFloor}, trigger unlocked → re-enter to trigger elevator.");
        }
        else
        {
            Debug.LogWarning("elevator is moving... cannot unlock trigger now.");
        }
    }

    private IEnumerator StartElevatorAfterDelay()
    {
        isMoving = true;
        isTriggerUnlocked = false;

        // If you want to use TriggerDelay instead of fixed 3 seconds,
        // replace this countdown with `yield return new WaitForSeconds(TriggerDelay);`
        for (int i = Mathf.CeilToInt(TriggerDelay); i > 0; i--)
        {
            Debug.Log($"going to start in {i} seconds...");
            yield return new WaitForSeconds(1f);
        }

        StartElevatorMove();
    }

    private void StartElevatorMove()
    {
        targetPos = initPlatformPos;

        if (isAtLow)
        {
            targetPos.y = HighPosY;
            Debug.Log("elevator start moving from floor 0 to floor 1");
        }
        else
        {
            targetPos.y = LowPosY;
            Debug.Log("elevator start moving from floor 1 to floor 0");
        }

        StartCoroutine(SmoothMoveToTarget());
    }

    private IEnumerator SmoothMoveToTarget()
    {
        while (Mathf.Abs(GetCurrentY() - targetPos.y) > 0.01f)
        {
            if (UseLocalSpace)
            {
                Platform.localPosition = Vector3.MoveTowards(
                    Platform.localPosition, targetPos, MoveSpeed * Time.deltaTime
                );
            }
            else
            {
                Platform.position = Vector3.MoveTowards(
                    Platform.position, targetPos, MoveSpeed * Time.deltaTime
                );
            }

            yield return null;
        }

        SetElevatorY(targetPos.y);
        isAtLow = !isAtLow;
        isMoving = false;

        string currentFloor = isAtLow ? "floor 0" : "floor 1";
        Debug.Log($"Elevator movement completed, current at {currentFloor} (Y={targetPos.y}m), " +
                  $"trigger is locked → leave the zone and re-enter to trigger.");
    }

    private float GetCurrentY()
    {
        return UseLocalSpace ? Platform.localPosition.y : Platform.position.y;
    }

    private void SetElevatorY(float y)
    {
        if (UseLocalSpace)
        {
            Vector3 pos = Platform.localPosition;
            pos.y = y;
            Platform.localPosition = pos;
        }
        else
        {
            Vector3 pos = Platform.position;
            pos.y = y;
            Platform.position = pos;
        }
    }

    void OnDisable()
    {
        ElevatorTriggerDetector.OnElevatorEntered -= OnRobotEnterZone;
        ElevatorTriggerDetector.OnElevatorExited -= OnRobotLeaveZone;

        isMoving = false;
        isTriggerUnlocked = false;
    }
}