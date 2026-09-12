using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEngine;

public class UnityRouteExecutor : MonoBehaviour
{
    [Header("Follower")]
    public BodyXYDrive_PurePursuit follower;

    [Header("CSV Path")]
    [Tooltip("Project-relative path to the generated waypoint CSV")]
    public string csvFilePath = "GeneratedRoutes/unity_combined_route.csv";

    [Header("Run Control")]
    public bool runOnStart = true;
    public bool verboseLog = true;

    [Header("Debug")]
    public bool drawDebugRoute = true;
    public float gizmoSphereRadius = 0.15f;

    [Header("Upper Floor Elevator Entry Push")]
    public bool enableUpperFloorElevatorEntryPush = true;
    public float upperFloorElevatorPushDistance = 1.0f;

    private readonly List<RouteCommand> commands = new List<RouteCommand>();
    private readonly List<Vector3> allMovePoints = new List<Vector3>();

    private Coroutine runCoroutine;

    [Serializable]
    public class RouteRow
    {
        public int globalSeq;
        public int seq;
        public int tripId;
        public Vector3 point;
        public float waitSec;
        public string tag;
    }

    public abstract class RouteCommand
    {
        public int tripId;
    }

    public class MoveCommand : RouteCommand
    {
        public List<Vector3> routePoints = new List<Vector3>();
    }

    public class WaitCommand : RouteCommand
    {
        public Vector3 waitPoint;
        public float waitSeconds;
        public string tag;
    }

    private void Start()
    {
        if (runOnStart)
        {
            LoadAndRun();
        }
    }

    [ContextMenu("Load And Run")]
    public void LoadAndRun()
    {
        if (follower == null)
        {
            Debug.LogError("[UnityRouteExecutor] follower is null.");
            return;
        }

        commands.Clear();
        allMovePoints.Clear();

        List<RouteRow> rows = LoadRowsFromCsv(csvFilePath);
        if (rows.Count == 0)
        {
            Debug.LogWarning("[UnityRouteExecutor] No rows loaded from csv.");
            return;
        }

        BuildCommands(rows);

        if (verboseLog)
        {
            Debug.Log($"[UnityRouteExecutor] Loaded {rows.Count} csv rows, built {commands.Count} commands.");
        }

        if (runCoroutine != null)
        {
            StopCoroutine(runCoroutine);
        }

        runCoroutine = StartCoroutine(RunCommandsCoroutine());
    }

    List<RouteRow> LoadRowsFromCsv(string path)
    {
        List<RouteRow> rows = new List<RouteRow>();

        if (!File.Exists(path))
        {
            Debug.LogError($"[UnityRouteExecutor] CSV file not found: {path}");
            return rows;
        }

        string[] lines = File.ReadAllLines(path);
        if (lines == null || lines.Length <= 1)
        {
            Debug.LogWarning("[UnityRouteExecutor] CSV file is empty or header only.");
            return rows;
        }

        string[] header = SplitCsvLine(lines[0]);
        Dictionary<string, int> col = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
        for (int i = 0; i < header.Length; i++)
        {
            col[header[i].Trim()] = i;
        }

        string[] required = { "global_seq", "seq", "trip_id", "x", "y", "z", "wait_sec", "tag" };
        foreach (string r in required)
        {
            if (!col.ContainsKey(r))
            {
                Debug.LogError($"[UnityRouteExecutor] Missing required column: {r}");
                return rows;
            }
        }

        for (int i = 1; i < lines.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(lines[i])) continue;

            string[] parts = SplitCsvLine(lines[i]);

            try
            {
                RouteRow row = new RouteRow();
                row.globalSeq = ParseInt(parts, col["global_seq"]);
                row.seq = ParseInt(parts, col["seq"]);
                row.tripId = ParseInt(parts, col["trip_id"]);

                float x = ParseFloat(parts, col["x"]);
                float y = ParseFloat(parts, col["y"]);
                float z = ParseFloat(parts, col["z"]);

                row.point = new Vector3(x, y, z);
                row.waitSec = ParseFloat(parts, col["wait_sec"]);
                row.tag = GetString(parts, col["tag"]);

                rows.Add(row);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[UnityRouteExecutor] Failed to parse line {i + 1}: {e.Message}");
            }
        }

        rows = rows.OrderBy(r => r.globalSeq).ToList();
        return rows;
    }

    void BuildCommands(List<RouteRow> rows)
    {
        List<Vector3> currentMoveBuffer = new List<Vector3>();
        int currentTripId = rows.Count > 0 ? rows[0].tripId : -1;

        foreach (RouteRow row in rows)
        {
            if (row.tripId != currentTripId)
            {
                FlushMoveBuffer(currentTripId, currentMoveBuffer);
                currentTripId = row.tripId;
            }

            if (row.waitSec > 0f)
            {
                FlushMoveBuffer(currentTripId, currentMoveBuffer);

                WaitCommand waitCmd = new WaitCommand
                {
                    tripId = row.tripId,
                    waitPoint = row.point,
                    waitSeconds = row.waitSec,
                    tag = row.tag
                };
                commands.Add(waitCmd);
            }
            else
            {
                if (currentMoveBuffer.Count == 0 || Vector3.Distance(currentMoveBuffer[currentMoveBuffer.Count - 1], row.point) > 0.0001f)
                {
                    currentMoveBuffer.Add(row.point);
                    allMovePoints.Add(row.point);
                }
            }
        }

        FlushMoveBuffer(currentTripId, currentMoveBuffer);
    }

    void FlushMoveBuffer(int tripId, List<Vector3> buffer)
    {
        if (buffer == null || buffer.Count == 0) return;

        MoveCommand moveCmd = new MoveCommand
        {
            tripId = tripId,
            routePoints = new List<Vector3>(buffer)
        };
        commands.Add(moveCmd);

        buffer.Clear();
    }

    IEnumerator RunCommandsCoroutine()
    {
        if (verboseLog)
        {
            Debug.Log("[UnityRouteExecutor] Starting command execution...");
        }

        for (int i = 0; i < commands.Count; i++)
        {
            RouteCommand cmd = commands[i];

            if (cmd is MoveCommand move)
            {
                if (move.routePoints == null || move.routePoints.Count == 0)
                    continue;

                if (verboseLog)
                {
                    Debug.Log($"[UnityRouteExecutor] MoveCommand trip={move.tripId}, points={move.routePoints.Count}");
                }

                follower.SetRoute(new List<Vector3>(move.routePoints), true);

                yield return new WaitUntil(() => follower.finished || follower.hardStopped);

                if (verboseLog)
                {
                    Debug.Log($"[UnityRouteExecutor] MoveCommand trip={move.tripId} finished.");
                }
            }
            else if (cmd is WaitCommand wait)
            {
                if (verboseLog)
                {
                    Debug.Log($"[UnityRouteExecutor] WaitCommand trip={wait.tripId}, wait={wait.waitSeconds:F1}s, tag={wait.tag}, point={wait.waitPoint}");
                }

                MoveCommand prevMove = null;
                if (i > 0 && commands[i - 1] is MoveCommand pm)
                    prevMove = pm;

                // 先推 1 米，再等待 10 秒
                if (ShouldDoUpperFloorElevatorEntryPush(wait, prevMove))
                {
                    if (verboseLog)
                    {
                        Debug.Log($"[UnityRouteExecutor] Upper-floor elevator ENTRY push triggered at {wait.waitPoint}, pushDistance={upperFloorElevatorPushDistance:F2}");
                    }

                    Vector3 start = wait.waitPoint;
                    Vector3 end = wait.waitPoint + new Vector3(0f, 0f, -upperFloorElevatorPushDistance);

                    List<Vector3> pushRoute = new List<Vector3> { start, end };
                    follower.SetRoute(pushRoute, true);

                    yield return new WaitUntil(() => follower.finished || follower.hardStopped);

                    if (verboseLog)
                    {
                        Debug.Log("[UnityRouteExecutor] Upper-floor elevator ENTRY push finished.");
                    }
                }

                // 推完以后再等待
                yield return new WaitForSeconds(wait.waitSeconds);
            }
        }
    }

    private bool ShouldDoUpperFloorElevatorEntryPush(WaitCommand wait, MoveCommand prevMove)
    {
        if (!enableUpperFloorElevatorEntryPush)
            return false;

        if (wait == null || prevMove == null || prevMove.routePoints == null || prevMove.routePoints.Count == 0)
            return false;

        // 条件1：必须是二楼电梯点
        Vector3 upperElevatorPoint = new Vector3(12.5f, 4.0f, 17.5f);
        float posTol = 0.25f;

        if (Vector3.Distance(wait.waitPoint, upperElevatorPoint) > posTol)
            return false;

        // 条件2：必须是10秒等待
        if (wait.waitSeconds < 9.9f)
            return false;

        // 条件3：必须是前一个移动段的终点
        Vector3 prevMoveEnd = prevMove.routePoints[prevMove.routePoints.Count - 1];
        if (Vector3.Distance(prevMoveEnd, wait.waitPoint) > posTol)
            return false;

        return true;
    }

    int ParseInt(string[] parts, int idx)
    {
        return int.Parse(GetString(parts, idx), CultureInfo.InvariantCulture);
    }

    float ParseFloat(string[] parts, int idx)
    {
        return float.Parse(GetString(parts, idx), CultureInfo.InvariantCulture);
    }

    string GetString(string[] parts, int idx)
    {
        if (idx < 0 || idx >= parts.Length) return "";
        return parts[idx].Trim().Trim('"');
    }

    string[] SplitCsvLine(string line)
    {
        List<string> result = new List<string>();
        bool inQuotes = false;
        System.Text.StringBuilder sb = new System.Text.StringBuilder();

        for (int i = 0; i < line.Length; i++)
        {
            char ch = line[i];

            if (ch == '"')
            {
                inQuotes = !inQuotes;
                continue;
            }

            if (ch == ',' && !inQuotes)
            {
                result.Add(sb.ToString());
                sb.Length = 0;
            }
            else
            {
                sb.Append(ch);
            }
        }

        result.Add(sb.ToString());
        return result.ToArray();
    }

    private void OnDrawGizmos()
    {
        if (!drawDebugRoute || allMovePoints == null || allMovePoints.Count == 0)
            return;

        Gizmos.color = Color.green;
        for (int i = 0; i < allMovePoints.Count; i++)
        {
            Gizmos.DrawSphere(allMovePoints[i], gizmoSphereRadius);
            if (i < allMovePoints.Count - 1)
            {
                Gizmos.DrawLine(allMovePoints[i], allMovePoints[i + 1]);
            }
        }
    }
}
