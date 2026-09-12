using System;
using System.Collections;
using UnityEngine;

public class MultiHeightLidar2D : MonoBehaviour
{
    [Header("Scan Geometry")]
    public float fovDeg = 360f;
    public int numRays = 720;
    public float rangeMin = 0.05f;
    public float rangeMax = 10f;

    [Header("Multi-height sampling (meters above LidarFrame origin along +Y)")]
    public float[] heights = new float[] { 0.1f, 0.7f, 1.3f };

    [Header("Physics")]
    public LayerMask obstacleMask = 1 << 0;

    [Header("Timing")]
    public float scanHz = 10f;

    [Header("Debug")]
    public bool drawGizmos = true;
    public float gizmoRayLen = 5f;

    // 保留最低层或展示层可视化用
    public float[] ranges { get; private set; }

    // 真正多层数据：layer-major flatten
    // 索引: flatRanges[layer * numRays + ray]
    public float[] flatRanges { get; private set; }

    public double lastScanTime { get; private set; }

    // 新事件：多层一起发
    public event Action<double, float, float, int, float[], float[]> OnScanMulti;
    // 参数:
    // tSec, angleMin, angleInc, layerCount, heights[], flatRanges[]

    Coroutine scanRoutine;

    void OnEnable()
    {
        ValidateParams();
        ranges = new float[numRays];
        flatRanges = new float[heights.Length * numRays];
        scanRoutine = StartCoroutine(ScanLoop());
    }

    void OnDisable()
    {
        if (scanRoutine != null) StopCoroutine(scanRoutine);
        scanRoutine = null;
    }

    void ValidateParams()
    {
        if (numRays < 2) numRays = 2;
        if (rangeMax <= rangeMin) rangeMax = rangeMin + 0.1f;

        if (heights == null || heights.Length == 0)
            heights = new float[] { 0.1f, 0.7f, 1.3f };
    }

    IEnumerator ScanLoop()
    {
        var wait = new WaitForSeconds(1f / Mathf.Max(0.1f, scanHz));
        while (true)
        {
            DoScan();
            yield return wait;
        }
    }

    void DoScan()
    {
        float angleMin = -fovDeg * 0.5f * Mathf.Deg2Rad;
        float angleMax = +fovDeg * 0.5f * Mathf.Deg2Rad;
        float angleInc = (angleMax - angleMin) / (numRays - 1);

        int layerCount = heights.Length;

        for (int i = 0; i < numRays; i++)
        {
            float a = angleMin + i * angleInc;
            Vector3 dirLocal = new Vector3(Mathf.Cos(a), 0f, Mathf.Sin(a));
            Vector3 dirWorld = transform.TransformDirection(dirLocal).normalized;

            for (int k = 0; k < layerCount; k++)
            {
                Vector3 origin = transform.position + transform.up * heights[k];

                float d = rangeMax;
                if (Physics.Raycast(origin, dirWorld, out RaycastHit hit, rangeMax, obstacleMask, QueryTriggerInteraction.Ignore))
                {
                    d = hit.distance;
                }

                if (d < rangeMin) d = rangeMin;
                if (d > rangeMax) d = rangeMax;

                flatRanges[k * numRays + i] = d;

                // 给调试显示一个默认层，这里用最低层
                if (k == 0)
                    ranges[i] = d;
            }
        }

        lastScanTime = Time.timeAsDouble;
        OnScanMulti?.Invoke(lastScanTime, angleMin, angleInc, layerCount, heights, flatRanges);
    }

    void OnDrawGizmosSelected()
    {
        if (!drawGizmos) return;
        if (!Application.isPlaying) return;
        if (ranges == null || ranges.Length != numRays) return;

        float angleMin = -fovDeg * 0.5f * Mathf.Deg2Rad;
        float angleMax = +fovDeg * 0.5f * Mathf.Deg2Rad;
        float angleInc = (angleMax - angleMin) / (numRays - 1);

        int step = Mathf.Max(1, numRays / 90);
        for (int i = 0; i < numRays; i += step)
        {
            float a = angleMin + i * angleInc;
            Vector3 dirLocal = new Vector3(Mathf.Cos(a), 0f, Mathf.Sin(a));
            Vector3 dirWorld = transform.TransformDirection(dirLocal).normalized;

            Vector3 o = transform.position + transform.up * heights[0];
            float len = Mathf.Min(ranges[i], gizmoRayLen);
            Gizmos.DrawLine(o, o + dirWorld * len);
        }
    }
}