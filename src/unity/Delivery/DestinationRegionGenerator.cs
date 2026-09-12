using System.Collections.Generic;
using UnityEngine;

public class DestinationRegionGenerator : MonoBehaviour
{
    [Header("Region Settings")]
    public float regionSize = 3f;                 // 3m x 3m
    public float regionHeight = 0.15f;            // 区域可视化厚度
    public float minDistanceBetween1F = 12f;      // 一楼两个区域中心点最小距离
    public int maxTryCount = 200;                 // 随机尝试次数

    [Header("Unload Zone")]
    public Vector3 unloadCenter = new Vector3(-19.36f, 0.0f, -13.11f);   // 固定卸货区中心
    public Material unloadMaterial;                                      // 推荐手动拖橘色材质

    [Header("Visual")]
    public Material regionMaterial;               // 推荐手动拖绿色材质
    public bool generateOnStart = true;

    [Header("Parent")]
    public Transform regionParent;

    private readonly List<GeneratedRegion> generatedRegions = new List<GeneratedRegion>();

    // 一楼 L 形：两个矩形
    private XZRect oneFRectA = new XZRect(-30f, -5f, 30f, 50f, 0.5f);
    private XZRect oneFRectB = new XZRect(-5f, 15f, 5f, 50f, 0.5f);

    // 二楼矩形
    private XZRect twoFRect = new XZRect(-5f, 15f, 25f, 50f, 4f);

    [System.Serializable]
    public class XZRect
    {
        public float xMin;
        public float xMax;
        public float zMin;
        public float zMax;
        public float y;

        public XZRect(float xMin, float xMax, float zMin, float zMax, float y)
        {
            this.xMin = xMin;
            this.xMax = xMax;
            this.zMin = zMin;
            this.zMax = zMax;
            this.y = y;
        }

        public float Width => xMax - xMin;
        public float Height => zMax - zMin;
        public float Area => Mathf.Max(0f, Width) * Mathf.Max(0f, Height);

        public XZRect GetInsetRect(float size)
        {
            float half = size * 0.5f;
            return new XZRect(
                xMin + half,
                xMax - half,
                zMin + half,
                zMax - half,
                y
            );
        }

        public bool IsValid(float size)
        {
            return (xMax - xMin) >= size && (zMax - zMin) >= size;
        }

        public Vector3 GetRandomCenter(float size)
        {
            XZRect inset = GetInsetRect(size);
            float x = Random.Range(inset.xMin, inset.xMax);
            float z = Random.Range(inset.zMin, inset.zMax);
            return new Vector3(x, y, z);
        }
    }

    [System.Serializable]
    public class GeneratedRegion
    {
        public string regionId;
        public int floor;
        public Vector3 center;
        public GameObject visualObject;

        public GeneratedRegion(string regionId, int floor, Vector3 center, GameObject visualObject)
        {
            this.regionId = regionId;
            this.floor = floor;
            this.center = center;
            this.visualObject = visualObject;
        }
    }

    public IReadOnlyList<GeneratedRegion> Regions => generatedRegions;

    private void Start()
    {
        if (generateOnStart)
        {
            GenerateRegions();
        }
    }

    [ContextMenu("Generate Regions")]
    public void GenerateRegions()
    {
        ClearRegions();

        if (regionParent == null)
        {
            GameObject parentObj = new GameObject("GeneratedRegions");
            parentObj.transform.SetParent(transform);
            regionParent = parentObj.transform;
        }

        if (regionMaterial == null)
        {
            regionMaterial = CreateFallbackRegionMaterial();
        }

        if (unloadMaterial == null)
        {
            unloadMaterial = CreateFallbackUnloadMaterial();
        }

        Vector3 first1F, second1F;
        bool success1F = TryGenerateTwoRegionsOn1F(out first1F, out second1F);

        if (!success1F)
        {
            Debug.LogError("一楼区域生成失败：无法在最大尝试次数内找到满足距离约束的两个区域。");
            return;
        }

        Vector3 one2F = GetRandomPointFromWeightedRects(
            new List<XZRect> { twoFRect },
            regionSize
        );

        // 固定卸货区
        CreateRegion("Unload", 0, unloadCenter, unloadMaterial);

        // 三个随机目标区
        CreateRegion("A", 1, first1F, regionMaterial);
        CreateRegion("B", 1, second1F, regionMaterial);
        CreateRegion("C", 2, one2F, regionMaterial);

        Debug.Log("区域生成完成：");
        foreach (var r in generatedRegions)
        {
            Debug.Log($"{r.regionId} | Floor {r.floor} | Center = {r.center}");
        }

        PrintRegionMapCoords_OneToOne();
    }

    [ContextMenu("Clear Regions")]
    public void ClearRegions()
    {
        generatedRegions.Clear();

        if (regionParent != null)
        {
            for (int i = regionParent.childCount - 1; i >= 0; i--)
            {
                DestroyImmediate(regionParent.GetChild(i).gameObject);
            }
        }
    }

    private bool TryGenerateTwoRegionsOn1F(out Vector3 p1, out Vector3 p2)
    {
        List<XZRect> oneFloorRects = new List<XZRect> { oneFRectA, oneFRectB };

        for (int i = 0; i < maxTryCount; i++)
        {
            Vector3 candidate1 = GetRandomPointFromWeightedRects(oneFloorRects, regionSize);
            Vector3 candidate2 = GetRandomPointFromWeightedRects(oneFloorRects, regionSize);

            if (Vector3.Distance(candidate1, candidate2) >= minDistanceBetween1F)
            {
                p1 = candidate1;
                p2 = candidate2;
                return true;
            }
        }

        p1 = Vector3.zero;
        p2 = Vector3.zero;
        return false;
    }

    private Vector3 GetRandomPointFromWeightedRects(List<XZRect> rects, float size)
    {
        List<XZRect> validRects = new List<XZRect>();
        float totalArea = 0f;

        foreach (var rect in rects)
        {
            if (!rect.IsValid(size)) continue;

            float area = rect.GetInsetRect(size).Area;
            if (area > 0f)
            {
                validRects.Add(rect);
                totalArea += area;
            }
        }

        if (validRects.Count == 0)
        {
            Debug.LogError("没有有效可生成区域，请检查边界和 regionSize。");
            return Vector3.zero;
        }

        float pick = Random.Range(0f, totalArea);
        float accum = 0f;

        foreach (var rect in validRects)
        {
            float area = rect.GetInsetRect(size).Area;
            accum += area;
            if (pick <= accum)
            {
                return rect.GetRandomCenter(size);
            }
        }

        return validRects[validRects.Count - 1].GetRandomCenter(size);
    }

    private void CreateRegion(string regionId, int floor, Vector3 center, Material mat)
    {
        GameObject region = GameObject.CreatePrimitive(PrimitiveType.Cube);
        region.name = $"Region_{regionId}_Floor{floor}";
        region.transform.SetParent(regionParent);

        // Cube pivot 在中心，所以 y 需要抬半个高度
        region.transform.position = new Vector3(
            center.x,
            center.y + regionHeight * 0.5f,
            center.z
        );

        region.transform.localScale = new Vector3(regionSize, regionHeight, regionSize);

        var renderer = region.GetComponent<MeshRenderer>();
        renderer.sharedMaterial = mat;

        // 删除碰撞体
        var collider = region.GetComponent<Collider>();
        if (collider != null)
        {
            DestroyImmediate(collider);
        }

        generatedRegions.Add(new GeneratedRegion(regionId, floor, center, region));
    }

    private void PrintRegionMapCoords_OneToOne()
    {
        Debug.Log("=== Region Map Coordinates (1:1 with Unity) ===");

        const float mapRes = 0.10f;
        const float mapXMin = -50f;
        const float mapZMin = -50f;

        foreach (var r in generatedRegions)
        {
            float mapX = r.center.x;
            float mapZ = r.center.z;

            int gx = Mathf.FloorToInt((mapX - mapXMin) / mapRes);
            int gz = Mathf.FloorToInt((mapZ - mapZMin) / mapRes);

            Debug.Log(
                $"{r.regionId} | Floor {r.floor} | " +
                $"Unity=({r.center.x:F2}, {r.center.y:F2}, {r.center.z:F2}) | " +
                $"Map=({mapX:F2}, {mapZ:F2}) | " +
                $"Grid=({gx}, {gz})"
            );
        }
    }

    private Material CreateFallbackRegionMaterial()
    {
        Shader shader =
            Shader.Find("HDRP/Lit") ??
            Shader.Find("Universal Render Pipeline/Lit") ??
            Shader.Find("Standard");

        Material mat = new Material(shader);
        mat.name = "Fallback_Green_Region";

        Color green = new Color(0.1f, 1f, 0.2f, 0.65f);

        if (mat.HasProperty("_BaseColor"))
            mat.SetColor("_BaseColor", green);

        if (mat.HasProperty("_Color"))
            mat.SetColor("_Color", green);

        return mat;
    }

    private Material CreateFallbackUnloadMaterial()
    {
        Shader shader =
            Shader.Find("HDRP/Lit") ??
            Shader.Find("Universal Render Pipeline/Lit") ??
            Shader.Find("Standard");

        Material mat = new Material(shader);
        mat.name = "Fallback_Orange_Unload";

        Color orange = new Color(1.0f, 0.55f, 0.1f, 0.75f);

        if (mat.HasProperty("_BaseColor"))
            mat.SetColor("_BaseColor", orange);

        if (mat.HasProperty("_Color"))
            mat.SetColor("_Color", orange);

        return mat;
    }

    public Vector3 GetRegionCenter(string regionId)
    {
        foreach (var r in generatedRegions)
        {
            if (r.regionId == regionId)
                return r.center;
        }

        Debug.LogWarning($"未找到区域 {regionId}");
        return Vector3.zero;
    }
}