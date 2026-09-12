using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

public class SceneNodeCsvExporter : MonoBehaviour
{
    [Header("References")]
    public Transform navNodesRoot;          // 拖 NavNodes
    public Transform generatedRegionsRoot;  // 拖 RegionGenerator/GeneratedRegions

    [Header("Project-relative output")]
    public string outputRelativeFolder = "GeneratedData";
    public string outputFileName = "scene_nodes.csv";

    [Header("Export Timing")]
    public bool exportOnStart = true;
    public bool overwriteFile = true;

    [Header("Options")]
    public bool includeUnloadRegion = false;

    [Header("Ignore Nav Node Name Contains")]
    public List<string> ignoreKeywords = new List<string>
    {
        "Task"
    };

    private IEnumerator Start()
    {
        if (exportOnStart)
        {
            // 多等几帧，尽量确保 DestinationRegionGenerator 已经生成完
            yield return null;
            yield return null;
            yield return new WaitForEndOfFrame();
            ExportCsv();
        }
    }

    [ContextMenu("Export CSV Now")]
    public void ExportCsv()
    {
        if (navNodesRoot == null)
        {
            Debug.LogError("[SceneNodeCsvExporter] navNodesRoot 未指定。");
            return;
        }

        if (generatedRegionsRoot == null)
        {
            Debug.LogError("[SceneNodeCsvExporter] generatedRegionsRoot 未指定。");
            return;
        }

        string filePath = ResolveOutputFilePath();
        if (string.IsNullOrWhiteSpace(filePath))
        {
            Debug.LogError("[SceneNodeCsvExporter] 输出路径为空，导出失败。");
            return;
        }

        string folderPath = Path.GetDirectoryName(filePath);
        if (string.IsNullOrWhiteSpace(folderPath))
        {
            Debug.LogError($"[SceneNodeCsvExporter] 无法解析输出文件夹路径: {filePath}");
            return;
        }

        if (!Directory.Exists(folderPath))
        {
            Directory.CreateDirectory(folderPath);
            Debug.Log($"[SceneNodeCsvExporter] Created output folder: {folderPath}");
        }

        if (File.Exists(filePath) && !overwriteFile)
        {
            string name = Path.GetFileNameWithoutExtension(filePath);
            string ext = Path.GetExtension(filePath);
            string dir = Path.GetDirectoryName(filePath);
            string stamp = System.DateTime.Now.ToString("yyyyMMdd_HHmmss");
            filePath = Path.Combine(dir, $"{name}_{stamp}{ext}");
        }

        StringBuilder sb = new StringBuilder();
        sb.AppendLine("id,type,floor,x,y,z");

        int navCount = ExportNavNodes(sb);
        int regionCount = ExportGeneratedRegionsFromHierarchy(sb);

        File.WriteAllText(filePath, sb.ToString(), Encoding.UTF8);

        Debug.Log($"[SceneNodeCsvExporter] navNodesRoot = {navNodesRoot.name}, childCount = {navNodesRoot.childCount}");
        Debug.Log($"[SceneNodeCsvExporter] generatedRegionsRoot = {generatedRegionsRoot.name}, childCount = {generatedRegionsRoot.childCount}");
        Debug.Log($"[SceneNodeCsvExporter] Exported nav nodes = {navCount}, regions = {regionCount}");
        Debug.Log($"[SceneNodeCsvExporter] CSV exported to: {filePath}");
    }

    private string ResolveOutputFilePath()
    {
        string projectRoot = GetProjectRootPath();
        string folderPath = Path.Combine(projectRoot, outputRelativeFolder);
        return Path.Combine(folderPath, outputFileName);
    }

    private int ExportNavNodes(StringBuilder sb)
    {
        int count = 0;

        foreach (Transform child in navNodesRoot)
        {
            if (child == null) continue;

            string nodeName = child.name;
            if (ShouldIgnoreNavNode(nodeName)) continue;

            int floor = InferFloorFromName(nodeName);
            Vector3 p = child.position;

            WriteRow(sb, nodeName, "topology_node", floor, p);
            count++;

            Debug.Log($"[SceneNodeCsvExporter] Export nav node: {nodeName}, floor={floor}, pos={p}");
        }

        return count;
    }

    private int ExportGeneratedRegionsFromHierarchy(StringBuilder sb)
    {
        int count = 0;

        foreach (Transform child in generatedRegionsRoot)
        {
            if (child == null) continue;

            string objName = child.name;
            Debug.Log($"[SceneNodeCsvExporter] Found region child: {objName}");

            // 例如：Region_A_Floor1 / Region_B_Floor1 / Region_C_Floor2 / Region_Unload_Floor0
            if (!objName.StartsWith("Region_")) continue;

            if (!includeUnloadRegion && objName.Contains("Unload")) continue;

            int floor = InferFloorFromRegionName(objName);

            // 取区域中心而不是 cube 顶面中心
            Vector3 worldPos = child.position;
            float halfHeight = child.localScale.y * 0.5f;

            Vector3 center = new Vector3(
                worldPos.x,
                worldPos.y - halfHeight,
                worldPos.z
            );

            WriteRow(sb, objName, "goal_region_center", floor, center);
            count++;

            Debug.Log($"[SceneNodeCsvExporter] Export region: {objName}, floor={floor}, center={center}");
        }

        return count;
    }

    private bool ShouldIgnoreNavNode(string nodeName)
    {
        if (string.IsNullOrWhiteSpace(nodeName)) return true;

        foreach (string keyword in ignoreKeywords)
        {
            if (!string.IsNullOrWhiteSpace(keyword) && nodeName.Contains(keyword))
                return true;
        }

        return false;
    }

    private int InferFloorFromName(string nodeName)
    {
        string lower = nodeName.ToLower();

        if (lower.Contains("ground")) return 0;
        if (lower.Contains("platform")) return 1;
        if (lower.Contains("floor2")) return 2;

        return -1;
    }

    private int InferFloorFromRegionName(string regionName)
    {
        string lower = regionName.ToLower();

        if (lower.Contains("floor0")) return 0;
        if (lower.Contains("floor1")) return 1;
        if (lower.Contains("floor2")) return 2;

        return -1;
    }

    private void WriteRow(StringBuilder sb, string id, string type, int floor, Vector3 p)
    {
        sb.AppendLine(string.Format(
            CultureInfo.InvariantCulture,
            "{0},{1},{2},{3:F6},{4:F6},{5:F6}",
            EscapeCsv(id),
            EscapeCsv(type),
            floor,
            p.x,
            p.y,
            p.z
        ));
    }

    private string EscapeCsv(string s)
    {
        if (string.IsNullOrEmpty(s)) return "";
        if (s.Contains(",") || s.Contains("\"") || s.Contains("\n"))
        {
            s = s.Replace("\"", "\"\"");
            return $"\"{s}\"";
        }
        return s;
    }

    private string GetProjectRootPath()
    {
#if UNITY_EDITOR
        return Directory.GetParent(Application.dataPath).FullName;
#else
        return Application.dataPath;
#endif
    }
}
