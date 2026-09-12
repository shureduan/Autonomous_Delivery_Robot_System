using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class TopologyGraphManager : MonoBehaviour
{
    public List<NavSurface> surfaces = new List<NavSurface>();
    public List<NavZone> zones = new List<NavZone>();
    public List<TopoNode> nodes = new List<TopoNode>();

    [ContextMenu("Refresh All Navigation Objects")]
    public void RefreshAll()
    {
        surfaces = FindObjectsOfType<NavSurface>().OrderBy(x => x.name).ToList();
        zones = FindObjectsOfType<NavZone>().OrderBy(x => x.name).ToList();
        nodes = FindObjectsOfType<TopoNode>().OrderBy(x => x.name).ToList();

        Debug.Log($"[TopologyGraphManager] surfaces={surfaces.Count}, zones={zones.Count}, nodes={nodes.Count}");
    }

    public TopoNode GetNodeById(string id)
    {
        return nodes.FirstOrDefault(n => n != null && n.nodeId == id);
    }

    public NavZone GetZoneById(string id)
    {
        return zones.FirstOrDefault(z => z != null && z.zoneId == id);
    }

    public NavSurface GetSurfaceContainingPoint(Vector3 worldPos, SurfaceLevel level)
    {
        foreach (var s in surfaces)
        {
            if (s == null) continue;
            if (s.level != level) continue;
            if (s.ContainsXZ(worldPos)) return s;
        }
        return null;
    }

    public TopoNode GetNearestNodeOnLevel(Vector3 worldPos, SurfaceLevel level, params NodeType[] allowedTypes)
    {
        TopoNode best = null;
        float bestDist = float.PositiveInfinity;

        foreach (var node in nodes)
        {
            if (node == null) continue;
            if (node.level != level) continue;

            if (allowedTypes != null && allowedTypes.Length > 0)
            {
                bool typeMatch = false;
                foreach (var t in allowedTypes)
                {
                    if (node.nodeType == t)
                    {
                        typeMatch = true;
                        break;
                    }
                }
                if (!typeMatch) continue;
            }

            float d = Vector3.Distance(worldPos, node.transform.position);
            if (d < bestDist)
            {
                bestDist = d;
                best = node;
            }
        }

        return best;
    }

    public TopoNode GetBestEntryNodeForPoint(Vector3 worldPos, SurfaceLevel level)
    {
        if (GetSurfaceContainingPoint(worldPos, level) == null)
        {
            Debug.LogWarning($"Point {worldPos} is not inside any {level} surface.");
            return null;
        }

        if (level == SurfaceLevel.Platform)
        {
            return GetNearestNodeOnLevel(worldPos, level, NodeType.RampTop);
        }

        if (level == SurfaceLevel.Floor2)
        {
            return GetNearestNodeOnLevel(worldPos, level, NodeType.ElevatorPoint);
        }

        if (level == SurfaceLevel.Ground)
        {
            return GetNearestNodeOnLevel(worldPos, level, NodeType.Depot, NodeType.RampBottom);
        }

        return null;
    }

    public List<TopoNode> FindPathBetweenZones(string startZoneId, string goalZoneId)
    {
        NavZone startZone = GetZoneById(startZoneId);
        NavZone goalZone = GetZoneById(goalZoneId);

        if (startZone == null || goalZone == null)
        {
            Debug.LogWarning("Start zone or goal zone not found.");
            return null;
        }

        if (startZone.entryNode == null || goalZone.entryNode == null)
        {
            Debug.LogWarning("Start zone or goal zone entryNode is missing.");
            return null;
        }

        return FindPath(startZone.entryNode, goalZone.entryNode);
    }

    public List<TopoNode> FindPath(TopoNode start, TopoNode goal)
    {
        if (start == null || goal == null)
            return null;

        Dictionary<TopoNode, float> dist = new Dictionary<TopoNode, float>();
        Dictionary<TopoNode, TopoNode> cameFrom = new Dictionary<TopoNode, TopoNode>();
        List<TopoNode> open = new List<TopoNode>();

        foreach (var node in nodes)
        {
            if (node != null)
                dist[node] = float.PositiveInfinity;
        }

        dist[start] = 0f;
        cameFrom[start] = null;
        open.Add(start);

        while (open.Count > 0)
        {
            TopoNode current = open.OrderBy(n => dist[n]).First();

            if (current == goal)
                return ReconstructPath(cameFrom, goal);

            open.Remove(current);

            foreach (TopoNode next in current.neighbors)
            {
                if (next == null)
                    continue;

                float edgeCost = Vector3.Distance(current.transform.position, next.transform.position);
                float newCost = dist[current] + edgeCost;

                if (!dist.ContainsKey(next) || newCost < dist[next])
                {
                    dist[next] = newCost;
                    cameFrom[next] = current;

                    if (!open.Contains(next))
                        open.Add(next);
                }
            }
        }

        Debug.LogWarning("No path found.");
        return null;
    }

    private List<TopoNode> ReconstructPath(Dictionary<TopoNode, TopoNode> cameFrom, TopoNode goal)
    {
        List<TopoNode> path = new List<TopoNode>();
        TopoNode current = goal;

        while (current != null)
        {
            path.Add(current);
            current = cameFrom[current];
        }

        path.Reverse();
        return path;
    }

    private void Awake()
    {
        RefreshAll();
    }
}