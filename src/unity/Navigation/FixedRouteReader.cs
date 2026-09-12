using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class FixedRouteData
{
    public string routeName;
    public Transform routeRoot;
    public List<Vector3> worldWaypoints = new List<Vector3>();
}

public class FixedRouteReader : MonoBehaviour
{
    public List<FixedRouteData> routes = new List<FixedRouteData>();

    [Header("Combined result")]
    public List<Vector3> combinedWaypoints = new List<Vector3>();

    [ContextMenu("Read All Routes")]
    public void ReadAllRoutes()
    {
        foreach (var route in routes)
        {
            if (route == null || route.routeRoot == null)
                continue;

            route.worldWaypoints.Clear();

            for (int i = 0; i < route.routeRoot.childCount; i++)
            {
                Transform child = route.routeRoot.GetChild(i);
                route.worldWaypoints.Add(child.position);
            }

            string msg = $"[FixedRouteReader] Route {route.routeRoot.name}: ";
            for (int i = 0; i < route.worldWaypoints.Count; i++)
            {
                msg += route.worldWaypoints[i].ToString("F2");
                if (i < route.worldWaypoints.Count - 1)
                    msg += " -> ";
            }

            Debug.Log(msg);
        }
    }

    [ContextMenu("Build Combined Route")]
    public void BuildCombinedRoute()
    {
        combinedWaypoints.Clear();

        foreach (var route in routes)
        {
            if (route == null || route.worldWaypoints == null || route.worldWaypoints.Count == 0)
                continue;

            for (int i = 0; i < route.worldWaypoints.Count; i++)
            {
                Vector3 p = route.worldWaypoints[i];

                if (combinedWaypoints.Count == 0 ||
                    Vector3.Distance(combinedWaypoints[combinedWaypoints.Count - 1], p) > 0.05f)
                {
                    combinedWaypoints.Add(p);
                }
            }
        }

        string msg = "[FixedRouteReader] Combined Route: ";
        for (int i = 0; i < combinedWaypoints.Count; i++)
        {
            msg += combinedWaypoints[i].ToString("F2");
            if (i < combinedWaypoints.Count - 1)
                msg += " -> ";
        }

        Debug.Log(msg);
    }
}