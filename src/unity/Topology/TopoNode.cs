using System.Collections.Generic;
using UnityEngine;

public enum NodeType
{
    Depot,
    RampBottom,
    RampTop,
    ElevatorPoint,
    TaskPoint
}

[ExecuteAlways]
public class TopoNode : MonoBehaviour
{
    public string nodeId;
    public NodeType nodeType;
    public SurfaceLevel level;

    [Header("Connected nodes")]
    public List<TopoNode> neighbors = new List<TopoNode>();

    private void OnDrawGizmos()
    {
        Color c = nodeType switch
        {
            NodeType.Depot => Color.cyan,
            NodeType.RampBottom => new Color(1f, 0.5f, 0f),
            NodeType.RampTop => new Color(1f, 0.85f, 0.1f),
            NodeType.ElevatorPoint => Color.green,
            NodeType.TaskPoint => Color.magenta,
            _ => Color.white
        };

        Gizmos.color = c;
        Gizmos.DrawSphere(transform.position, 0.18f);

        if (neighbors != null)
        {
            Gizmos.color = new Color(1f, 1f, 1f, 0.7f);
            foreach (var n in neighbors)
            {
                if (n != null)
                {
                    Gizmos.DrawLine(transform.position, n.transform.position);
                }
            }
        }
    }
}
