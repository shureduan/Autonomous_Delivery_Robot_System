using UnityEngine;

public enum ZoneType
{
    Depot,
    Task
}

[ExecuteAlways]
public class NavZone : MonoBehaviour
{
    public string zoneId;
    public ZoneType zoneType;
    public SurfaceLevel level;

    [Header("Zone size in XZ")]
    public Vector2 sizeXZ = new Vector2(3f, 3f);

    [Header("Entry node of this zone")]
    public TopoNode entryNode;

    private void OnDrawGizmos()
    {
        Color c = zoneType switch
        {
            ZoneType.Depot => new Color(0f, 1f, 1f, 0.25f),
            ZoneType.Task => new Color(1f, 0f, 1f, 0.25f),
            _ => new Color(1f, 1f, 1f, 0.2f)
        };

        Gizmos.color = c;
        Matrix4x4 old = Gizmos.matrix;
        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.DrawCube(Vector3.zero, new Vector3(sizeXZ.x, 0.05f, sizeXZ.y));
        Gizmos.DrawWireCube(Vector3.zero, new Vector3(sizeXZ.x, 0.05f, sizeXZ.y));
        Gizmos.matrix = old;

        if (entryNode != null)
        {
            Gizmos.color = Color.white;
            Gizmos.DrawLine(transform.position, entryNode.transform.position);
        }
    }
}