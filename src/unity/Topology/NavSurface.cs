using UnityEngine;

public enum SurfaceLevel
{
    Ground = 0,
    Platform = 1,
    Floor2 = 2
}

[ExecuteAlways]
public class NavSurface : MonoBehaviour
{
    public SurfaceLevel level;

    [Header("Auto set by level")]
    public float heightY;

    [Header("Surface size in XZ")]
    public Vector2 sizeXZ = new Vector2(10f, 10f);

    private void OnValidate()
    {
        switch (level)
        {
            case SurfaceLevel.Ground:
                heightY = 0f;
                break;
            case SurfaceLevel.Platform:
                heightY = 0.5f;
                break;
            case SurfaceLevel.Floor2:
                heightY = 4f;
                break;
        }

        Vector3 p = transform.position;
        transform.position = new Vector3(p.x, heightY, p.z);
    }

    public bool ContainsXZ(Vector3 worldPos)
    {
        Vector3 local = transform.InverseTransformPoint(worldPos);

        return Mathf.Abs(local.x) <= sizeXZ.x * 0.5f &&
               Mathf.Abs(local.z) <= sizeXZ.y * 0.5f;
    }

    private void OnDrawGizmos()
    {
        Color c = level switch
        {
            SurfaceLevel.Ground => new Color(0.2f, 0.9f, 0.2f, 0.2f),
            SurfaceLevel.Platform => new Color(1f, 0.7f, 0.1f, 0.25f),
            SurfaceLevel.Floor2 => new Color(0.2f, 0.6f, 1f, 0.25f),
            _ => new Color(1f, 1f, 1f, 0.2f)
        };

        Gizmos.color = c;
        Matrix4x4 old = Gizmos.matrix;
        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.DrawCube(Vector3.zero, new Vector3(sizeXZ.x, 0.05f, sizeXZ.y));
        Gizmos.DrawWireCube(Vector3.zero, new Vector3(sizeXZ.x, 0.05f, sizeXZ.y));
        Gizmos.matrix = old;
    }
}