using System;
using System.Net;
using System.Net.Sockets;
using UnityEngine;

[RequireComponent(typeof(MultiHeightLidar2D))]
public class LidarUdpSender : MonoBehaviour
{
    public MultiHeightLidar2D lidar;

    [Header("UDP Target")]
    public string ip = "127.0.0.1";
    public int port = 9999;

    [Header("Send")]
    public bool send = true;

    private UdpClient udp;
    private IPEndPoint ep;

    void Awake()
    {
        if (lidar == null) lidar = GetComponent<MultiHeightLidar2D>();
        ep = new IPEndPoint(IPAddress.Parse(ip), port);
        udp = new UdpClient();
    }

    void OnEnable()
    {
        if (lidar == null) lidar = GetComponent<MultiHeightLidar2D>();
        lidar.OnScanMulti += HandleScanMulti;
    }

    void OnDisable()
    {
        if (lidar != null) lidar.OnScanMulti -= HandleScanMulti;
    }

    void OnDestroy()
    {
        try { udp?.Close(); } catch { }
    }

    void HandleScanMulti(double tSec, float angleMin, float angleInc, int layerCount, float[] heights, float[] flatRanges)
    {
        if (!send) return;

        int n = lidar.numRays;
        int totalRanges = flatRanges.Length;

        Vector3 p = transform.position;
        float posX = p.x;
        float posZ = p.z;

        Vector3 f = transform.right;
        float fwdX = f.x;
        float fwdZ = f.z;

        // Packet:
        // magic "LDM3"
        // uint32 nRays
        // uint32 nLayers
        // float angleMin angleInc rangeMin rangeMax
        // float posX posZ fwdX fwdZ
        // float heights[nLayers]
        // float flatRanges[nLayers * nRays]

        int total = 4 + 4 + 4 + 16 + 16 + 4 * layerCount + 4 * totalRanges;
        byte[] buf = new byte[total];
        int off = 0;

        buf[off++] = (byte)'L';
        buf[off++] = (byte)'D';
        buf[off++] = (byte)'M';
        buf[off++] = (byte)'3';

        WriteU32(buf, ref off, (uint)n);
        WriteU32(buf, ref off, (uint)layerCount);

        WriteF32(buf, ref off, angleMin);
        WriteF32(buf, ref off, angleInc);
        WriteF32(buf, ref off, lidar.rangeMin);
        WriteF32(buf, ref off, lidar.rangeMax);

        WriteF32(buf, ref off, posX);
        WriteF32(buf, ref off, posZ);
        WriteF32(buf, ref off, fwdX);
        WriteF32(buf, ref off, fwdZ);

        for (int k = 0; k < layerCount; k++)
            WriteF32(buf, ref off, heights[k]);

        for (int i = 0; i < totalRanges; i++)
            WriteF32(buf, ref off, flatRanges[i]);

        udp.Send(buf, buf.Length, ep);
    }

    static void WriteU32(byte[] b, ref int o, uint v)
    {
        b[o++] = (byte)(v & 0xFF);
        b[o++] = (byte)((v >> 8) & 0xFF);
        b[o++] = (byte)((v >> 16) & 0xFF);
        b[o++] = (byte)((v >> 24) & 0xFF);
    }

    static void WriteF32(byte[] b, ref int o, float v)
    {
        byte[] bytes = BitConverter.GetBytes(v);
        Buffer.BlockCopy(bytes, 0, b, o, 4);
        o += 4;
    }
}