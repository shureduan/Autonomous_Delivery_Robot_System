using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

// 挂载在TriggerZone上，检测物体进入+离开，发送对应事件
public class ElevatorTriggerDetector : MonoBehaviour
{
    // 进入触发区事件、离开触发区事件
    public static event Action OnElevatorEntered;
    public static event Action OnElevatorExited;

    [Header("Stay Detection")]
    public float requiredStaySeconds = 3f;

    private Collider triggerCollider;

    // 记录每个进入物体的协程
    private readonly Dictionary<Collider, Coroutine> pendingEnterCoroutines = new Dictionary<Collider, Coroutine>();

    // 记录哪些物体已经满足“停留3秒”并成功触发过进入事件
    private readonly HashSet<Collider> confirmedInside = new HashSet<Collider>();

    void Start()
    {
        // 自动启用触发器
        triggerCollider = GetComponent<Collider>();
        if (triggerCollider != null)
        {
            triggerCollider.isTrigger = true;
            Debug.Log("✅ 触发区已启用，检测进入/离开事件");
        }
        else
        {
            Debug.LogError("❌ 给TriggerZone添加Box Collider组件！");
        }
    }

    // 检测物体进入触发区
    void OnTriggerEnter(Collider other)
    {
        if (other == null)
            return;

        // 已经确认在里面的，不重复计时
        if (confirmedInside.Contains(other))
            return;

        // 已经有等待协程的，不重复启动
        if (pendingEnterCoroutines.ContainsKey(other))
            return;

        Debug.Log($"🕒 检测到物体进入触发区，开始计时 {requiredStaySeconds} 秒: {other.name}");
        Coroutine co = StartCoroutine(ConfirmStayAndInvoke(other));
        pendingEnterCoroutines[other] = co;
    }

    // 检测物体离开触发区
    void OnTriggerExit(Collider other)
    {
        if (other == null)
            return;

        // 如果还在等待3秒确认，说明只是路过，取消进入
        if (pendingEnterCoroutines.TryGetValue(other, out Coroutine co))
        {
            StopCoroutine(co);
            pendingEnterCoroutines.Remove(other);
            Debug.Log($"🚶 物体在 {requiredStaySeconds} 秒内离开，判定为路过，不触发进入事件: {other.name}");
        }

        // 只有已经确认停留满3秒的物体，离开时才触发退出事件
        if (confirmedInside.Contains(other))
        {
            confirmedInside.Remove(other);
            Debug.Log($"✅ 检测到物体离开触发区（已满足停留条件）: {other.name}");
            OnElevatorExited?.Invoke();
        }
    }

    private IEnumerator ConfirmStayAndInvoke(Collider other)
    {
        yield return new WaitForSeconds(requiredStaySeconds);

        // 等待结束后，如果还没被移除，说明它确实停留满3秒
        if (other != null && pendingEnterCoroutines.ContainsKey(other))
        {
            pendingEnterCoroutines.Remove(other);
            confirmedInside.Add(other);

            Debug.Log($"✅ 物体已在触发区停留满 {requiredStaySeconds} 秒，触发进入事件: {other.name}");
            OnElevatorEntered?.Invoke();
        }
    }
}