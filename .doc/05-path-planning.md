# 路径规划：S型曲线与多轴同步

## 一、为什么用 S 曲线？

| 曲线类型 | 加速度 | 加加速度(jerk) | 优缺点 |
|---------|--------|---------------|--------|
| 梯形速度曲线 | 突变 (0→a_max) | 无穷大(瞬时) | 简单，但产生冲击和振动 |
| **S型曲线** | 连续变化 | **有限且可控** | 平滑，减少机械冲击和振动 |

机械臂高速运动时，梯形曲线会导致末端抖动，S 曲线通过限制 jerk 使速度/加速度平滑过渡。

## 二、7段 S 曲线结构

文件: `robot_controller/motors/s_curve_trajectory.hpp`

```
jerk:   +J₀    0    -J₀    0    -J₀    0   +J₀
         ┌─┐        ┌─┐        ┌─┐        ┌─┐
         │ │        │ │        │ │        │ │
    ─────┘ └────────┘ └────────┘ └────────┘ └─────

accel:   ↗  ──  ↘   ───  ↘  ──  ↗
        /   平   \  匀   \  平  /
       /    坦    \  速   \ 坦 /
      /            \       \/

vel:   /              ──────── \           /
      /                        \         /
     /                          \       /
    /                            \     /
   /                              \   /
  /                                \_/

pos:  ───────────────────────────────────────────
     T1    T2    T3    T4    T5    T6    T7
    (加加速)(匀加速)(减加速)(匀速)(加减速)(匀减速)(减减速)
```

### 各段含义

| 段 | 加加速度(jerk) | 加速度变化 | 速度变化 |
|----|---------------|-----------|---------|
| T1 (加加速) | +J_max | 0 → +A_max | 从0开始增加 |
| T2 (匀加速) | 0 | 保持 +A_max | 线性增加 |
| T3 (减加速) | -J_max | +A_max → 0 | 增加到 V_max |
| T4 (匀速) | 0 | 0 | 保持 V_max |
| T5 (加减速) | -J_max | 0 → -A_max | 从 V_max 减少 |
| T6 (匀减速) | 0 | 保持 -A_max | 线性减少 |
| T7 (减减速) | +J_max | -A_max → 0 | 减少到 0 |

---

## 三、核心算法 — compute_7segment()

文件: `robot_controller/motors/s_curve_trajectory.hpp` 第226行

```cpp
void compute_7segment(double dir) {
    double tj = 0.0;  // 加加速段时间 (T1=T3=T5=T7)
    double ta = 0.0;  // 匀加速段时间 (T2=T6)
    double tv = 0.0;  // 匀速段时间   (T4)

    // 临界速度: 三角速度曲线能达到的最大速度 (无匀加速段)
    double v_tri = a_max² / j_max;

    if (v_max > v_tri) {
        // === 情况1: 梯形速度曲线 (有匀加速段 T2/T6) ===
        tj = a_max / j_max;                    // 加加速时间
        ta = (v_max - v_tri) / a_max;          // 匀加速时间

        // 检查位移是否足够
        double d_ramp = 0.5 * v_max * (2*tj + ta);   // 一条斜坡的距离
        double d_min  = 2 * d_ramp;                    // 加速+减速最短距离

        if (L >= d_min) {
            tv = (L - d_min) / v_max;           // 有匀速段
        } else {
            // 位移不足 → 降低峰值速度 (无匀速段)
            v_peak = 解二次方程 v² + v_tri·v - a_max·L = 0
            if (v_peak <= v_tri) {
                // 回退到情况2
                v_max = pow(L * sqrt(j_max) / 2, 2/3);
                tj = sqrt(v_max / j_max);
                a_max = j_max * tj;
                ta = 0;
            }
        }
    } else {
        // === 情况2: 三角速度曲线 (无匀加速段 T2=T6=0) ===
        tj = sqrt(v_max / j_max);
        a_max = j_max * tj;              // 实际最大加速度 (比设定的小)
        ta = 0;

        double d_ramps = 2 * v_max * tj;  // 加速+减速最短距离
        if (L >= d_ramps) {
            tv = (L - d_ramps) / v_max;   // 有匀速段
        } else {
            // 位移还是不够 → 进一步降低 v_max
            v_max = pow(L * sqrt(j_max) / 2, 2/3);
            tj = sqrt(v_max / j_max);
            a_max = j_max * tj;
            tv = 0;
        }
    }

    // 7段时间: [T1, T2, T3, T4, T5, T6, T7]
    durations_ = {tj, ta, tj, tv, tj, ta, tj};
}
```

---

## 四、实时求值 — evaluate(t)

文件: `robot_controller/motors/s_curve_trajectory.hpp` 第61行

根据当前时间 t，确定处于哪一段，用该段公式计算 (pos, vel, acc, jerk):

```
Phase 1 (T1, 加加速, jerk=+J):
  a(t) = J·t
  v(t) = 1/2·J·t²
  p(t) = 1/6·J·t³

Phase 2 (T2, 匀加速, jerk=0):
  a(t) = A_max
  v(t) = v₁ + A_max·τ
  p(t) = p₁ + v₁·τ + 1/2·A_max·τ²

Phase 3 (T3, 减加速, jerk=-J):
  a(t) = A_max - J·τ
  v(t) = v₂ + A_max·τ - 1/2·J·τ²
  p(t) = p₂ + v₂·τ + 1/2·A_max·τ² - 1/6·J·τ³

Phase 4 (T4, 匀速):
  v(t) = V_max
  p(t) = p₃ + V_max·τ

Phase 5-7: 与 Phase 1-3 对称 (减速过程)
```

---

## 五、多轴同步 — SyncedTrajectory

文件: `robot_controller/motors/synced_trajectory.cpp`

### 问题
多个关节同时运动，但各关节行程不同、速度不同 → 到达时间不同 → 末端轨迹扭曲。

### 解决方案：时间缩放

```cpp
bool SyncedTrajectory::plan(p0, p1, params) {
    // 1. 各轴独立规划 S 曲线
    for (i: 0..N) {
        trajs_[i].plan(p0[i], p1[i], params[i]);
        sync_time_ = max(sync_time_, trajs_[i].total_time());  // 取最长时间
    }
}

std::vector<SCurvePoint> SyncedTrajectory::evaluate(double t) {
    for (i: 0..N) {
        double scale = Ti / sync_time_;   // 快轴缩放因子 < 1
        // 时间"拉长": 快轴以更慢的节奏跟随全局时间
        result[i] = trajs_[i].evaluate(t * scale);
        result[i].velocity     *= scale;       // 速度相应缩小
        result[i].acceleration *= scale * scale; // 加速度缩小平方倍
    }
}
```

### 核心思想

- **慢轴 (最长耗时)**: `scale = 1.0`，以最优速度运行
- **快轴 (耗时短)**: `scale < 1.0`，被"拉长"到与慢轴同步完成
- **结果**: 所有关节 **同时启动、同时到达**

---

## 六、SCurvePlanner 封装

文件: `robot_controller/src/SCurvPlan.cpp`

```cpp
class SCurvePlanner {
    // 规划3轴同步运动 (腕部3个关节)
    bool plan(cur, tgt, cur_vel) {
        ft_params[i] = {v_max, a_max, j_max, v0, v_min};
        ft_sync_.plan(ft_cur, ft_tgt, ft_params);  // 调用 SyncedTrajectory
    }

    // 查询某轴在某时刻的状态
    void evaluate(int idx, double t, double& pos, double& vel, double& acc) {
        auto pts = ft_sync_.evaluate(min(t, ft_sync_.sync_time()));
        pos = pts[idx].position;
        vel = abs(pts[idx].velocity);
        acc = abs(pts[idx].acceleration);
    }
};
```

### 当前使用状态

路径规划模块 (`SCurvePlanner`, `SyncedTrajectory`, `SCurveTrajectory`) **目前已编写完成但未在 ControllerNode 中实际使用**。当前的 `driveJoints` 采用的是 **"直接发目标 + 轮询到位"** 的简化方式。

未来如果启用，S曲线规划器将替换当前的简化方式，实现：
1. 规划完整轨迹 (位置、速度、加速度 vs 时间)
2. 以固定频率 (如100Hz) 沿轨迹插补位置指令
3. 更平滑的运动和更低的机械冲击
