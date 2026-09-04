# 正逆运动学

## 一、机械臂 DH 参数 (URDF 模型)

文件: `dm2c-controller/src/left_arm_ik.cpp`

### 8自由度模型

| 索引 | 关节名 | 类型 | 运动 | URDF范围 |
|------|--------|------|------|----------|
| q[0] | l_post_Joint (升降柱) | 直线 | Z方向移动 | [0, 0.95] m |
| q[1] | l_shoulder_Joint (肩部) | 旋转 | 绕Z轴旋转 | [0, 1.57] rad |
| q[2] | l_arm_JointA (伸展A) | 直线 | Z方向伸缩 | [0, 0.12] m |
| q[3] | l_arm_JointB (伸展B) | 直线 | Z方向伸缩 | [0, 0.13] m |
| q[4] | l_arm_JointC (伸展C) | 直线 | Z方向伸缩 | [0, 0.11] m |
| q[5] | l_wrist_p_Joint (腕俯仰) | 旋转 | 绕Z轴旋转 | [-π, π] rad |
| q[6] | l_wrist_y_Joint (腕偏航) | 旋转 | 绕Z轴旋转 | [-π/2, π/2] rad |
| q[7] | l_wrist_r_Joint (腕翻滚) | 旋转 | 绕Z轴旋转 | [-π, π] rad |

### 连杆变换矩阵 (URDF 20260623)

```cpp
// 基座 → 升降柱起点
T_O0 = Frame(I, Vector(-0.1275, -0.1065, 0.34054))
// 升降柱终点 → 肩部起点
T_O1 = Frame(I, Vector(0, 0.119, 0.0575))
// 肩部终点 → 伸展A起点 (绕X轴旋转 -90°)
T_O2A = Frame(RPY(-π/2, 0, 0), Vector(0.0002, 0.254, 0.0328))
// 伸展A终点 → 伸展B起点
T_O2B = Frame(I, Vector(0, 0, 0.015))
// 伸展B终点 → 伸展C起点
T_O2C = Frame(I, Vector(0, 0, 0.011))
// 伸展终点 → 腕部Pitch
T_O3 = Frame(RPY(-π/2, 0, 0), Vector(-0.0002, 0.0368, 0.057))
// 腕Pitch → 腕Yaw
T_O4 = Frame(RPY(π/2, 0, 0), Vector(0, 0.027, 0.0565))
// 腕Yaw → 腕Roll
T_O5 = Frame(RPY(-π/2, 0, 0), Vector(0, 0.042, 0.027))
```

### 关节运动变换

```cpp
static Frame joint_motion(int idx, double qi) {
    // 直线关节 (0=升降柱, 2/3/4=伸展A/B/C): Z方向移动
    if (idx == 0 || idx == 2 || idx == 3 || idx == 4)
        return Frame(Vector(0, 0, qi));

    // 旋转关节 (1=肩部, 5/6/7=腕部): 绕Z轴旋转
    return Frame(Rotation::RotZ(qi));
}
```

---

## 二、正运动学 (FK)

### 链式乘法

```cpp
Frame fk(const std::array<double, 8>& q) {
    Frame T = Frame::Identity();
    T = T * T_O0  * joint_motion(0, q[0]);  // 基座→升降柱
    T = T * T_O1  * joint_motion(1, q[1]);  // 升降柱→肩部
    T = T * T_O2A * joint_motion(2, q[2]);  // 肩部→伸展A
    T = T * T_O2B * joint_motion(3, q[3]);  // 伸展A→伸展B
    T = T * T_O2C * joint_motion(4, q[4]);  // 伸展B→伸展C
    T = T * T_O3  * joint_motion(5, q[5]);  // 伸展→腕Pitch
    T = T * T_O4  * joint_motion(6, q[6]);  // 腕Pitch→腕Yaw
    T = T * T_O5  * joint_motion(7, q[7]);  // 腕Yaw→腕Roll (TCP)
    return T;
}
```

### 分段FK

```cpp
// FK到腕部基点 (前5个关节)
Frame fk_arm(q)  = T_O0·J0·T_O1·J1·T_O2A·J2·T_O2B·J3·T_O2C·J4·T_O3

// FK腕部 (后3个关节)
Frame fk_wrist(qw) = J5·T_O4·J6·T_O5·J7
```

---

## 三、逆运动学 (IK) — 解析几何法

文件: `dm2c-controller/src/left_arm_ik.cpp` 第145行

### 核心思路：位置-姿态解耦

机械臂6DOF存在腕部偏置，采用 **迭代法** 解耦：
1. 假设腕部姿态 → 反算腕部基点位置
2. 从基点位置 → 反算大臂参数 (升降柱+肩部+伸展)
3. 从大臂姿态 + 目标姿态 → 反算腕部关节
4. 迭代收敛

### 步骤1：从目标TCP反算腕部基点

```cpp
static Frame wrist_base_from_target(P_tcp, R_tcp, q3, q4, q5) {
    Frame T_tcp(R_tcp, P_tcp);                      // 目标TCP位姿
    // 逆推: TCP → 腕Roll → 腕Yaw → 腕Pitch → 腕基点
    Frame T_inv = J5⁻¹ · T_O5⁻¹ · J4⁻¹ · T_O4⁻¹ · J3⁻¹;
    return T_tcp * T_inv;                            // 腕基点在世界系中的位姿
}
```

### 步骤2：从腕基点反算大臂 (升降柱 + 肩部 + 伸展)

```cpp
static void solve_arm_from_wp(P_wp, d0, t1, d_total) {
    // 升降柱: 从Z坐标直接反算
    d0 = clamp(P_wp.z - 0.39404, 0, 0.95);           // 单位: 米

    // 肩部: 几何关系 (如图)
    //   P_wp.x = -0.1275 - (d_total + 0.337)·sin(t1)
    //   P_wp.y =  0.0125 + (d_total + 0.337)·cos(t1)
    double A = P_wp.x + 0.1275;
    double B = P_wp.y - 0.0125;
    double L = sqrt(A² + B²);                         // 水平投影距离
    d_total = clamp(L - 0.337, 0, 0.36);             // 伸展总长 (0.12+0.13+0.11=0.36)
    t1 = clamp(atan2(-A, B), 0, 1.57);              // 肩部角度
}
```

### 步骤3：从目标姿态反算腕部

```cpp
// 腕部旋转矩阵公式:
// R_wrist = [[c3c4c5-s3s5, -c3c4s5-s3c5, -c3s4],
//            [s3c4c5+c3s5, -s3c4s5+c3c5, -s3s4],
//            [s4c5,        -s4s5,         c4]]

static array<double,3> solve_wrist_from_R(R) {
    // q4 (腕偏航) = acos(R[2,2])
    double c4 = clamp(R(2,2), -1, 1);
    double q4 = acos(c4);
    double s4 = sin(q4);

    // q3 (腕俯仰) = atan2(-R[1,2], -R[0,2])
    // q5 (腕翻滚) = atan2(-R[2,1],  R[2,0])
    if (|s4| > 1e-8) {
        q3 = atan2(-R(1,2), -R(0,2));
        q5 = atan2(-R(2,1),  R(2,0));
    } else {
        // 奇异: s4 ≈ 0, q3和q5有无穷多解
        q3 = 0;
        q5 = (c4>0) ? atan2(-R(1,0), -R(0,0)) : atan2(R(1,0), R(0,0));
    }
    return {clamp(q3), clamp(q4), clamp(q5)};
}
```

还考虑了 **腕部双解**（`solve_wrist_from_R_alt`）:
```cpp
// 另一种解: 翻转腕部
q3' = q3 + π,  q4' = -q4,  q5' = q5 + π
```
每次迭代同时评估两种解，选误差更小的。

### 步骤4：迭代收敛 + 双解选择

```cpp
IKResult solve_ik(target_pos, target_rot, q_init) {
    double q3=q_init[5], q4=q_init[6], q5=q_init[7];

    for (int iter = 0; iter < 5; ++iter) {
        // Step 1: 反算腕基点
        T_wp = wrist_base_from_target(target_pos, target_rot, q3,q4,q5);

        // Step 2: 反算大臂
        solve_arm_from_wp(T_wp.p, d0, t1, d_total);

        // Step 3: 反算腕部 (双解)
        R_arm = arm_wp_rotation(t1);
        R_wrist_target = R_arm⁻¹ * target_rot;
        wq_a = solve_wrist_from_R(R_wrist_target);     // 主解
        wq_b = solve_wrist_from_R_alt(R_wrist_target); // 替代解

        // Step 4: 双向评估, 选误差小者
        fk(q_a) → pos_err_a, rot_err_a
        fk(q_b) → pos_err_b, rot_err_b
        if (pos_err_a + 0.1*rot_err_a <= pos_err_b + 0.1*rot_err_b)
            选 a
        else
            选 b

        // 收敛判断: pos_err < 1mm 且 rot_err < 0.05 rad (≈2.86°)
        if (pos_err < 0.01 && rot_err < 0.05)
            return SUCCESS;
    }
}
```

### 步骤5：伸展三段分配

```cpp
static void split_arm(double d_total, double& dA, double& dB, double& dC) {
    // 优先填满 A 段 (max 120mm)
    dA = clamp(d_total, 0, 0.12);
    d_total -= dA;
    // 再填 B 段 (max 130mm)
    dB = clamp(d_total, 0, 0.13);
    d_total -= dB;
    // 剩余给 C 段 (max 110mm)
    dC = clamp(d_total, 0, 0.11);
}
```

---

## 四、IK结果到电机指令的映射

```cpp
std::vector<double> mapIkToJoints(const std::array<double, 8>& q8) {
    // q8[0] (米) → 升降柱 (毫米)
    j[Lift]     = q8[0] * 1000;                     // m → mm

    // q8[1] (弧度) → 肩部 (度)
    j[Shoulder] = q8[1] * 180/π;                    // rad → °

    // q8[2+3+4] (米) → 伸展 (度) via 传动系数
    j[Extend]   = (q8[2] + q8[3] + q8[4]) * 1000;   // m → ° (EXT_M_TO_DEG=1000)

    // q8[5/6/7] (弧度) → 腕部 (度)
    j[Wrist1]   = q8[5] * 180/π;
    j[Wrist2]   = q8[6] * 180/π;
    j[Wrist3]   = q8[7] * 180/π;
}
```

---

## 五、TCP偏移

```cpp
// 工具中心点偏移 (腕Roll末端 → 工具末端)
left_tool_offset_ = KDL::Frame(I, Vector(0.0, 0.0, 0.32));  // Z方向前伸 32cm

// 腕TPC → 工具TCP
left_tool_ = left_tcp_ * left_tool_offset_;
```

### 两种参考系

| frame_id | 含义 | 变换 |
|----------|------|------|
| `base_link` | 基坐标系（绝对） | `target = original_pose * tool_offset⁻¹` |
| `l_wrist_r_Link` | 腕部坐标系（相对） | `target = left_tcp * tool_offset * original_pose * tool_offset⁻¹` |

---

## 六、姿态表示

```cpp
// 四元数 → RPY (Roll-Pitch-Yaw)
static void quatToRPY(x, y, z, w, &roll, &pitch, &yaw) {
    roll  = atan2(2(wx + yz), 1 - 2(x² + y²));
    pitch = asin(2(wy - zx));         // 限制在 [-π/2, π/2]
    yaw   = atan2(2(wz + xy), 1 - 2(y² + z²));
}
```
