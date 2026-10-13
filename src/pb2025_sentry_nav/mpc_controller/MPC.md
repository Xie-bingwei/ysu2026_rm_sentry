# MPC 路径跟踪控制器 —— 原理、设计、调参与踩坑记录

> 面向接手这个模块的同学。**读之前请先确认你会**：
> 1. 矩阵乘法、转置、分块矩阵
> 2. 二次型 $\|x\|_Q^2 = x^TQx$
> 3. 拉格朗日乘子 / KKT 条件（知道"带约束优化"是什么意思即可）
> 4. nav2 的 `nav2_core::Controller` 插件接口
>
> 不会也没关系，第 2 章是从零推的，每一步都给了代码对应。

---

## 目录

- [0. 任务说明与进度](#0-任务说明与进度)
- [1. 为什么用 MPC](#1-为什么用-mpc)
- [2. 原理推导](#2-原理推导)
- [3. 代码结构](#3-代码结构)
- [4. 设计思路与关键决策](#4-设计思路与关键决策)
- [5. 调参指南](#5-调参指南)
- [6. 我改了哪些东西](#6-我改了哪些东西)
- [7. 踩过的坑（重要，能省你几天）](#7-踩过的坑重要能省你几天)
- [8. 已知限制与 TODO](#8-已知限制与-todo)

---

## 0. 任务说明与进度

> 只列**大阶段**。细节见 [第 6 章](#6-我改了哪些东西) 与 [第 8 章](#8-已知限制与-todo)。
> 勾选框是 HTML 写法：在源文件里给 `<input>` 加/去 `checked` 即可切换。

**已完成**

- <input type="checkbox" checked> 阶段一 · 环境与依赖打通
- <input type="checkbox" checked> 阶段二 · 模型层 `MPC_Model`
- <input type="checkbox" checked> 阶段三 · 求解层 `MPC_Solver`
- <input type="checkbox" checked> 阶段四 · 控制层 `MPCController`
- <input type="checkbox" checked> 阶段五 · 接入 nav2 并跑通仿真
- <input type="checkbox" checked> 阶段六 · 调试与修复

**待办**

- <input type="checkbox"> 阶段七 · 碰撞检测
- <input type="checkbox"> 阶段八 · 实车配置同步与验证
- <input type="checkbox"> 阶段九 · 参数精调

---

## 1. 为什么用 MPC

先说清楚 MPC 相比 PID / Pure Pursuit **到底强在哪**，不然调参时会没有方向感。

| | Pure Pursuit / PID | MPC |
|---|---|---|
| 控制律 | `v = f(当前位置误差)`，只看当下 | 解一个**未来 N 步**的最优问题 |
| 前瞻能力 | 靠"前视距离"这一个标量粗糙地模拟 | 真正在优化里考虑未来轨迹 |
| 约束 | 只能事后截断（超了再 clamp） | **约束写在优化问题里**，解出来天然满足 |
| 多目标 | 靠调 PID 增益硬凑 | 代价函数里各项权重，物理意义清晰 |
| 代价 | 极低 | 每拍解一个 QP，**算力是主要成本** |

这台车是**舵轮全向底盘**，可以横向平移，所以这个 MPC 的模型选了最简单的**双积分器**

一句话总结本模块：

> **把"跟踪路径"写成一个带约束的二次规划（QP），每 50 ms 解一次，只取第一步控制量发出去。**

---

## 2. 原理推导

### 2.1 状态与控制量定义

$$
x = \begin{bmatrix} p_x \\ p_y \\ v_x \\ v_y \end{bmatrix} \in \mathbb{R}^4,
\qquad
u = \begin{bmatrix} a_x \\ a_y \end{bmatrix} \in \mathbb{R}^2
$$

- $p_x, p_y$：机器人在 **odom 系**下的位置
- $v_x, v_y$：机器人在 odom 系下的速度
- $a_x, a_y$：odom 系下的加速度，也就是**我们要优化的控制量**

> ⚠️ 注意这里**没有姿态角 $\psi$**。因为我们只控制平移，姿态由云台/底盘自己的逻辑负责（见 4.2）。

**连续时间模型**（双积分器）：

$$
\dot{x} = A_c x + B_c u,
\qquad
A_c = \begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0
\end{bmatrix},
\qquad
B_c = \begin{bmatrix}
0 & 0 \\ 0 & 0 \\ 1 & 0 \\ 0 & 1
\end{bmatrix}
$$

**位置对时间求导是速度，速度对时间求导是加速度**。

---

### 2.2 离散化（零阶保持 ZOH）

计算机只能算离散的，设采样周期 $T_s$（对应参数 `control_dt`）。用零阶保持（在一个周期内认为 $u$ 恒定）：

$$
x_{k+1} = A_d x_k + B_d u_k
$$

$$
A_d = \begin{bmatrix}
1 & 0 & T_s & 0 \\
0 & 1 & 0 & T_s \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix},
\qquad
B_d = \begin{bmatrix}
\frac{T_s^2}{2} & 0 \\[4pt]
0 & \frac{T_s^2}{2} \\[4pt]
T_s & 0 \\[4pt]
0 & T_s
\end{bmatrix}
$$

**怎么来的**：对 $p$ 积分两次，$p(t+T_s) = p(t) + T_s v(t) + \frac{T_s^2}{2}a$，$v(t+T_s) = v(t) + T_s a$。

对应 `mpc_model.cpp:16-24`：

```cpp
Ad_.setIdentity();
Ad_(0, 2) = dt_;
Ad_(1, 3) = dt_;

Bd_.setZero();
Bd_(0, 0) = 0.5 * dt_ * dt_;
Bd_(1, 1) = 0.5 * dt_ * dt_;
Bd_(2, 0) = dt_;
Bd_(3, 1) = dt_;
```

---

### 2.3 预测方程（把 N 步堆起来）

把未来 $N$ 步（预测步长 `horizon`）展开：

$$
\begin{aligned}
x_1 &= A_d x_0 + B_d u_0 \\
x_2 &= A_d^2 x_0 + A_d B_d u_0 + B_d u_1 \\
&\;\;\vdots \\
x_N &= A_d^N x_0 + A_d^{N-1}B_d u_0 + \cdots + B_d u_{N-1}
\end{aligned}
$$

写成矩阵形式。记堆叠向量：

$$
\bar{X} = \begin{bmatrix} x_1 \\ x_2 \\ \vdots \\ x_N \end{bmatrix} \in \mathbb{R}^{4N},
\qquad
\bar{U} = \begin{bmatrix} u_0 \\ u_1 \\ \vdots \\ u_{N-1} \end{bmatrix} \in \mathbb{R}^{2N}
$$

则：

$$
\bar{X} = \Phi\, x_0 + \Gamma\, \bar{U}
$$

$$
\Phi = \begin{bmatrix} A_d \\ A_d^2 \\ \vdots \\ A_d^{N} \end{bmatrix} \in \mathbb{R}^{4N \times 4},
\qquad
\Gamma_{ij} = \begin{cases}
A_d^{\,i-j}\, B_d, & j \le i \\[4pt]
0, & j > i
\end{cases}
\;\; (i,j = 0,1,\dots,N-1)
$$

$\Gamma$ 是**下三角分块矩阵**（当前时刻的控制量影响不了过去）。

代码里 $\Phi$ 的第 $i$ 块是 $A_d^{\,i+1}$（`mpc_model.cpp:31`：先 `Ak = Ak * Ad_` 再存），所以 $\bar X$ 装的是 $x_1 \dots x_N$，**不含 $x_0$**。这会影响后面终端代价的位置。

```cpp
// Phi: 第 i 块行 = Ad^(i+1)
AType Ak = AType::Identity();
for (int i = 0; i < N_; ++i) {
    Ak = Ak * Ad_;
    Phi_.block<4,4>(4*i, 0) = Ak;
}

// Gamma: block(i,j) = Ad^(i-j) * Bd   (j <= i)
for (int i = 0; i < N_; ++i) {
    AType Aij = AType::Identity();
    for (int j = i; j >= 0; --j) {
        Gamma_.block<4,2>(4*i, 2*j) = Aij * Bd_;
        Aij = Aij * Ad_;
    }
}
```

---

### 2.4 优化问题

**代价函数**：让预测轨迹贴住参考轨迹，同时别把加速度用太猛。

$$
J = \sum_{k=1}^{N-1}\|x_k - x_k^{ref}\|_Q^2
\;+\; \|x_N - x_N^{ref}\|_{Q_N}^2
\;+\; \sum_{k=0}^{N-1}\|u_k\|_R^2
$$

其中 $\|e\|_Q^2 \equiv e^T Q e$，权重矩阵：

$$
Q = \mathrm{diag}(q_{pos},\, q_{pos},\, q_{vel},\, q_{vel})
$$
$$
Q_N = \mathrm{diag}(q_{pos}^{term},\, q_{pos}^{term},\, q_{vel}^{term},\, q_{vel}^{term})
\quad
$$
$$
R = r_{acc} \cdot I_2
$$

堆叠形式（$\bar Q = \mathrm{blkdiag}(Q,\dots,Q,Q_N)$，$\bar R = \mathrm{blkdiag}(R,\dots,R)$）：

$$
J = \|\bar X - \bar X^{ref}\|_{\bar Q}^2 + \|\bar U\|_{\bar R}^2
$$

**约束**（MPC 核心优势）：

$$
|a_{x,k}| \le a_{max},\quad |a_{y,k}| \le a_{max}
\qquad\text{(加速度约束，参数 \texttt{a\_max})}
$$
$$
|v_{x,k}| \le v_{max},\quad |v_{y,k}| \le v_{max}
\qquad\text{(速度约束，参数 \texttt{v\_max})}
$$

注意：**速度约束是加在"预测出来的速度"上的**，不是加在输出上的。所以哪怕当前超速了，MPC 也会自己规划一条"减到合规"的轨迹。

---

### 2.5 化成标准 QP

把预测方程 $\bar X = \Phi x_0 + \Gamma \bar U$ 代进代价，令：

$$
E \equiv \Phi x_0 - \bar X^{ref} \quad (\text{当前状态推出来的"零输入轨迹"与参考的偏差})
$$

则 $\bar X - \bar X^{ref} = \Gamma\bar U + E$，展开：

$$
\begin{aligned}
J &= (\Gamma\bar U + E)^T \bar Q (\Gamma\bar U + E) + \bar U^T \bar R \bar U \\
  &= \bar U^T (\Gamma^T\bar Q\Gamma + \bar R)\bar U
   + 2E^T\bar Q\Gamma\,\bar U
   + \underbrace{E^T\bar Q E}_{\text{常数，不影响最优解}}
\end{aligned}
$$

所以：

$$
H = 2\left(\Gamma^T \bar Q \Gamma + \bar R\right) \in \mathbb{R}^{2N\times 2N}
\qquad
f = 2\,\Gamma^T \bar Q\, E \in \mathbb{R}^{2N}
$$

**约束也要堆成矩阵**。定义一个"抽速度"算子（从每个 4 维状态块里抽出 $v_x, v_y$）：

$$
S_v = \begin{bmatrix} 0&0&1&0 \\ 0&0&0&1 \end{bmatrix} \otimes I_N \in \mathbb{R}^{2N \times 4N}
$$

预测速度 $\bar V = S_v \bar X = S_v\Phi x_0 + S_v\Gamma \bar U$，于是 $|\bar V| \le v_{max}$ 变成：

$$
A_{ineq}\,\bar U \le b
$$

$$
A_{ineq} = \begin{bmatrix}
 I_{2N} \\[2pt]
 -I_{2N} \\[2pt]
 S_v\Gamma \\[2pt]
 -S_v\Gamma
\end{bmatrix} \in \mathbb{R}^{8N\times 2N}
\qquad
b = \begin{bmatrix}
 a_{max} \\[2pt]
 -a_{min} \\[2pt]
 v_{max} - S_v\Phi x_0 \\[2pt]
 -v_{min} + S_v\Phi x_0
\end{bmatrix} \in \mathbb{R}^{8N}
$$

- 前 $2N$ 行：$u \le a_{max}$
- 第 $2N \sim 4N$ 行：$-u \le -a_{min}$，即 $u \ge a_{min}$
- 后 $4N$ 行：速度上下界（**注意 $b$ 里带了 $-S_v\Phi x_0$，把已知项移到右边**）

**最终 QP**（标准形式）：

$$
\min_{\bar U}\;\; \frac{1}{2}\bar U^T H \bar U + f^T \bar U
\qquad \text{s.t.}\qquad A_{ineq}\bar U \le b
$$

---

### 2.6 用 OSQP 求解

OSQP 解的是：

$$
\min_{z}\;\; \frac{1}{2}z^T P z + q^T z
\qquad \text{s.t.}\qquad l \le A z \le u
$$

对照关系：

| OSQP | 我们 |
|---|---|
| $z$ | $\bar U$（决策变量，$2N$ 维） |
| $P$ | $H$（**必须只填上三角**） |
| $q$ | $f$ |
| $A$ | $A_{ineq}$ |
| $l$ | $-\infty$（只有上界） |
| $u$ | $b$ |

**几个实现要点**：

1. OSQP 用 **CSC 稀疏格式**（三个数组 `x/i/p`），要自己从稠密矩阵转
2. $P$ 只需上三角，$A$ 要全填
3. **`osqp_setup` 会深拷贝 $P$、$A$ 的数据**，所以 setup 之后本地的 `vector` 就可以不管了；**千万不要 `csc_spfree`**，否则会和 OSQP 内部重复释放
4. `P_csc.n = n; P_csc.m = n;`（$P$ 是方阵！曾经写成 `m = m` 导致过 bug）

**性能关键**：$H$、$A_{ineq}$、$\bar Q$、$\bar R$、$S_v$、$\Phi$、$\Gamma$ 全都是**常值矩阵**，所以在构造函数里算一次就够。每拍只需要：

```cpp
updateCost(x0, x_ref);                       // 只更新 f → osqp_update_lin_cost
computeBounds(x0, a_max, a_min, v_max, v_min); // 只更新 b → osqp_update_bounds
solve();                                      // 暖启动 + osqp_solve
```

**这是本模块能跑 20 Hz 的前提。** 如果每拍都 `osqp_setup`，必崩。

---

### 2.7 滚动时域（只取第一步）

解出 $\bar U^* = [u_0^*, u_1^*, \dots, u_{N-1}^*]^T$ 后，**只把 $u_0^*$ 用掉**，下一拍重新测状态、重新解。

这叫 **receding horizon control（滚动时域）**，是 MPC 的本质：每拍都基于最新状态重新规划，所以对扰动和模型误差有天然鲁棒性。

**本模块的输出形式**：

$$
v^{cmd}_{odom} = v_0 + T_s\, u_0^*
$$

即"当前速度 + 一步加速度"。然后转到机体系：

$$
\begin{bmatrix} v_x^{body} \\ v_y^{body} \end{bmatrix}
=
\begin{bmatrix}
 \cos\psi & \sin\psi \\
 -\sin\psi & \cos\psi
\end{bmatrix}
\begin{bmatrix} v_x^{cmd} \\ v_y^{cmd} \end{bmatrix}
$$

其中 $\psi$ 是 `pose.pose.orientation` 的 yaw。

---

## 3. 代码结构

```
mpc_controller/
├── include/mpc_controller/
│   ├── mpc_model.hpp          # 模型层：Ad, Bd, Phi, Gamma
│   ├── mpc_solver.hpp         # 求解层：H, Aineq, OSQP 封装
│   └── mpc_controller.hpp     # 控制层：nav2 插件
├── src/
│   ├── mpc_model.cpp
│   ├── mpc_solver.cpp
│   └── mpc_controller.cpp
└── mpc_controller.xml         # pluginlib 描述
```

**三层职责严格分离**：

| 层 | 类 | 职责 | 什么时候算 |
|---|---|---|---|
| 模型 | `MPC_Model` | 生成 $A_d, B_d, \Phi, \Gamma$ | **构造时一次** |
| 求解 | `MPC_Solver` | 生成 $H, A_{ineq}$，封装 OSQP | $H, A$ 构造时一次；$f, b$ 每拍 |
| 控制 | `MPCController` | nav2 接口、参考轨迹、坐标变换 | 每拍 |

**数据流**：

```
configure()  ─┬─ 读参数 (N, dt, v_des, a_max, v_max, Q, R)
              ├─ new MPC_Model(dt, N)      → Phi, Gamma
              └─ new MPC_Solver(model, w)  → H, Aineq, osqp_setup()  ★只此一次

setPlan(path) ──── 存 global_plan_，重置 s_prev_

computeVelocityCommands(pose, velocity, gc)  ← 每拍
   │
   ├─ x0 = [px, py, vx_odom, vy_odom]        (机体系速度旋到 odom)
   ├─ x_ref = buildReference(pose)           (map→odom + 弧长采样 + 切向)
   ├─ solver.updateCost(x0, x_ref)           (算 f, 写 OSQP)
   ├─ solver.computeBounds(x0, ...)          (算 b, 写 OSQP)
   ├─ solver.solve()                         (暖启动 + 迭代)
   ├─ u0 = solver.firstControl()             (只取第一步)
   └─ return v0 + dt*u0  →  转机体系
```

---

## 4. 设计思路与关键决策

设计原理

### 4.1 为什么用双积分器，不用阿克曼/差速模型

因为**这是麦克纳姆全向底盘**：可以朝任意方向瞬时平移，没有非完整约束。

- 阿克曼/差速模型 $[\dot x, \dot y, \dot\psi] = [v\cos\psi, v\sin\psi, \omega]$ 是**非线性**的，做 MPC 需要 NMPC（每拍解非线性规划，慢几十倍）
- 双积分器是**线性**的 → 代价是二次型、约束是线性的 → **标准 QP** → OSQP 几毫秒解完

**代价**：模型里没有姿态。姿态由上层（云台跟随 / `fake_vel_transform`）负责，我们只控制平移速度。这是一个**用模型精度换实时性**的刻意取舍。

### 4.2 为什么状态在 odom 系，而不是机体系

如果状态用机体系 $[p_x^b, p_y^b, v_x^b, v_y^b]$，那么机器人一转，状态就要跟着转，模型里必须引入 $\psi$ 和 $\omega$ → 又变非线性了。

用 odom 系（惯性系）：
- 模型是**常系数线性**的，$\Phi/\Gamma$ 只算一次
- 位置误差、速度参考都是**直线**，权重可以各向同性
- 缺点：转弯时必须实时做机体系↔odom 的坐标变换（代码里那两处 `cos/sin`）

**这是一个非常划算的取舍**，也是本设计能保持简单的核心。

### 4.3 为什么"稠密 QP + OSQP"，不用 iLQR / NMPC

- 精度够用：双积分器已经是真实运动学的合理近似
- 实时性：$N=40$ 时决策变量才 80 维，稠密 QP + OSQP 在几毫秒内
- 可调试：$H$、$A$ 都是显式矩阵，出问题可以打出来看

**没有用 CasADi / acados 这类自动微分框架**，是因为这题的规模不值得引入重型依赖。

### 4.4 参考轨迹怎么生成（⭐ 最关键的一节）

参考轨迹 $\bar X^{ref}$ 的质量**直接决定跟踪效果**，比调权重重要得多。

#### 步骤 1：路径从 map 转到 odom

`transformPlanToOdom()`：`tf_->lookupTransform(odom, plan.header.frame_id, ...)`

**只查一次 TF**（整条路径在同一帧同一时刻），然后对每个点 `tf2::doTransform`。这比逐点 `lookupTransform` 快几十倍。

同时按"到机器人的距离"裁剪掉太远的点（`keep_dist = 3 N T_s v_{des}`）。

#### 步骤 2：算累积弧长

$$
s_0 = 0,\qquad s_k = s_{k-1} + \|p_k - p_{k-1}\|
$$

#### 步骤 3：找当前在路径上的位置 $s_0$

取**离机器人最近的点**对应的弧长。

**不能全局搜索**！在 U 形 / 折返路径上，最近点会跳到另一条腿上，参考轨迹瞬间传送，车会猛冲。所以要加**单调前视窗口**：

```cpp
const double back_margin = 0.5;          // 允许往回找 0.5 m
size_t k_begin = 0;
while (k_begin + 1 < s.size() && s[k_begin] < s_prev_ - back_margin) ++k_begin;
// 只从 k_begin 往后搜最近点
s_prev_ = s0;                             // 记住，下一拍从这里继续
```

`setPlan()` 里重置 `s_prev_ = 0`（nav2 每次重规划，路径起点都是机器人当前位置，所以重置是对的）。

#### 步骤 4：按时间采样

第 $k$ 步的参考点取在路径上弧长为

$$
s_k = s_0 + k\, T_s\, v_{des}
$$

的位置（线性插值）。

#### 步骤 5：参考速度 = 路径切向 × 期望速度

**这是最容易写错、后果最严重的一步。**

❌ **错误写法**（踩过的坑）：

```cpp
const double yaw_i = tf2::getYaw(pi.pose.orientation);   // ← 大坑！
vx_ref = v_des * cos(yaw_i);
vy_ref = v_des * sin(yaw_i);
```

**nav2 的 `Path` 里 `orientation` 全是单位四元数 $(0,0,0,1)$**（规划器只填位置，不填朝向），`tf2::getYaw` 恒等于 0。结果就是**参考速度永远指向 odom 的 +X**，跟路径往哪走毫无关系。车会朝 +X 猛冲、冲出路外、再被位置误差拽回来 —— **这就是"超调严重 + 往回走很多"的根源**。

✅ **正确写法**：用相邻采样点差分求切向

$$
\hat t_k = \frac{p(s_k + \Delta s) - p(s_k - \Delta s)}{\|p(s_k+\Delta s) - p(s_k-\Delta s)\|}
$$

```cpp
geometry_msgs::msg::PoseStamped pa, pb;
samplePathAt(plan, s, std::max(0.0, si - ds), pa);
if (!samplePathAt(plan, s, si + ds, pb)) pb = plan.poses.back();

double tx = pb.pose.position.x - pa.pose.position.x;
double ty = pb.pose.position.y - pa.pose.position.y;
const double tn = std::hypot(tx, ty);
if (tn > 1e-6) { tx /= tn; ty /= tn; }
```

#### 步骤 6：终点刹车曲线

如果参考速度一路都是 $v_{des}$，到了终点车刹不住，会冲过头再倒回来。所以要限速：

$$
v_k^{ref} = \min\left(v_{des},\; \sqrt{2\, a_{max}\, (s_{total} - s_k)}\right)
$$

这是**匀减速运动公式** $v^2 = 2as$ 反解出来的"保证刹得住"的最大速度。$s_{total} - s_k$ 是剩余弧长。

超出路径末端后直接令 $v^{ref} = 0$。

#### 汇总

$$
x_k^{ref} = \begin{bmatrix} p(s_k) \\[2pt] v_k^{ref}\,\hat t_k \end{bmatrix}
\qquad (k = 1, \dots, N)
$$

### 4.5 为什么输出写成 $v_0 + T_s u_0^*$

这是标准的"**增量式输出**"，好处是：
- 计算结果和当前状态连续，不会突然跳变
- 天然利用了车辆的现有速度，起步/停止平滑

**但它是本模块最脆弱的地方**（详见第 7 章）：它**代数依赖 $v_0$**，只要速度反馈错一点，输出就跟着错。

### 4.6 与 nav2 的接口约定

| 事项 | 说明 |
|---|---|
| `pose` | 在 **costmap 的 global frame**（本项目配的是 `odom`） |
| `velocity` | nav2 认为在**机器人 base frame**；本项目实际给的是系统 odom 速度，两者在 `yaw≈0` 时等价 |
| 返回 | `TwistStamped`，**机体系** 的 `linear.x/linear.y`，`angular.z` 恒为 0 |
| `setSpeedLimit` | **未实现**（nav2 动态限速功能失效） |

---

## 5. 调参指南

### 5.1 参数表

配置位置：`pb2025_nav_bringup/config/simulation/nav2_params.yaml` → `controller_server.FollowPath`

| 参数 | 当前值 | 物理含义 | 调大 → | 调小 → |
|---|---|---|---|---|
| `horizon` | 40 | 预测步数 $N$，前瞻时间 $=N T_s$ | 看得更远，更"从容"，**但算力↑↑** | 更快，但短视、易过冲 |
| `control_dt` | 0.05 | 采样周期 $T_s$，应 $=1/f_{ctrl}$ | 预测跨度变大 | 预测跨度变小 |
| `v_des` | 1.5 | 参考速度（路径切向速度） | 跑得快，跟踪误差变大 | 稳但慢 |
| `a_max` | 2.0 | 加速度限幅 $\pm a_{max}$ | 更灵活，易超调 | 更柔和，响应慢 |
| `v_max` | 2.0 | 速度限幅 $\pm v_{max}$ | 超调修正更猛 | 更温和（**建议贴近 `v_des`**） |
| `q_pos` | 10.0 | 位置跟踪权重 | 贴线更紧，但更抖 | 更平滑，但会切内弯 |
| `q_vel` | 1.0 | 速度跟踪权重 | 速度更准，但位置可能滞后 | 位置优先 |
| `q_terminal_pos` | 100.0 | **终点**位置权重 | 收尾更准，末端更急 | 收尾松，末端平滑 |
| `q_terminal_vel` | 10.0 | **终点**速度权重 | 终点速度更接近参考 | 终点速度放松 |
| `r_acc` | 3.0 | 加速度代价 | **更平顺、更不超调**，但迟钝 | 更激进、易超调 |

### 5.2 权重怎么理解

代价函数本质是在"贴线"和"省力"之间找平衡：

$$
J = \underbrace{q_{pos}\|p - p^{ref}\|^2}_{\text{贴线}} + \underbrace{q_{vel}\|v - v^{ref}\|^2}_{\text{速度匹配}} + \underbrace{r_{acc}\|a\|^2}_{\text{省力}}
$$

**只看相对比例，不看绝对值**（整体放大缩小不改变最优解）。当前配置的比例是：

$$
q_{pos} : q_{vel} : r_{acc} = 10 : 1 : 3
$$

经验起点：**$q_{pos} : q_{vel} : r_{acc} \approx 10 : 1 : 1 \sim 10 : 1 : 5$**。

终端权重 $Q_N$ 一般是 $Q$ 的 5~10 倍，用来让末端收敛得更准。

### 5.3 调参顺序（**别跳步**）

```
① 慢速跑通
   v_des = 1.0,  v_max = 1.2,  r_acc = 3.0,  horizon = 20
   目标：能贴住线，不超调，不振荡

② 提 r_acc 消超调
   还超调 → r_acc 再加大（3 → 5 → 8）
   反应太肉 → r_acc 往回降

③ 压 v_max
   v_max 建议 = v_des ~ 1.3 × v_des
   给太大 = 超调时能用双倍速度往回冲

④ 调 q_pos / q_vel
   切内弯 → q_pos 加大
   抖 → q_pos 减小 或 r_acc 加大

⑤ 最后放 horizon
   算力够（`missed its desired rate` 不刷）再往上加
   horizon 大 + r_acc 小 = 灾难（看得远、又敢猛打）

⑥ 最后提 v_des
   每提 0.2 跑一圈，观察超调量
```

### 5.4 症状 → 病因速查

| 症状 | 最可能的原因 | 先查什么 |
|---|---|---|
| 反复超调、冲过路径再回来 | **参考速度方向错** | 打日志看 `ref_v` 是否随路径方向变化 |
| 横向极慢 / 几乎不动 | **`min_y_velocity_threshold`** | 见 7.2 |
| 直线跑得挺好、拐弯就飞出去 | `q_pos` 太小 或 `horizon` 太大 | 先降 `horizon` 试 |
| 抵近终点时来回画龙 | 没有刹车曲线 | 检查 `sqrt(2*a*remain)` 那段 |
| 车抖 / 高频抖 | `r_acc` 太小 或 `q_pos` 太大 | `r_acc` ×2 |
| `Control loop missed its desired rate` | `horizon` 太大 / 求解太慢 | 降 `horizon`，查 `lastIterations()` |
| 起步/停车突兀 | `r_acc` 太小 | 加大 `r_acc` |
| 在 U 形路径上突然反向冲 | 没有 `s0` 单调窗口 | 看 `s_prev_` 逻辑 |

### 5.5 调参必须有的两个观测手段

**① 打印参考速度**（临时加，验证用）：

```cpp
RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
    "s0=%.2f  ref_v=(%.2f, %.2f)  vref=%.2f",
    s0, x_ref[4], x_ref[5], std::hypot(x_ref[4], x_ref[5]));
```

**期望**：车沿 +Y 走时 `ref_v ≈ (0, +v_des)`；沿 -X 走时 `≈ (-v_des, 0)`。

**② 打印求解迭代次数**：

```cpp
RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
    "QP iters=%d status=%d", mpc_solver_->lastIterations(), mpc_solver_->lastStatus());
```

`status` 必须是 1（`OSQP_SOLVED`）或 2（`OSQP_SOLVED_INACCURATE`）。**如果是 3 以上（不可行/不收敛），调权重是没用的，要先查约束和规模。**

---

## 6 新增内容

- 整个 `mpc_controller` 包（`mpc_model` / `mpc_solver` / `mpc_controller` 三层）
- `nav2_params.yaml` 的 `controller_server.FollowPath` 段：`plugin` 从 `pb_omni_pid_pursuit_controller::OmniPidPursuitController` 换成 `mpc_controller::MPCController`，并加上 MPC 参数块

### 6.1 相对于第一版实现修掉的问题

| # | 问题 | 位置 | 后果 |
|---|---|---|---|
| 1 | `computeBounds` 返回局部变量的引用 | `mpc_solver` | UB，可能崩 |
| 2 | `for(int i=0; i<=N_; ++i)` 越界（3 处） | 多处 | 数组越界 |
| 3 | `Sv_(i+1, 4i+3)` 索引错 | `mpc_solver` 构造 | 速度约束抽错分量 |
| 4 | `P_csc.m = m` 应为 `n` | `setupOSQP` | $P$ 维度错，setup 失败 |
| 5 | `toFullCSC` 把值 push 到 `i` 而不是 `x` | CSC 转换 | 矩阵全错 |
| 6 | `solve()` 缺 `return true` / 状态判断用错变量 | `solve` | 求解永远报失败 |
| 7 | `computeVelocityCommands` 缺第三个参数 | 接口 | pluginlib 加载失败 |
| 8 | `samplePathAt` 缺 `MPCController::` 和 `const` | 定义 | 链接失败 |
| 9 | `tf_->transform(...)` 当 bool 用 | 坐标变换 | Humble 里返回引用，抛异常 |
| 10 | **参考速度用 `pose.orientation` 求切向** | `buildReference` | **超调 + 反向冲**（见 7.1） |
| 11 | 没有终点刹车曲线 | `buildReference` | 冲过终点 |
| 12 | `s0` 全局最近点，无单调约束 | `buildReference` | U 形路径上跳变 |

### 6.2 顺带修掉的配置问题

- RViz 的 `robot_description` 显示 QoS：`Volatile` → `Transient Local`（否则 RViz 报"无法加载机器人模型"）

---

## 7. 踩过的坑

### 7.1 参考速度方向：nav2 的 `Path.orientation` 是单位四元数

**现象**：车不跟路径走，反复超调、往回冲。

**原因**：nav2 的规划器只填 `position`，`orientation` 是 $(0,0,0,1)$。`tf2::getYaw` 对它恒返回 0。

**验证方法**：

```bash
ros2 topic echo /red_standard_robot1/plan --once | grep -A4 orientation
# 会看到 x:0 y:0 z:0 w:1  ← 全是单位四元数
```

**修复**：用相邻采样点差分求切向（见 4.4 步骤 5）。

### 7.2 ⭐ nav2 的 `min_y_velocity_threshold` 会把横向速度"清零"

**这是全向底盘最容易中的坑，而且从现象上完全看不出是 nav2 干的。**

`nav2_params.yaml` 的 `controller_server` 段：

```yaml
min_x_velocity_threshold: 0.001
min_y_velocity_threshold: 0.5       # ← 差速车模板的默认值，全向底盘会出事
min_theta_velocity_threshold: 0.001
```

nav2 官方 `controller_server.cpp` 里：

```cpp
void ControllerServer::computeAndPublishVelocity()
{
  ...
  // ★ 阈值作用在【输入】：里程计速度被逐分量清零后才传给插件
  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());

  cmd_vel_2d = controllers_[current_controller_]->computeVelocityCommands(
      pose,
      nav_2d_utils::twist2Dto3D(twist),      // ← 插件拿到的是被清零后的速度
      goal_checkers_[current_goal_checker_].get());
  ...
}

double getThresholdedVelocity(double velocity, double threshold)   // controller_server.hpp:186
{
  return (std::abs(velocity) > threshold) ? velocity : 0.0;
}
```

**为什么对差速车没问题**：差速车横向速度恒为 0，这个阈值就是用来滤掉横向噪声的。nav2 自己的默认值其实是 `0.0001`，`0.5` 是从 nav2 的差速车参考配置抄来的。

**为什么对 MPC 致命**：本模块输出是 $v^{cmd} = v_0 + T_s u_0^*$，**代数依赖 $v_0$**。$v_0.y$ 被清零后：

$$
|v^{cmd}_y| = |0 + T_s u_{0,y}^*| \le T_s\, a_{max} = 0.05 \times 2.0 = 0.1 \ \text{m/s}
$$

永远够不到 0.5 → 下一次又被清零 → **横向速度被永久锁死在 0.1 m/s 以下**。

**死循环**：

```
真实 |vy| 很小(<0.5) → nav2 清零 → x0[3]=0 → 输出 vy ≤ 0.1 → 真实 |vy| 仍然很小 → ...
```

**为什么换成 MPC 才出现**：之前的 PID 控制器**不用这个速度算命令**：

```cpp
// PID：velocity 只用来算前视距离，而且是标量模长
double lookahead_dist = getLookAheadDistance(velocity);   // hypot(vx, vy)，y 清零只影响一点
// 速度命令完全由「到 carrot 的距离」经 PID 产生
auto lin_vel = move_pid_->calculate(lin_dist, 0);
cmd_vel.twist.linear.x = lin_vel * cos(theta_dist);
cmd_vel.twist.linear.y = lin_vel * sin(theta_dist);      // 与速度反馈无关
```

**修复**：

```yaml
min_y_velocity_threshold: 0.001    # 仿真和实车两处都要改
```

**教训**：**换控制器时必须重新审视 nav2 的"差速车假设"参数。** 除了这个阈值，还有 `robot_radius`、`min_theta_velocity_threshold`、`deadband_velocity` 等。

### 7.3 OSQP 的几个坑

| 坑 | 说明 |
|---|---|
| `P` 必须**上三角** CSC | 只填 $r \le c$ 的部分，否则解不对 |
| `P_csc.n = n; P_csc.m = n;` | $P$ 是 $n\times n$ 方阵（曾写成 `m=m`） |
| **不要 `csc_spfree`** | `osqp_setup` 会**深拷贝**数据；本地 vector 由 `std::vector` 自己管；手动 free 会双重释放 |
| `polish` 不是 `polishing` | 0.6.2 的字段名 |
| 参数名 `max_iter` | 默认值偏小，$N$ 大时要显式设 |
| 单位矩阵 QP 也要给下界 | 只有上界时 `l` 必须显式填 `-OSQP_INFTY`，不能留空 |
| **别每拍 `osqp_setup`** | 只 setup 一次，之后用 `osqp_update_lin_cost` / `osqp_update_bounds` |

### 7.4 控制周期必须对应

`control_dt` 写死在 $\Phi/\Gamma$ 里，但真实控制周期由 `controller_frequency` 和**实际算力**决定。

如果实际只有 17.5 Hz 而 `control_dt = 0.05`（=20 Hz），模型和实物差 14%，而且周期还在抖 —— **这本身就在制造振荡和超调**。

```bash
# 跑起来后看有没有这句
ros2 launch ... 2>&1 | grep "missed its desired rate"
```

有就说明算力不够，要么降 `horizon`，要么把 `controller_frequency` 降到真能跑到的值。

### 7.5 坐标系链路

这台车的 `cmd_vel` 要经过一串旋转：

```
nav2 controller_server
   └─ cmd_vel_controller ─→ velocity_smoother ─→ cmd_vel_nav2_result
                                                        │
                          fake_vel_transform（云台角补偿）
                                                        ↓
                                                      cmd_vel
                                                        │
                    chassis_controller（云台→底盘坐标变换）
                                                        ↓
                                              IgnChassisActuator → Gazebo
```

`fake_vel_transform` 会**给 `angular.z` 加上 `spin_speed_`（默认 3.14）**，也就是底盘会持续自转；同时它发 `gimbal_yaw → gimbal_yaw_fake` 这个 TF，让 nav2 看到的"机器人基座"始终朝向固定方向。

**调 MPC 时如果发现车在"边转边走"，那不是 bug，是这个设计。**

---

## 8. 已知限制与 TODO

### 已实现

- ✅ 双积分器 MPC + 稠密 QP + OSQP
- ✅ 加速度 / 速度双约束
- ✅ 弧长参数化参考 + 差分切向 + 刹车曲线
- ✅ `s0` 单调前视窗口

### 未实现

1. **碰撞检测** ❌
   当前 `computeVelocityCommands` **完全不看 local costmap**，MPC 只跟踪路径。
   - 后续会考虑将障碍物代价加入建模或使用北极熊的方式
   - 可用的预测轨迹：$\bar X = \Phi x_0 + \Gamma \bar U^*$。

2. **`setSpeedLimit` 未实现** ❌
   - nav2 的动态限速功能目前是失效的。

3. **`computeVelocityCommands` 里的空指针日志** 

4. **输出形式对 $v_0$ 的强依赖** 
   `v = v_0 + T_s u_0` 假定速度反馈可信。而 `sensor_scan_generation` 发的 odom twist 是**裸差分、无滤波**（而且 `dt` 用墙钟 `steady_clock`，位置却来自仿真时间）。思路：
   - 给 odom twist 加低通，或
   - 把输出改成"参考前馈 + 有界位置修正"

5. **可选的性能优化**
   - $\bar Q$、$\bar R$ 是块对角，可以避免稠密乘法
   - 试 `st.scaling = 0` / 关掉 `polish` 看迭代是否还能收敛

---

## 附录 A：公式速查

$$
\begin{aligned}
&\text{模型} && x_{k+1} = A_d x_k + B_d u_k \\
&\text{预测} && \bar X = \Phi x_0 + \Gamma \bar U \\
&\text{代价} && J = \|\bar X - \bar X^{ref}\|_{\bar Q}^2 + \|\bar U\|_{\bar R}^2 \\
&\text{QP} && H = 2(\Gamma^T\bar Q\Gamma + \bar R),\quad f = 2\Gamma^T\bar Q(\Phi x_0 - \bar X^{ref}) \\
&\text{约束} && A_{ineq}\bar U \le b,\quad A_{ineq}\in\mathbb{R}^{8N\times 2N} \\
&\text{输出} && v^{cmd}_{odom} = v_0 + T_s u_0^* \\
&\text{参考} && s_k = s_0 + kT_sv_{des},\quad v^{ref}_k = \min\!\left(v_{des},\sqrt{2a_{max}(s_{tot}-s_k)}\right)
\end{aligned}
$$

## 附录 B：规模速查（$N=40$）

| 量 | 维度 |
|---|---|
| $x_0$ | 4 |
| $\bar U$（决策变量，OSQP 的 $n$） | $2N = 80$ |
| $\bar X$ | $4N = 160$ |
| $H$ | $80\times 80$ |
| $A_{ineq}$（OSQP 的 $m$） | $8N \times 2N = 320\times 80$ |
| $\Phi$ | $160\times 4$ |
| $\Gamma$ | $160\times 80$ |
| 前瞻时间 | $N T_s = 2.0$ s |
| 前瞻距离 | $N T_s v_{des} = 3.0$ m |

## 附录 C：相关文件索引

| 文件 | 作用 |
|---|---|
| `mpc_controller/src/mpc_model.cpp` | $A_d, B_d, \Phi, \Gamma$ |
| `mpc_controller/src/mpc_solver.cpp` | $H, A_{ineq}$、OSQP 封装 |
| `mpc_controller/src/mpc_controller.cpp` | nav2 插件、参考轨迹、坐标变换 |
| `pb2025_nav_bringup/config/simulation/nav2_params.yaml` | `controller_server.FollowPath` 参数 |
| `sensor_scan_generation/src/sensor_scan_generation.cpp` | `/odometry` 的来源（MPC 的 $v_0$） |
| `fake_vel_transform/src/fake_vel_transform.cpp` | 云台角补偿、底盘自转 |
| `nav2_controller/controller_server.cpp`（系统） | `min_y_velocity_threshold` 的实现 |
