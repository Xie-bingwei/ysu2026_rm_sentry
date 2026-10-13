# OSQP 使用说明（只覆盖本项目用到的部分）

> 配套文档：本目录 `README.md`（MPC 原理与调参）
> OSQP 版本：**0.6.2**（`ros-humble-osqp-vendor`），头文件在 `/opt/ros/humble/include/osqp/`
> 本文只讲 `mpc_solver.cpp` / `mpc_solver.hpp` 里实际用到的东西，不做完整 API 手册。

---

## 1. OSQP 解的是什么问题

$$
\min_{x}\;\; \frac{1}{2}x^T P x + q^T x
\qquad \text{s.t.}\qquad l \;\le\; A x \;\le\; u
$$

- $x \in \mathbb{R}^n$ 决策变量，$P$ 必须**半正定**
- $A \in \mathbb{R}^{m\times n}$ 约束矩阵
- $l, u$ 逐分量的下/上界；**要单边约束就把另一边设成 $\pm\infty$**

**本项目的对照**（`mpc_solver.hpp:86` 有注释）：

| OSQP | 我们 | 维度（$N=40$） |
|---|---|---|
| $x$ | $\bar U$（$N$ 步的 $[a_x, a_y]$） | $n = 2N = 80$ |
| $P$ | $H = 2(\Gamma^T\bar Q\Gamma + \bar R)$ | $80\times80$ |
| $q$ | $f = 2\Gamma^T\bar Q E$ | 80 |
| $A$ | $A_{ineq}$ | $320\times80$ |
| $l$ | 全 $-\infty$ | 320 |
| $u$ | $b$（加速度/速度上界） | 320 |

---

## 2. 数据怎么喂进去：CSC 稀疏格式

OSQP 只吃 **CSC（Compressed Sparse Column，列主序）**。定义在 `types.h:21`：

```c
typedef struct {
  c_int    nzmax;  ///< 非零元素个数上限（= x/i 数组长度）
  c_int    m;      ///< 行数
  c_int    n;      ///< 列数
  c_int   *p;      ///< 列指针，长度 n+1。p[j] = 第 j 列第一个非零在 x/i 里的下标
  c_int   *i;      ///< 行下标，长度 nzmax，从 0 开始
  c_float *x;      ///< 数值，长度 nzmax
  c_int    nz;     ///< 三元组格式的条目数；**CSC 必须填 -1**
} csc;
```

### 三个必须记住的点

1. **列主序**：按列遍历，同一列里行号任意顺序都行。
2. **`P` 只填上三角**（$r \le c$）。OSQP 文档明确说它只看上三角；填了整个矩阵会算错。
3. **`p` 长度是 `n+1`**，且 `p[n]` 必须等于非零总数。

### 两个转换函数（`mpc_solver.cpp:56` / `:72`）

```cpp
// 只取上三角 -> 给 P
static void toUpperCSC(const Eigen::MatrixXd &M,
                       std::vector<c_float> &x, std::vector<c_int> &i, std::vector<c_int> &p)
{
    const int n = static_cast<int>(M.cols());
    x.clear(); i.clear(); p.assign(n + 1, 0);
    for (int j = 0; j < n; ++j) {
        p[j] = static_cast<int>(i.size());   // 记住这一列从哪开始
        for (int r = 0; r <= j; ++r) {       // r <= j 才是上三角
            i.push_back(r);
            x.push_back(M(r, j));
        }
    }
    p[n] = static_cast<int>(i.size());       // 收尾
}

// 全填 -> 给 A
static void toFullCSC(const Eigen::MatrixXd &M, ...)   // 内层改成 for (r = 0; r < m; ++r)
```

> ⚠️ 这里是把**稠密矩阵按原样展开成 CSC**（包括零元素），也就是"稀疏格式装稠密数据"。
> 对这个规模（$80\times80$、$320\times80$）完全没问题；如果以后 $N$ 上百，应该改成真稀疏（跳过零元素）。

### 组装 `OSQPData`（`types.h:126`）

```c
typedef struct {
  c_int    n;   ///< 变量数
  c_int    m;   ///< 约束数
  csc     *P;   ///< n x n，上三角
  csc     *A;   ///< m x n
  c_float *q;   ///< 长度 n
  c_float *l;   ///< 长度 m
  c_float *u;   ///< 长度 m
} OSQPData;
```

具体是这样填的（`mpc_solver.cpp:103-119`）：

```cpp
csc P_csc;
P_csc.nzmax = static_cast<c_int>(P_x_.size());
P_csc.n = n; P_csc.m = n;          // ★ P 是方阵！两边都是 n
P_csc.p = P_p_.data(); P_csc.i = P_i_.data(); P_csc.x = P_x_.data();
P_csc.nz = -1;                      // ★ CSC 固定 -1

csc A_csc;
A_csc.nzmax = static_cast<c_int>(A_x_.size());
A_csc.n = n; A_csc.m = m;          // A 是 m x n
A_csc.p = A_p_.data(); A_csc.i = A_i_.data(); A_csc.x = A_x_.data();
A_csc.nz = -1;

OSQPData data;
data.n = n; data.m = m;
data.P = &P_csc; data.A = &A_csc;
data.q = q_.data(); data.u = u_.data(); data.l = l_.data();
```

> **生命周期**：`data` 里全是裸指针，指向自己的 `std::vector`。
> 这些 vector 必须在 `osqp_setup` 调用期间活着（成员变量，天然满足）。

### 类型说明

本项目 OSQP 编译时定义了 `DLONG`（见 `osqp_configure.h:32`），所以：

| 类型 | 实际是 | 注意 |
|---|---|---|
| `c_int` | **`long long`** | 打印用 `%lld`，别用 `%d` |
| `c_float` | **`double`** | 可以直接和 `double` 混用 |

---

## 3. 六个 API

| 函数 | 何时调用 | 我们的位置 |
|---|---|---|
| `osqp_set_default_settings(&st)` | 调 setup 前，先把 settings 填默认值 | `mpc_solver.cpp:123` |
| `osqp_setup(&work, &data, &st)` | **整个生命周期只调一次** | `:132` |
| `osqp_update_lin_cost(work, q)` | 每拍，更新 $f$ | `:145` |
| `osqp_update_bounds(work, l, u)` | 每拍，更新 $b$ | `:169` |
| `osqp_warm_start(work, x, y)` | 每拍求解**之前** | `:175` |
| `osqp_solve(work)` | 每拍 | `:177` |
| `osqp_cleanup(work)` | 析构时 | `:49` |

全部返回 `c_int`，**0 = 成功**，非 0 = 出错。

### 完整调用序列

```cpp
// ── 构造时（一次）──
OSQPSettings st;
osqp_set_default_settings(&st);      // ← 必须先调，否则 st 是未初始化内存
st.verbose = 0;
st.warm_start = 1;
// ... 其他参数
if (osqp_setup(&work_, &data, &st) != 0) {
    throw std::runtime_error("MPC_Solver: osqp_setup failed");
}

// ── 每拍（update -> warm_start -> solve）──
osqp_update_lin_cost(work_, q_.data());
osqp_update_bounds(work_, l_.data(), u_.data());
osqp_warm_start(work_, work_->solution->x, work_->solution->y);
if (osqp_solve(work_) != 0) { /* 出错 */ }

// ── 析构 ──
osqp_cleanup(work_);
```

### 为什么有 update 就不用重新 setup

`osqp_setup` 会做**符号分解 + 数值分解**，很贵。而我们的 $P$ 和 $A$ 是**常值矩阵**（只和 $N$、$dt$、权重有关），每拍只有 $q$ 和 $u/l$ 在变：

$$
f = 2\Gamma^T\bar Q(\Phi x_0 - \bar X^{ref}) \quad\text{(每拍变)}, \qquad
b = [a_{max},\;-a_{min},\;v_{max}-S_v\Phi x_0,\;\ldots] \quad\text{(每拍变)}
$$

所以 **setup 一次 + 每拍 update** 是必须的。**如果每拍 `osqp_setup`，20 Hz 根本跑不到。**

---

## 4. 参数（`OSQPSettings`）

显式设置的只有这几个（`mpc_solver.cpp:122-130`）：

| 字段 | 自己设的 | 默认 | 含义 | 调整建议 |
|---|---|---|---|---|
| `verbose` | `0` | 1 | 是否打印每次迭代 | 调试时可临时开 1 |
| `warm_start` | `1` | 1 | 用上次解做初值 | **别关**，关了迭代次数翻几倍 |
| `polish` | `1` | 0 | 解完做抛光，提高精度 | $N$ 大时很贵，可试关掉 |
| `scaling` | `10` | 10 | 数据缩放迭代次数；0 = 关闭 | 关掉可省时间，但要确认还能收敛 |
| `eps_abs` | `1e-4` | 1e-3 | 绝对收敛容差 | 收紧 → 更准但更慢 |
| `eps_rel` | `1e-4` | 1e-3 | 相对收敛容差 | 同上 |
| `max_iter` | `2000` | 4000 | 最大迭代次数 | 我们实测 $\approx$250 就够，设 2000 是安全余量 |

### 其他值得知道但没设的字段

| 字段 | 默认 | 说明 |
|---|---|---|
| `rho` | 0.1 | ADMM 步长，通常不用手动调 |
| `sigma` | 1e-6 | ADMM 正则项 |
| `alpha` | 1.6 | 松弛因子 |
| `adaptive_rho` | 1 | 自动调 `rho` |
| `scaled_termination` | 1 | 用缩放后的判据 |
| `check_termination` | 25 | 每多少次迭代检查一次收敛 |
| `time_limit` | 0 | 求解时间上限（秒），0 = 不限 |
| `eps_prim_inf` / `eps_dual_inf` | 1e-4 | 不可行性判据 |

> **调优思路**：与其乱调 `rho`，不如先看 `info->iter`。迭代次数远超预期说明问题条件数差，通常是**权重尺度差异太大**，优先调权重而不是调 OSQP。

---

## 5. 求解状态码

定义在 `constants.h`：

| 值 | 宏 | 含义 | 处理方式 |
|---|---|---|---|
| **1** | `OSQP_SOLVED` | 找到高精度最优解 | ✅ 接受 |
| **2** | `OSQP_SOLVED_INACCURATE` | 低精度但可用 | ✅ 接受 |
| 3 | `OSQP_PRIMAL_INFEASIBLE_INACCURATE` | 可能原问题不可行 | ❌ 拒绝 |
| 4 | `OSQP_DUAL_INFEASIBLE_INACCURATE` | 可能对偶不可行 | ❌ 拒绝 |
| -2 | `OSQP_MAX_ITER_REACHED` | 迭代用尽 | ❌ 拒绝 |
| -3 | `OSQP_PRIMAL_INFEASIBLE` | 原问题不可行 | ❌ 拒绝 |
| -4 | `OSQP_DUAL_INFEASIBLE` | 对偶不可行 | ❌ 拒绝 |
| -5 | `OSQP_SIGINT` | 被中断 | ❌ 拒绝 |
| -7 | `OSQP_NON_CVX` | 问题非凸 | ❌ 拒绝 |
| -10 | `OSQP_UNSOLVED` | 只 setup 过，没解 | ❌ 拒绝 |

判断（`mpc_solver.cpp:185`）：

```cpp
last_status_ = static_cast<int>(work_->info->status_val);
last_iter_   = static_cast<int>(work_->info->iter);

if (last_status_ != OSQP_SOLVED && last_status_ != OSQP_SOLVED_INACCURATE)
    return false;
```

> **`osqp_solve` 返回 0 不等于求解成功！** 0 只表示"函数调用没出错"。必须再查 `info->status_val`。
> 这是最容易漏的一步。

---

## 6. 结果怎么读

```c
work_->solution->x   // 解向量，长度 n      -> 我们的 U*（2N 维）
work_->solution->y   // 对偶变量，长度 m    -> 热启动用
work_->info->status_val  // 状态码
work_->info->iter        // 迭代次数
work_->info->obj_val     // 目标函数值
work_->info->pri_res     // 原始残差
work_->info->dua_res     // 对偶残差
```

转成 Eigen（`mpc_solver.cpp:188`）：

```cpp
solution_ = Eigen::Map<Eigen::VectorXd>(work_->solution->x, numVars());

// 只用第一步
Eigen::Vector2d firstControl() const { return solution_.head<2>(); }
```

### 💡 本项目 `PROFILING` 是开着的

`osqp_configure.h:23` 定义了 `PROFILING`，所以这些**计时字段可用**：

```cpp
work_->info->setup_time    // setup 耗时（秒）
work_->info->solve_time    // 求解耗时（秒）
work_->info->update_time   // update 耗时
work_->info->polish_time   // 抛光耗时
work_->info->run_time      // 总耗时
```

排查 `Control loop missed its desired rate` 时，直接打这个最有效：

```cpp
RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
    "QP solve=%.2fms polish=%.2fms iter=%d status=%d",
    work->info->solve_time  * 1e3,
    work->info->polish_time * 1e3,
    work->info->iter, work->info->status_val);
```

---

## 7. 实际踩过的坑

| # | 坑 | 后果 | 正确做法 |
|---|---|---|---|
| 1 | `P_csc.m = m`（m 是约束数） | `P` 维度错，setup 失败 | **`P_csc.n = n; P_csc.m = n;`**（$P$ 是 $n\times n$ 方阵） |
| 2 | `P` 填了下三角/全矩阵 | 目标函数算错，解不对 | 只填 $r \le c$ |
| 3 | `toFullCSC` 里 `x.push_back(M(r,j))` 写成了 `i.push_back(...)` | 行索引和数值错位，矩阵全乱 | 数值进 `x`，行号进 `i` |
| 4 | 手动 `csc_spfree(&P_csc)` | **崩溃 / 双重释放** | **不要 free**。`osqp_setup` 会**深拷贝** $P$、$A$ 的数据，我们的 vector 由 `std::vector` 自己管 |
| 5 | `l` 只填了 `u`，忘了下界 | 解出来是垃圾 | 只有上界时 `l` 必须显式填 `-OSQP_INFTY`（`1e30`） |
| 6 | 每拍调 `osqp_setup` | 20 Hz 跑不到 | 只 setup 一次，之后 `osqp_update_*` |
| 7 | 只看 `osqp_solve` 返回值 | 把不可行解当成功用 | 必须查 `info->status_val` |
| 8 | 用 `%d` 打印 `c_int` | 输出乱码 | `c_int` 是 `long long`，用 `%lld` |
| 9 | `osqp_set_default_settings` 忘了调 | `settings` 是未初始化内存，行为随机 | setup 前必须先调它 |
| 10 | 字段名写成 `polishing` | 编译错误 | 0.6.2 里叫 **`polish`** |

---

## 8. 最小可用模板

把上面的东西串起来，一个新 QP 问题接入 OSQP 的骨架就是：

```cpp
class MyQpSolver {
  OSQPWorkspace *work_ = nullptr;
  std::vector<c_float> P_x_, A_x_, q_, l_, u_;
  std::vector<c_int>   P_i_, P_p_, A_i_, A_p_;
  Eigen::VectorXd solution_;
  int last_status_ = 0, last_iter_ = 0;

public:
  MyQpSolver(int n, int m) {
    // 1) 构造常量矩阵，转 CSC
    toUpperCSC(H, P_x_, P_i_, P_p_);      // P 只填上三角
    toFullCSC (A, A_x_, A_i_, A_p_);

    q_.assign(n, 0.0);
    l_.assign(m, -OSQP_INFTY);            // ★ 只有上界
    u_.assign(m, 0.0);

    // 2) 组装 csc / OSQPData
    csc Pc{static_cast<c_int>(P_x_.size()), n, n, P_p_.data(), P_i_.data(), P_x_.data(), -1};
    csc Ac{static_cast<c_int>(A_x_.size()), m, n, A_p_.data(), A_i_.data(), A_x_.data(), -1};
    OSQPData data{n, m, &Pc, &Ac, q_.data(), l_.data(), u_.data()};

    // 3) settings + setup（一次）
    OSQPSettings st;
    osqp_set_default_settings(&st);
    st.verbose = 0; st.warm_start = 1; st.polish = 1; st.scaling = 10;
    st.eps_abs = 1e-4; st.eps_rel = 1e-4; st.max_iter = 2000;
    if (osqp_setup(&work_, &data, &st) != 0)
      throw std::runtime_error("osqp_setup failed");
  }

  ~MyQpSolver() { if (work_) osqp_cleanup(work_); }

  bool solve(const Eigen::VectorXd &q_new, const Eigen::VectorXd &u_new) {
    for (int i = 0; i < (int)q_.size(); ++i) q_[i] = q_new[i];
    for (int i = 0; i < (int)u_.size(); ++i) u_[i] = u_new[i];
    osqp_update_lin_cost(work_, q_.data());
    osqp_update_bounds  (work_, l_.data(), u_.data());
    osqp_warm_start     (work_, work_->solution->x, work_->solution->y);

    if (osqp_solve(work_) != 0) return false;
    last_status_ = (int)work_->info->status_val;
    last_iter_   = (int)work_->info->iter;
    if (last_status_ != OSQP_SOLVED && last_status_ != OSQP_SOLVED_INACCURATE) return false;

    solution_ = Eigen::Map<Eigen::VectorXd>(work_->solution->x, q_.size());
    return true;
  }
};
```

---

## 9. 编译链接

```cmake
# CMakeLists.txt
find_package(osqp_vendor REQUIRED)                 # ROS 封装，提供 osqp::osqp 目标
target_link_libraries(${PROJECT_NAME} osqp::osqp)
```

```xml
<!-- package.xml -->
<depend>osqp_vendor</depend>
```

头文件写法：`#include <osqp.h>`（路径由 `osqp_vendor` 的 CMake 目标带出来，不用手动加 `-I`）。

---

## 10. 参考

- OSQP 官方文档：<https://osqp.org/docs/>
- 本机头文件：`/opt/ros/humble/include/osqp/`
  - `types.h` —— `csc` / `OSQPData` / `OSQPSettings` / `OSQPInfo` / `OSQPSolution`
  - `constants.h` —— 版本号、状态码、`OSQP_INFTY`
  - `osqp.h` —— 所有 API 原型
  - `osqp_configure.h` —— 编译开关（`DLONG` / `PROFILING` 等）
- 本项目实现：`src/mpc_solver.cpp`、`include/mpc_controller/mpc_solver.hpp`
