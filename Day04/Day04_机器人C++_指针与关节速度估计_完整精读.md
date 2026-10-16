# 机器人运动控制 C++17 每日手撕训练｜Day 04

> **日期**：2026-10-16　｜　**难度**：零基础·第 4 天　｜　**建议时间**：60–90 分钟  
> **今日主题**：指针 `*`、取地址 `&`、`nullptr`、只读输入指针、可写输出指针、数组首地址。  
> **连续课程**：Day 01 变量/循环/限幅 → Day 02 函数/返回值 → Day 03 按值传递/`const`/引用 → **Day 04 指针与传感器缓冲区**。

## 0. 学完今天，你应该能做到什么？

1. 解释 `double x`、`double* p`、`&x`、`*p` 分别代表什么。
2. 不看答案，写出一个带 `nullptr` 检查的 C++ 函数。
3. 区分 `const double*`（通过此指针不可修改数据）与 `double*`（可修改数据）。
4. 用 `vector.data()` 将数组交给一个指针参数函数。
5. 将“角度校准”和“离散差分估计关节角速度”的数学式写成可运行的 C++17 代码。

**本日两题**：

- **题 1：关节编码器零位校准（指针读写）**：把度转换成弧度，安全处理空指针。
- **题 2：多关节角速度估计（指针与连续数组）**：利用相邻采样做差分，检查 `dt`、数据和缓冲区。

---

## 1. 开始前：指针到底是什么？

### 1.1 从变量到地址

```cpp
double angle = 1.2;    // 创建变量：存放数值 1.2
double* p = &angle;    // &angle 取出 angle 的地址；p 保存这个地址
double copy = *p;     // *p 访问 p 指向的数值；copy 现在是 1.2
*p = 1.5;             // 修改 p 指向的变量；angle 也变成 1.5
```

- **`double angle`**：一个普通双精度变量，存储数值。
- **`double* p`**：声明 `p` 是一个指向 `double` 的指针，里面保存的是地址。
- **`&angle`**：对变量 `angle` 使用“取地址运算符”，得到它的地址。
- **`*p`**：当 `p` 是指针表达式时，`*p` 表示“解引用”，读写它所指向的变量。
- **易混淆**：声明中的 `double* p` 与表达式中的 `*p` 虽然都有星号，语义不同：前者声明指针类型，后者访问指针指向的对象。

想象机器人控制程序：`angle` 是内存中一格关节角度数据，`p` 是这格数据的“地址标签”。拿着地址就能找到同一份数据，不必复制它。

### 1.2 为什么要检查空指针？

```cpp
double* p = nullptr;  // 不指向任何有效对象
if (p != nullptr) {   // 只有非空才允许访问
    *p = 1.0;
}
```

**绝对不要**执行 `*nullptr`，这是未定义行为，可能崩溃，也可能表现得更隐蔽。非空仅仅是必要条件，不代表一定有效：悬空指针、越界指针也可能非空。

### 1.3 指针与 Day 03 引用的区别

| 写法 | 是什么 | 是否允许空值 | 函数调用方式 | 常见用途 |
|---|---|---|---|---|
| `double x` | 按值传递，复制数值 | 不适用 | `f(angle)` | 小型数值参数 |
| `double& x` | 可写引用，绑定已有对象 | 正常有效引用不可为空 | `f(angle)` | 必须存在的输出对象 |
| `const double& x` | 只读引用 | 正常有效引用不可为空 | `f(angle)` | 不复制、只读访问 |
| `double* p` | 可写指针 | 可以是 `nullptr` | `f(&angle)` | 可选输出、数组缓冲区 |
| `const double* p` | 只读指针 | 可以是 `nullptr` | `f(&angle)` | 只读输入缓冲区 |

**补充**：`const double* p` 中，`const` 约束的是通过 `p` 对目标数值的修改；指针 `p` 自身仍可指向其他地址。`double* const p = &angle` 则是“指针本身不能改指向”。不要把两者混为一谈。

---

## 2. 题目一：关节编码器零位校准

### 2.1 机器人场景与面试题目

假设一个关节编码器读出原始角度 `raw_degree`（单位：度），标定得到零位偏移 `zero_offset_degree`（单位：度）。机器人运动学/控制程序希望拿到以弧度为单位的关节角 `angle_rad`。

要求编写函数：

```cpp
bool calibrateAngle(const double* raw_degree,
                    double zero_offset_degree,
                    double* angle_rad);
```

- 原始角度使用 `const double*` 输入，**不得修改原始读数**。
- 校准后的角度通过 `double*` 输出。
- 遇到空指针、NaN、无穷大等非法数据返回 `false`，且**不能改变调用者原有输出**。
- 允许正常的正角度和负角度；本题不执行角度周期归一化。

### 2.2 数学原理与详细推导

设原始角度为 \(q_{\mathrm{raw}}\)，零位偏移为 \(q_0\)。零位校正：

\[
q_{\mathrm{corrected,deg}}=q_{\mathrm{raw,deg}}-q_{0,\mathrm{deg}}.
\]

因为 \(180^\circ=\pi\,\mathrm{rad}\)，所以：

\[
\boxed{q_{\mathrm{rad}}=(q_{\mathrm{raw,deg}}-q_{0,\mathrm{deg}})\frac{\pi}{180}}.
\]

**算例**：原始读数 \(90^\circ\)，零位偏移 \(10^\circ\)：

\[
q_{\mathrm{corrected,deg}}=90-10=80^\circ,
\qquad q_{\mathrm{rad}}=80\frac{\pi}{180}\approx1.396263\ \mathrm{rad}.
\]

**边界理解**：负角度是合法物理数据，不应该被截断；超过 \(180^\circ\) 的读数也不一定非法。角度是否需要 wrap 到 \([-\pi,\pi)\) 取决于编码器与算法定义，本题不自动处理。

### 2.3 输入输出

**输入**（一行两个数，单位均为度）：

```text
90 10
```

**输出**：

```text
校准角度(rad): 1.396263
```

**负角度测试**：

```text
输入：-170 -10
输出：校准角度(rad): -2.792527
```

### 2.4 完整 C++17 代码（中文注释）

保存为 `src/day04_q1_angle_calibration.cpp`：

```cpp
#include <cmath>      // std::isfinite：检查数值是否有限
#include <iomanip>    // std::fixed、std::setprecision：控制输出格式
#include <iostream>   // std::cin、std::cout、std::cerr

// 功能：将编码器角度（度）减去零位偏移，再换算为弧度。
// raw_degree：输入指针，只读取它指向的数值。
// zero_offset_degree：按值传入的零位偏移，单位为度。
// angle_rad：输出指针，写入校准后的弧度值。
// 返回 true 表示成功，false 表示输入无效；失败时不修改输出值。
bool calibrateAngle(const double* raw_degree,
                    double zero_offset_degree,
                    double* angle_rad) {
    // 先判断指针是否为空，避免对 nullptr 解引用。
    if (raw_degree == nullptr || angle_rad == nullptr) {
        return false;
    }

    // 再检查数据是否为有限数，排除 NaN 和正负无穷。
    if (!std::isfinite(*raw_degree) ||
        !std::isfinite(zero_offset_degree)) {
        return false;
    }

    // C++17 没有标准常量 std::numbers::pi，手动定义圆周率。
    constexpr double pi = 3.14159265358979323846;

    // *raw_degree 表示读取指针指向的原始角度。
    const double corrected_degree = *raw_degree - zero_offset_degree;
    const double result = corrected_degree * pi / 180.0;

    // 防止极端大数在计算中溢出成无穷。
    if (!std::isfinite(result)) {
        return false;
    }

    // *angle_rad 表示把结果写入调用者提供的变量。
    *angle_rad = result;
    return true;
}

int main() {
    double raw_degree = 0.0;    // 原始编码器角度，单位：度
    double zero_offset = 0.0;   // 零位偏移，单位：度
    double angle_rad = 0.0;     // 校准后的角度，单位：弧度

    // 示例输入：90 10
    if (!(std::cin >> raw_degree >> zero_offset)) {
        std::cerr << "输入错误：请输入两个数字。\n";
        return 1;
    }

    // &raw_degree 取得变量地址，&angle_rad 取得输出变量地址。
    const bool ok = calibrateAngle(&raw_degree, zero_offset, &angle_rad);
    if (!ok) {
        std::cerr << "角度校准失败：指针或数据无效。\n";
        return 1;
    }

    // 固定保留小数点后 6 位，便于核对结果。
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "校准角度(rad): " << angle_rad << '\n';
    return 0;
}
```

### 2.5 逐段代码与语法解释

**① `#include` 不是“运行时调用”**：`#include <iostream>` 让编译器看到标准输入输出声明；`<cmath>` 提供 `std::isfinite`；`<iomanip>` 提供输出格式控制。`std::` 表示标准库命名空间，避免名字冲突。

**② `bool` 函数为什么不直接返回 `double`？** 这里有两件事：是否成功，以及成功时的结果。用 `bool` 返回状态、用 `double*` 输出结果，能明确表示失败；否则若返回 `0.0`，无法区分“合法的 0 rad”和“出错”。这只是常见接口设计之一，后面会学 `std::optional` 等现代写法。

**③ `const double* raw_degree`**：`raw_degree` 是指向输入角度的地址。`*raw_degree` 读取角度。`const` 表示不能通过这个指针写 `*raw_degree = ...`。

**④ `double* angle_rad`**：传入调用者变量的地址。`*angle_rad = result;` 修改调用者的变量，因此 `main()` 中 `angle_rad` 能接收到结果。

**⑤ `&raw_degree` 与 `*raw_degree` 不是一回事**：前者在 `main` 中取局部变量的地址；后者在函数内部解引用一个指针形参。

**⑥ `nullptr` 检查必须在 `*p` 之前**：本题先判断两个指针，随后才检查 `std::isfinite(*raw_degree)`，避免空指针解引用。

**⑦ `constexpr double pi`**：定义编译期常量。注意 `std::numbers::pi` 属于 C++20，不是本课程 C++17 的标准接口。

**⑧ `std::isfinite`**：检查浮点数不是 NaN、`+inf` 或 `-inf`。输入值有限，不代表中间运算一定有限，因此也检查结果。

**⑨ `if (!(std::cin >> a >> b))`**：`>>` 连续读取；如果输入失败，流转成布尔假，外层 `!` 取反后进入错误分支。`return 1` 表示程序异常结束，`return 0` 表示正常结束。

**⑩ `std::fixed << std::setprecision(6)`**：让后续浮点输出采用固定小数形式，保留 6 位小数，方便人工检查。它只改变输出显示，不改变内存中 `double` 的精度。

### 2.6 题 1 常见错误

```cpp
// 错误 A：传了数值而不是地址
// calibrateAngle(raw_degree, zero_offset, angle_rad);  // 类型不匹配

// 正确：传地址
// calibrateAngle(&raw_degree, zero_offset, &angle_rad);

// 错误 B：未判断 nullptr 就解引用
// double x = *raw_degree;

// 错误 C：忘了度到弧度的转换
// *angle_rad = *raw_degree - zero_offset_degree;
```

### 2.7 复杂度

- 时间复杂度：\(O(1)\)，只做固定次数的判断与计算。
- 额外空间复杂度：\(O(1)\)，只使用少量局部变量。

---

## 3. 题目二：多关节角速度估计

### 3.1 机器人场景与面试题目

控制器每隔 \(\Delta t\) 秒读取一次多个关节的位置。若没有直接使用关节速度传感器，可用相邻两次关节位置做最简单的**后向差分**速度估计。

实现接口：

```cpp
bool estimateJointVelocities(const double* q_now,
                             const double* q_prev,
                             std::size_t n,
                             double dt,
                             double* dq);
```

要求：`q_now`、`q_prev` 是长度至少为 `n` 的只读数组；`dq` 是长度至少为 `n` 的输出数组；指针不可为空、`n>0`、`dt>0` 且数据必须有限。任何检查失败时，`dq` 原有内容保持不变。

> **接口前提**：调用者必须保证传入缓冲区确实包含至少 `n` 个元素，且输入与输出缓冲区不重叠。仅凭原始指针和 `n`，函数无法检查真实分配长度；后续学习 `std::span`（C++20）时可改善接口表达。

### 3.2 数学原理与详细推导

关节位置是时间函数 \(q_i(t)\)，速度定义为：

\[
\dot q_i(t)=\lim_{\Delta t\to0}\frac{q_i(t)-q_i(t-\Delta t)}{\Delta t}.
\]

在离散采样下，令 \(t_k-t_{k-1}=\Delta t\)：

\[
\boxed{\dot q_i[k]\approx\frac{q_i[k]-q_i[k-1]}{\Delta t}},\quad i=0,\ldots,n-1.
\]

向量形式：

\[
\boxed{\dot{\boldsymbol q}[k]\approx\frac{\boldsymbol q[k]-\boldsymbol q[k-1]}{\Delta t}}.
\]

**物理单位检查**：\(q\) 是 rad，\(\Delta t\) 是 s，所以 \(\dot q\) 是 rad/s。

**算例**：

\[
\boldsymbol q[k-1]=[0.10,-0.20,1.00]^\top,\quad
\boldsymbol q[k]=[0.15,-0.22,0.98]^\top,\quad \Delta t=0.1\,s.
\]

\[
\dot{\boldsymbol q}[k]\approx
\frac{[0.05,-0.02,-0.02]^\top}{0.1}
=[0.5,-0.2,-0.2]^\top\,\mathrm{rad/s}.
\]

**重要工程局限**：差分会放大位置噪声，采样间隔过小而测量噪声不变时，速度估计可能更抖；编码器跨越 \(\pm\pi\) 边界时若不处理角度环绕，会出现虚假大速度；非均匀采样必须使用真实 \(\Delta t\)。这些问题以后会在滤波和状态估计中解决。

### 3.3 输入输出

**输入**：第一行 `n dt`；第二行 `n` 个上一时刻角度；第三行 `n` 个当前时刻角度。

```text
3 0.1
0.1 -0.2 1.0
0.15 -0.22 0.98
```

**输出**：

```text
关节角速度(rad/s): 0.500 -0.200 -0.200
```

### 3.4 完整 C++17 代码（中文注释）

保存为 `src/day04_q2_joint_velocity.cpp`：

```cpp
#include <cmath>      // std::isfinite
#include <cstddef>    // std::size_t
#include <iomanip>    // std::fixed、std::setprecision
#include <iostream>   // std::cin、std::cout、std::cerr
#include <vector>     // std::vector

// 功能：用相邻两次关节角采样估计每个关节的角速度。
// q_now、q_prev：指向至少 n 个 double 的只读连续数组。
// n：关节数量；dt：采样时间间隔（秒），必须大于 0。
// dq：指向至少 n 个 double 的可写输出数组。
// 约定：输入和输出缓冲区不重叠；调用者保证缓冲区长度足够。
// 失败时返回 false，且不会修改 dq 中的任何元素。
bool estimateJointVelocities(const double* q_now,
                             const double* q_prev,
                             std::size_t n,
                             double dt,
                             double* dq) {
    // 必须先检查空指针、数组长度和采样周期。
    if (q_now == nullptr || q_prev == nullptr || dq == nullptr ||
        n == 0 || !std::isfinite(dt) || dt <= 0.0) {
        return false;
    }

    // 第一遍：验证所有输入及计算结果，避免中途失败导致部分写入。
    for (std::size_t i = 0; i < n; ++i) {
        if (!std::isfinite(q_now[i]) || !std::isfinite(q_prev[i])) {
            return false;
        }
        const double candidate = (q_now[i] - q_prev[i]) / dt;
        if (!std::isfinite(candidate)) {
            return false;
        }
    }

    // 第二遍：确认全部合法后，再写入输出数组。
    for (std::size_t i = 0; i < n; ++i) {
        dq[i] = (q_now[i] - q_prev[i]) / dt;
    }
    return true;
}

int main() {
    int count = 0;     // 用 int 读入，先检查是否为负数。
    double dt = 0.0;   // 采样间隔，单位秒。

    // 第一行示例：3 0.1
    if (!(std::cin >> count >> dt) || count <= 0 || count > 1000) {
        std::cerr << "输入错误：关节数应为 1~1000。\n";
        return 1;
    }

    // 用 vector 自动管理数组内存，避免手动 new/delete。
    const std::size_t n = static_cast<std::size_t>(count);
    std::vector<double> q_prev(n);
    std::vector<double> q_now(n);
    std::vector<double> dq(n, 0.0);

    // 第二行输入 n 个上一时刻的关节角度（rad）。
    for (std::size_t i = 0; i < n; ++i) {
        if (!(std::cin >> q_prev[i])) {
            std::cerr << "上一时刻角度输入不完整。\n";
            return 1;
        }
    }

    // 第三行输入 n 个当前时刻的关节角度（rad）。
    for (std::size_t i = 0; i < n; ++i) {
        if (!(std::cin >> q_now[i])) {
            std::cerr << "当前时刻角度输入不完整。\n";
            return 1;
        }
    }

    // .data() 返回 vector 底层连续数组首元素的指针。
    const bool ok = estimateJointVelocities(
        q_now.data(), q_prev.data(), n, dt, dq.data());
    if (!ok) {
        std::cerr << "速度估计失败：请检查 dt 和角度数据。\n";
        return 1;
    }

    // 输出单位 rad/s；每个元素之间用空格分隔。
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "关节角速度(rad/s):";
    for (std::size_t i = 0; i < n; ++i) {
        std::cout << ' ' << dq[i];
    }
    std::cout << '\n';
    return 0;
}
```

### 3.5 逐段代码与语法解释

**① `const double* q_now` 与 `const double* q_prev`**：它们指向连续数组的第一个元素，函数承诺不通过这些指针修改输入。

**② `double* dq`**：指向输出数组的首元素；`dq[i]` 相当于 `*(dq + i)`，但后者暂时只需认识，不建议新手到处写指针运算。

**③ `std::size_t n`**：标准库用于表示对象大小和数组下标的无符号整数类型。由于无符号类型无法表示负数，示例 `main` 先用 `int count` 读入并验证为正，再转换成 `std::size_t`。

**④ `static_cast<std::size_t>(count)`**：显式进行安全前提已验证的整数类型转换；比 C 风格强制转换更明确。

**⑤ `std::vector<double> q_prev(n)`**：创建长度为 `n` 的动态数组。`vector` 自动管理内存，不需要初学者手写 `new`/`delete`。

**⑥ `.data()`**：例如 `q_now.data()` 返回连续存储区的首地址，类型是 `double*`；可以隐式转换为 `const double*` 传给只读形参。若随后让 `vector` 扩容，之前保存的指针可能失效，因此本题在调用期间不改变数组容量。

**⑦ 为什么两次 `for` 循环？** 第一遍先验证所有输入和候选计算值，第二遍才写入结果。这样可以保证本题约定的“失败时输出不变”。单次循环若第 3 个元素是 NaN，前两个结果可能已经覆盖输出。

**⑧ `||` 是逻辑或**：任何一个错误条件成立就返回 `false`。`||` 有短路求值，前面为空时不会继续进行不安全的解引用。

**⑨ `n == 0` 与 `dt <= 0.0`**：没有关节或没有正的时间间隔，差分无意义。`std::isfinite(dt)` 还排除 NaN 和无穷。

**⑩ `std::cout << ' ' << dq[i]`**：在每个速度前输出空格，使一行结果易于阅读；`' '` 是字符字面量，`" "` 是字符串字面量。

### 3.6 复杂度与接口约束

- 计算函数执行两次长度为 `n` 的循环，总时间复杂度仍为 \(O(n)\)。
- 计算函数额外空间复杂度为 \(O(1)\)；调用者提供的输入和输出数组不计入函数额外空间。
- 示例 `main()` 中创建 3 个长度为 `n` 的 `vector`，因此整个示例的数组存储为 \(O(n)\)。
- **不要认为非空指针一定安全**：若 `n` 大于真实数组长度，`q_now[i]` 仍会越界；若指针指向已释放内存，也会出错。

### 3.7 常见错误

```cpp
// 错误 A：把角度差当作速度，忘了除以 dt
// dq[i] = q_now[i] - q_prev[i];

// 错误 B：在 dt=0 时除法
// dq[i] = (q_now[i] - q_prev[i]) / dt;

// 错误 C：用数组变量传给指针形参时忘了 .data()
// estimateJointVelocities(q_now, q_prev, n, dt, dq); // vector 不能直接当 double*

// 正确：
// estimateJointVelocities(q_now.data(), q_prev.data(), n, dt, dq.data());
```

---

## 4. 如何在 Ubuntu / VS Code 中编译运行？

进入本学习包目录后，分别编译两道题（编译器需支持 C++17）：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic src/day04_q1_angle_calibration.cpp -o day04_q1
g++ -std=c++17 -Wall -Wextra -Wpedantic src/day04_q2_joint_velocity.cpp -o day04_q2
```

- `g++`：C++ 编译器；`-std=c++17`：指定 C++17；`-Wall -Wextra -Wpedantic`：启用常见警告；`-o`：指定输出可执行文件名。
- 运行题 1：

```bash
printf '90 10\n' | ./day04_q1
# 校准角度(rad): 1.396263
```

- 运行题 2：

```bash
printf '3 0.1\n0.1 -0.2 1.0\n0.15 -0.22 0.98\n' | ./day04_q2
# 关节角速度(rad/s): 0.500 -0.200 -0.200
```

- 编译并运行附带的函数级测试：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -O2 -UNDEBUG tests/day04_tests.cpp -o day04_tests
./day04_tests
# PASS: 16 groups of function and boundary tests
```

**注意**：测试文件使用 `#define main` 临时重命名示例入口，以便测试原始 `.cpp` 里的函数。新手暂时不需要模仿这种测试组织方法；后面学习头文件、源文件分离时会采用更标准的项目结构。

---

## 5. 测试设计：正常、边界、异常

| 测试 | 输入 / 条件 | 预期 | 意义 |
|---|---|---|---|
| 题1 正常 | 90°, 偏移 10° | 1.396263 rad | 校准与换算 |
| 题1 负角 | -170°, 偏移 -10° | -2.792527 rad | 负角不应被截断 |
| 题1 零值 | 0°, 偏移 0° | 0 rad | 零值合法 |
| 题1 空输入指针 | `raw_degree=nullptr` | `false` | 防止空指针解引用 |
| 题1 空输出指针 | `angle_rad=nullptr` | `false` | 防止非法写入 |
| 题1 无穷读数 | `raw=+inf` | `false` | 排除非法数值 |
| 题1 NaN 偏移 | `offset=NaN` | `false` | 排除非法数值 |
| 题2 正常 | 3 关节，`dt=0.1` | `[0.5,-0.2,-0.2]` | 差分计算 |
| 题2 任一空指针 | `nullptr` | `false` | 输入/输出检查 |
| 题2 空数组 | `n=0` | `false` | 长度检查 |
| 题2 零/负采样周期 | `dt=0/-0.1` | `false` | 避免非法时间 |
| 题2 无穷采样周期 | `dt=+inf` | `false` | 浮点边界 |
| 题2 某个角度为 NaN | `q_now[2]=NaN` | `false` 且 `dq` 不变 | 原子式写入约定 |

**本次实际执行**：两份源代码均通过 `g++ -std=c++17 -Wall -Wextra -Wpedantic` 编译，测试程序输出 `PASS: 16 groups of function and boundary tests`。表中将若干空指针和时间检查合并展示，因此表格行数与测试断言数量不一一对应。

---

## 6. 面试官追问与参考答案（12 问）

**Q1：指针和引用的本质区别？**  
A：指针变量保存地址，可以显式为 `nullptr`、可以改变指向；引用是已有对象的别名，正常有效引用在初始化后绑定到同一对象。两者都能避免复制大对象，但接口语义不同。

**Q2：`&` 和 `*` 分别有什么含义？**  
A：`&x` 在表达式中取得对象地址，`*p` 在表达式中解引用指针。注意 `double& x` 里的 `&` 是引用类型声明，不是取地址操作。

**Q3：`const double* p` 是否意味着 p 自己不能改变？**  
A：不是。它限制通过 `p` 修改目标值；`p` 可以重新指向别处。若限制指针本身，写 `double* const p`。

**Q4：为什么先判断 `nullptr`？**  
A：解引用空指针是未定义行为。检查必须发生在 `*p` 之前，但非空检查本身不能保证指针有效。

**Q5：`double* output` 和 `double& output` 怎么选？**  
A：输出必须存在时引用接口通常更直观；允许空值或需与 C API / 原始数组交互时指针更合适。今天用指针是为了练习空指针检查与内存地址。

**Q6：为什么 `vector.data()` 可以传给 `const double*`？**  
A：`vector<double>` 的元素连续存储，`.data()` 返回首元素地址；`double*` 可隐式转换为 `const double*`，只读访问安全。

**Q7：`std::size_t` 与 `int` 的区别？**  
A：`size_t` 是用于大小和索引的无符号类型，范围取决于平台；`int` 可表达负数。输入若可能为负，先用有符号整数验证，再转换。

**Q8：函数为什么不能只靠 `n` 检查数组真实长度？**  
A：原始指针不携带分配长度元数据，调用者必须保证缓冲区至少有 `n` 个元素；越界访问属于未定义行为。

**Q9：为什么采用后向差分？**  
A：它只需要当前和上一个采样值，适合在线控制；在采样间隔足够小且轨迹光滑时可近似导数，但有离散误差和噪声放大问题。

**Q10：采样周期越小，速度估计一定越好吗？**  
A：不一定。差分中的测量噪声也被 `dt` 除，过小 `dt` 可能导致速度噪声增大，需要结合传感器分辨率、滤波与采样时间戳考虑。

**Q11：为何题 2 分两遍遍历？**  
A：第一遍检查所有输入及候选输出，第二遍统一写入。若中途出现 NaN/Inf，可保证调用者输出未被部分修改，代价是多一次线性遍历。

**Q12：这段代码能直接用于真实机器人速度闭环吗？**  
A：只能作为教学用基础模块。真实系统还要处理时间戳抖动、角度环绕、编码器噪声、滤波、异常数据处理与线程同步，且需按控制频率验证实时性。

---

## 7. 课后手撕练习与参考答案

### 练习 1（基础）：写出指针的值与修改

已知：

```cpp
double q = 0.8;
double* p = &q;
*p = *p + 0.2;
```

问：`q` 最后是多少？`p` 里面存放的是 `q` 的数值还是地址？

**参考答案**：`q` 最后约为 `1.0`（浮点表示存在舍入）；`p` 保存 `q` 的地址，`*p` 才是其指向的数值。

### 练习 2（基础）：编写安全的单关节速度函数

函数签名：

```cpp
bool estimateSingleVelocity(const double* q_now,
                            const double* q_prev,
                            double dt,
                            double* velocity);
```

要求：任何指针为空、时间不合法、数据非有限时返回 `false`，成功时输出速度。

**参考实现**：

```cpp
#include <cmath>

bool estimateSingleVelocity(const double* q_now,
                            const double* q_prev,
                            double dt,
                            double* velocity) {
    if (q_now == nullptr || q_prev == nullptr || velocity == nullptr ||
        !std::isfinite(dt) || dt <= 0.0) {
        return false;
    }
    if (!std::isfinite(*q_now) || !std::isfinite(*q_prev)) {
        return false;
    }
    const double result = (*q_now - *q_prev) / dt;
    if (!std::isfinite(result)) {
        return false;
    }
    *velocity = result;
    return true;
}
```

### 练习 3（理解）：为什么 `const double*` 更合适？

若函数只需读取传感器数据，应写 `double* sensor` 还是 `const double* sensor`？

**参考答案**：优先 `const double* sensor`，它表达只读意图并能接受常量数据指针，减少误改输入的风险。`const` 不能替代对指针生命周期、边界和并发访问的检查。

### 练习 4（拓展）：差分估计为什么会有“速度尖峰”？

假设关节位置读数从 \(+3.13\,rad\) 跳到 \(-3.13\,rad\)，采样间隔 \(0.01s\)。直接差分会得到多少？为什么它不一定是真实关节速度？

**参考答案**：直接计算 \((-3.13-3.13)/0.01=-626\,rad/s\)。如果这是周期角度从 \(+\pi\) 穿越到 \(-\pi\) 的表示跳变，实际最短角度变化可能只有约 \(0.023\,rad\)，不应直接用原始差分；需要根据传感器编码方式执行角度展开（unwrap）。但若传感器报告的是多圈连续角度，就不能随意按最短角度修正。

---

## 8. 今日复习卡：最容易混淆的 6 行代码

```cpp
double x = 1.0;          // 一个变量，里面是数值 1.0
double* p = &x;         // 指针 p 保存 x 的地址
const double* input = &x; // 通过 input 只读 x
*p = 2.0;               // 修改 x，现在 x=2.0
// *input = 3.0;        // 编译错误：不能通过只读指针修改
if (p != nullptr) {     // 先判断非空，再访问
    std::cout << *p;    // 打印 2.0
}
```

> 上面片段需要 `#include <iostream>` 才能作为独立程序的一部分编译。**不要只背符号，要能解释“指针保存地址，解引用访问对象”。**

## 9. 今天的学习清单

- [ ] 看懂题 1 的完整代码，理解 `&` 和 `*`。
- [ ] 亲手输入并运行题 1，改成负角度测试。
- [ ] 看懂题 2 的 `.data()` 和两次 `for` 循环。
- [ ] 不看答案写出一个带 `nullptr` 检查的函数。
- [ ] 回答至少 6 个面试追问。
- [ ] 做完练习 2 和练习 4。

**Day 05 预告**：`std::vector` 的常用操作、范围 `for` 与多关节误差范数/报警阈值。让“指针数组”过渡到更安全的现代 C++ 容器写法。

---

## 附录：自动化测试代码

以下文件已包含在学习包的 `tests/day04_tests.cpp` 中，可自行运行：

```cpp
// 通过重命名示例中的 main，直接复用两份独立程序的函数实现。
#define main day04_demo_q1_main
#include "../src/day04_q1_angle_calibration.cpp"
#undef main
#define main day04_demo_q2_main
#include "../src/day04_q2_joint_velocity.cpp"
#undef main

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

bool near(double a, double b, double tol = 1e-10) {
    return std::fabs(a - b) <= tol;
}

int main() {
    constexpr double pi = 3.14159265358979323846;
    double x = 90.0, out = 123.0;
    assert(calibrateAngle(&x, 10.0, &out) && near(out, 80.0 * pi / 180.0));
    x = -170.0;
    assert(calibrateAngle(&x, -10.0, &out) && near(out, -160.0 * pi / 180.0));
    x = 0.0;
    assert(calibrateAngle(&x, 0.0, &out) && near(out, 0.0));
    out = 123.0;
    assert(!calibrateAngle(nullptr, 0.0, &out) && near(out, 123.0));
    assert(!calibrateAngle(&x, 0.0, nullptr));
    x = std::numeric_limits<double>::infinity();
    assert(!calibrateAngle(&x, 0.0, &out) && near(out, 123.0));
    x = 2.0;
    assert(!calibrateAngle(&x, std::numeric_limits<double>::quiet_NaN(), &out));

    double prev[3] = {0.1, -0.2, 1.0};
    double now[3] = {0.15, -0.22, 0.98};
    double vel[3] = {42.0, 42.0, 42.0};
    assert(estimateJointVelocities(now, prev, 3, 0.1, vel));
    assert(near(vel[0], 0.5) && near(vel[1], -0.2) && near(vel[2], -0.2));
    assert(!estimateJointVelocities(nullptr, prev, 3, 0.1, vel));
    assert(!estimateJointVelocities(now, nullptr, 3, 0.1, vel));
    assert(!estimateJointVelocities(now, prev, 3, 0.1, nullptr));
    assert(!estimateJointVelocities(now, prev, 0, 0.1, vel));
    assert(!estimateJointVelocities(now, prev, 3, 0.0, vel));
    assert(!estimateJointVelocities(now, prev, 3, -0.1, vel));
    assert(!estimateJointVelocities(now, prev, 3, std::numeric_limits<double>::infinity(), vel));
    const double old[3] = {42.0, 42.0, 42.0};
    for (int i = 0; i < 3; ++i) vel[i] = old[i];
    now[2] = std::numeric_limits<double>::quiet_NaN();
    assert(!estimateJointVelocities(now, prev, 3, 0.1, vel));
    assert(near(vel[0], 42.0) && near(vel[1], 42.0) && near(vel[2], 42.0));
    now[2] = 0.98;
    std::cout << "PASS: 16 groups of function and boundary tests\n";
}
```

> 本课程用于学习编程与机器人算法原理；实机控制需要额外的传感器标定、实时性验证和安全机制。
