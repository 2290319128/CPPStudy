# 机器人算法 C++ 手撕训练｜Day 02

> **日期：2026-10-14（周三）** · **难度：★☆☆☆☆（零基础）** · **预计学习：45–70 分钟**  
> 今日主题：**函数、形参和实参、返回值、代码复用**  
> 题目 1：封装关节力矩限幅函数 `clampTorque()`  
> 题目 2：封装 P 控制器函数 `computePControl()`，复用题目 1 的限幅函数

---

## 0. 为什么今天要学“函数”？

Day 01 中，我们直接把统计角度和 P 控制器的逻辑写进了 `main()`。这样做可以入门，但在真实机器人程序里，同一个限幅操作可能出现在关节位置控制、速度控制、轨迹跟踪等不同模块中。如果每次都复制一段 `if / else if`，代码会重复，修改时也容易漏掉。

今天我们只引入一个核心思想：**把完成一件具体事情的代码封装成函数，需要时直接调用。** 先学会函数，再逐步学习类、库、控制器模块，最后才能读懂 MPC / WBC / DDP 的大型 C++ 工程。

**今日学习目标**：

1. 能解释 `double clampTorque(double value, double limit)` 的每个部分。
2. 知道什么是函数定义、函数调用、形参、实参和返回值。
3. 能自己写出正负对称的力矩限幅函数。
4. 能在另一个函数中调用它，封装一个 P 控制器。
5. 能在终端独立编译运行两个 C++17 文件，并判断常见边界输入。

### 前置知识回顾（Day 01）

| 已学过的语法 | 本日使用场景 |
| --- | --- |
| `double` | 表示角度、增益和力矩 |
| `if` / `else if` | 判断是否超出上下限 |
| `std::cin` / `std::cout` | 读取测试数据和打印结果 |
| `+ - *` | 计算位置误差与控制输出 |
| `return 0;` | 结束 `main()`；今天扩展到返回计算结果 |

---

# 题目 1｜将力矩限幅写成一个函数

## 1.1 面试题目

实现函数：

```cpp
double clampTorque(double value, double limit);
```

输入未限幅力矩 `value` 和**非负**限幅阈值 `limit`，返回位于 `[-limit, limit]` 内的力矩。要求自己写 `if / else if`，暂时不使用 `std::clamp`。

**输入**：一行两个浮点数 `value limit`，单位均为 N·m。  
**输出**：`限幅后力矩: 数值`。

| 输入 | 输出 | 原因 |
| --- | --- | --- |
| `5 3` | `限幅后力矩: 3` | 5 大于正向上限 3 |
| `-5 3` | `限幅后力矩: -3` | -5 小于负向下限 -3 |
| `1.2 3` | `限幅后力矩: 1.2` | 未超限 |
| `0 0` | `限幅后力矩: 0` | 上下限都为 0 |
| `5 -3` | `限幅值不能为负数` | 非法参数 |

## 1.2 机器人背景

实际执行器的电机力矩、电流或速度命令都有允许范围。假设控制器计算出的期望关节力矩是 5 N·m，但我们在这个简化示例中只允许命令值处于 -3 到 3 N·m，则发送给下游模块的命令应被截断为 3 N·m。

> **重要区分**：软件限幅只是命令处理的一部分，并不等于完整的硬件安全保障；实际机器人还需要驱动器限流、故障保护、状态监测等机制。

## 1.3 数学原理：饱和函数（saturation）

给定原始命令 $u$ 和非负上限 $L\ge0$，定义

$$
\operatorname{sat}(u,L)=
\begin{cases}
 L, & u>L,\\
 -L, & u<-L,\\
 u, & -L\le u\le L.
\end{cases}
$$

也可以写成紧凑形式：

$$
\boxed{u_{\mathrm{cmd}}=\min\bigl(L,\max(-L,u)\bigr)}.
$$

逐步理解 `u = -5, L = 3`：首先 $\max(-3,-5)=-3$，再取 $\min(3,-3)=-3$。对应 C++ 中“判断超限 → 返回边界值”的分支。

### 画在纸上的执行流程

```text
接收 value 和 limit
        │
        ▼
  value > limit ? ──是──> return limit
        │否
        ▼
 value < -limit ? ──是──> return -limit
        │否
        ▼
    return value
```

## 1.4 完整可编译代码（逐行中文注释）

文件名：`01_clamp_torque.cpp`

```cpp
#include <iostream>  // 引入标准输入输出工具 std::cin 和 std::cout

// 函数功能：把力矩限制在 [-limit, limit] 范围内。
// value：未限幅的力矩；limit：非负的最大允许力矩。
// 返回值：经过限幅后的力矩。
double clampTorque(double value, double limit) {
    if (value > limit) {       // 超过正向上限
        return limit;          // 返回上限，并立即结束函数
    } else if (value < -limit) { // 低于负向下限
        return -limit;         // 返回负向下限
    }
    return value;              // 未超限，保持原值
}

int main() {                   // 程序入口
    double torque = 0.0;       // 待限幅的力矩，单位 N·m
    double max_torque = 0.0;   // 最大允许力矩，单位 N·m

    if (!(std::cin >> torque >> max_torque)) { // 检查输入是否成功
        std::cout << "输入无效\n";
        return 1;              // 非零返回码：发生错误
    }
    if (max_torque < 0.0) {    // 限幅阈值不能为负数
        std::cout << "限幅值不能为负数\n";
        return 1;
    }

    double result = clampTorque(torque, max_torque); // 调用函数并接收返回值
    std::cout << "限幅后力矩: " << result << "\n"; // 输出结果
    return 0;                  // 正常结束程序
}
```

### 如何编译运行

Ubuntu 终端进入保存文件的目录：

```bash
g++ -std=c++17 -Wall -Wextra -pedantic 01_clamp_torque.cpp -o clamp
./clamp
```

程序开始等待输入时，输入 `5 3`，然后按回车。你也可以直接执行自动输入：

```bash
printf '5 3\n' | ./clamp
# 预期：限幅后力矩: 3
```

命令解释：`g++` 是编译器；`-std=c++17` 选择 C++17；`-Wall -Wextra -pedantic` 打开常见警告；`-o clamp` 指定可执行程序名字；`./clamp` 表示运行当前目录下的可执行程序。

## 1.5 今天最重要的语法：函数定义

看这一行：

```cpp
double clampTorque(double value, double limit)
```

逐个拆开：

| 代码片段 | 名称 | 含义 |
| --- | --- | --- |
| 第一个 `double` | 返回值类型 | 函数最终返回一个双精度浮点数 |
| `clampTorque` | 函数名 | 我们自己给功能起的名字 |
| `( ... )` | 参数列表 | 调用时需要提供的数据 |
| `double value` | 第 1 个形参 | 函数内部用 `value` 表示原始力矩 |
| `double limit` | 第 2 个形参 | 函数内部用 `limit` 表示限幅值 |
| `{ ... }` | 函数体 | 具体计算过程 |
| `return limit;` | 返回语句 | 把 `limit` 交回调用位置，并立刻结束函数 |

### 形参与实参有什么区别？

```cpp
// 函数定义：value、limit 是“形参”
double clampTorque(double value, double limit) {
    // ...
    return value;
}

// 函数调用：torque、max_torque 是“实参”
double result = clampTorque(torque, max_torque);
```

调用发生时，**当前这个示例采用按值传递**：`torque` 的数值被复制到形参 `value`，`max_torque` 的数值被复制到形参 `limit`。在函数内部改变形参，不会自动改变外面的原变量。后面会专门学习“引用传参”和“指针传参”。

### `return` 与 `std::cout` 不一样

```cpp
return limit;             // 让调用者得到一个值
std::cout << limit;       // 把值打印到终端
```

函数可以返回数值但不打印；也可以打印而不返回数值。面试手撕通常更偏好**计算函数返回结果、`main()` 负责输入输出**的结构，方便测试和复用。

### 为什么要把函数定义写在 `main()` 前面？

编译器处理到 `main()` 中的 `clampTorque(...)` 时，需要先知道这个函数存在。对于当前入门代码，把完整定义写在 `main()` 前面最直观。以后会学函数声明、头文件 `.h` 和实现文件 `.cpp`。

## 1.6 手工追踪执行过程

以输入 `-5 3` 为例：

1. `main()` 读取 `torque=-5` 和 `max_torque=3`。
2. 检查 `max_torque < 0`，结果为假。
3. 执行 `clampTorque(-5, 3)`。
4. 进入函数：`value=-5, limit=3`。
5. `value > limit` 即 `-5 > 3` 为假。
6. `value < -limit` 即 `-5 < -3` 为真。
7. 执行 `return -limit;`，把 `-3` 交回 `main()`。
8. `result=-3`，输出 `限幅后力矩: -3`。

## 1.7 边界情况、错误和复杂度

- `value == limit` 或 `value == -limit`：直接返回原值，正确。
- `limit == 0`：输出应为 0。
- `limit < 0`：没有合理的对称区间，当前程序在 `main()` 中拒绝。
- `value` 和 `limit` 类型用 `double`：允许小数；如果用 `int` 会丢失小数部分。
- 如果输入字母等无法解析的数据：通过 `if (!(std::cin >> ...))` 检测输入失败。
- `NaN` / 无穷大：本课暂不处理；真实控制系统应增加 `std::isfinite` 检查，禁止非有限命令进入执行器。

**时间复杂度**：$O(1)$，固定次数的比较；**额外空间复杂度**：$O(1)$。

### 常见错误

```cpp
if (value < limit) return limit;  // 错：正向判断应该是 >
if (value < -limit) return limit; // 错：负向超限应该返回 -limit
clampTorque(torque, limit);       // 错：如果没有接收返回值，torque 不会自动改变
```

---

# 题目 2｜用函数封装一维 P 控制器

## 2.1 面试题目

编写一个函数：

```cpp
double computePControl(double target, double current,
                       double kp, double max_torque);
```

计算机器人关节位置误差、比例控制力矩，并**调用题目 1 的 `clampTorque()` 函数**对输出进行限幅。

**输入**：`target current kp max_torque`，目标和当前角度单位为 rad，`kp` 单位为 N·m/rad，力矩单位为 N·m。为方便入门，本题假设 `kp >= 0`、`max_torque >= 0`。

**示例**：

```text
输入：1.2 0.8 10 3
输出：控制力矩: 3
```

## 2.2 数学原理与逐步推导

**第一步：定义位置误差**

$$
\boxed{e=q_{\mathrm{ref}}-q}.
$$

`target` 对应 $q_{\mathrm{ref}}$；`current` 对应 $q$。

**第二步：P 控制器输出未限幅的命令**

$$
\boxed{\tau_{\mathrm{raw}}=K_p e=K_p(q_{\mathrm{ref}}-q)}.
$$

**第三步：输出饱和处理**

$$
\boxed{\tau_{\mathrm{cmd}}=\operatorname{sat}(\tau_{\mathrm{raw}},\tau_{\max})}.
$$

合并得到：

$$
\boxed{\tau_{\mathrm{cmd}}=\min\left(\tau_{\max},\max\left(-\tau_{\max},K_p(q_{\mathrm{ref}}-q)\right)\right)}.
$$

### 数值例子

给定 $q_{\mathrm{ref}}=1.2\,\mathrm{rad}$、$q=0.8\,\mathrm{rad}$、$K_p=10\,\mathrm{N\cdot m/rad}$、$\tau_{\max}=3\,\mathrm{N\cdot m}$：

$$
e=1.2-0.8=0.4\,\mathrm{rad},
$$

$$
\tau_{\mathrm{raw}}=10\times0.4=4\,\mathrm{N\cdot m},
$$

$$
\tau_{\mathrm{cmd}}=\min(3,\max(-3,4))=3\,\mathrm{N\cdot m}.
$$

**物理解释**：目标角度比当前角度大，正比例增益产生正向控制命令；但期望力矩超出允许范围，因此发送限幅后的 3 N·m。

> 这里的 P 控制器只是**一维简化控制律**。它没有引入关节惯量、重力、速度阻尼、摩擦和闭环稳定性分析。真实机器人更常使用 PD + 重力补偿、逆动力学或更高级的控制器，后续会逐步学习。

## 2.3 函数调用关系：一个函数调用另一个函数

```text
main()
  │
  ├─ 读取 target / current / kp / max_torque
  │
  └─ computePControl(target, current, kp, max_torque)
         │
         ├─ error = target - current
         ├─ raw_torque = kp * error
         │
         └─ clampTorque(raw_torque, max_torque)
                  │
                  └─ return 限幅后的力矩
         │
         └─ return 最终控制力矩
  │
  └─ 打印结果
```

**核心价值**：限幅逻辑只写一次，两个函数分工明确。后续要做 PD 控制时，可以复用 `clampTorque()`，不需要复制限幅代码。

## 2.4 完整可编译代码（逐行中文注释）

文件名：`02_p_controller_function.cpp`

```cpp
#include <iostream>  // 提供标准输入输出工具

// 复用题目 1：限制控制力矩的范围。
double clampTorque(double value, double limit) {
    if (value > limit) {        // 大于上限，返回上限
        return limit;
    } else if (value < -limit) { // 小于下限，返回负上限
        return -limit;
    }
    return value;               // 未超限，返回原值
}

// 把 P 控制器封装成函数，便于重复调用。
// target：目标关节角度；current：当前关节角度（均为 rad）。
// kp：比例增益（N·m/rad）；max_torque：最大力矩（N·m）。
double computePControl(double target, double current,
                       double kp, double max_torque) {
    double error = target - current;    // 计算位置误差 e
    double raw_torque = kp * error;     // 未限幅力矩 τ = Kp * e
    return clampTorque(raw_torque, max_torque); // 复用函数并返回结果
}

int main() {                            // 程序从这里开始
    double target = 0.0;                // 目标角度 rad
    double current = 0.0;               // 当前角度 rad
    double kp = 0.0;                    // 比例增益 N·m/rad
    double max_torque = 0.0;            // 最大力矩 N·m

    if (!(std::cin >> target >> current >> kp >> max_torque)) {
        std::cout << "输入无效\n";      // 输入不是四个数字
        return 1;
    }
    if (kp < 0.0 || max_torque < 0.0) { // || 表示“或者”
        std::cout << "kp 和 max_torque 不能为负数\n";
        return 1;
    }

    double torque = computePControl(target, current, kp, max_torque);
    std::cout << "控制力矩: " << torque << "\n";
    return 0;                           // 正常结束
}
```

### 编译与运行

```bash
g++ -std=c++17 -Wall -Wextra -pedantic 02_p_controller_function.cpp -o pcontrol
./pcontrol
```

输入 `1.2 0.8 10 3` 后输出：

```text
控制力矩: 3
```

或者用管道自动输入：

```bash
printf '1.2 0.8 10 3\n' | ./pcontrol
```

## 2.5 必须看懂的三行代码

```cpp
double error = target - current;
double raw_torque = kp * error;
return clampTorque(raw_torque, max_torque);
```

第一行建立一个局部变量 `error`；第二行使用它计算比例控制输出；第三行先调用 `clampTorque`，得到它的返回值，再把这个返回值作为 `computePControl` 的返回值交给 `main()`。

把第三行拆开写，含义完全一样：

```cpp
double limited = clampTorque(raw_torque, max_torque);
return limited;
```

### 变量作用域：为什么两个函数里都能出现 `value` 等变量？

函数内部定义的变量只在它自己的作用域中使用。`computePControl` 内的 `error` 不会直接成为 `main()` 中的变量。初学时可以把每个函数想象成一个独立的小盒子：**输入参数进入盒子，返回值从盒子出来**。

### `if (kp < 0.0 || max_torque < 0.0)` 是什么意思？

`||` 表示逻辑“或”：只要增益为负**或者**力矩上限为负，就拒绝输入。这里检查是为了匹配本题的参数假设，不意味着所有控制器的所有反馈增益都只能为正。

## 2.6 自己推演两组数据

| `target current kp max_torque` | 误差 $e$ | 原始力矩 | 最终力矩 |
| --- | ---: | ---: | ---: |
| `1.2 0.8 10 3` | 0.4 | 4 | 3 |
| `0 1 2 3` | -1 | -2 | -2 |
| `0 1 10 3` | -1 | -10 | -3 |
| `0 0 0 0` | 0 | 0 | 0 |

**时间复杂度**：$O(1)$；**额外空间复杂度**：$O(1)$。

## 2.7 常见错误与工程注意

1. **误差方向写反**：`current - target` 会改变反馈方向。本课规定 $e=target-current$。
2. **忘记接收返回值**：`clampTorque(raw_torque, max_torque);` 不会自动改写 `raw_torque`。
3. **混用角度与弧度**：本题全部采用 rad；若输入的是角度值，需要先转换，不能直接当 rad。
4. **误把 P 控制当完整动力学控制**：`kp * error` 是简化控制律，不保证任何实际机械臂系统在所有情况下都稳定。
5. **参数校验缺失**：本题验证负值和读取失败，尚未处理 `NaN`、无穷大、关节软限位及执行器通讯故障。
6. **命名含糊**：`target`、`current`、`max_torque` 比 `a`、`b`、`c` 更容易维护。

---

# 3. C++ 基础知识点汇总

| 知识点 | 最简记忆 | 本日例子 |
| --- | --- | --- |
| 函数定义 | 规定功能怎样实现 | `double clampTorque(...) { ... }` |
| 函数调用 | 使用已定义的功能 | `clampTorque(5, 3)` |
| 返回值类型 | 函数交回的数据类型 | `double` |
| 形参 | 函数定义里的变量名 | `value`, `limit` |
| 实参 | 调用时提供的具体数据 | `torque`, `max_torque` |
| 按值传递 | 把数值复制给形参 | `double value` |
| `return` | 交回结果并结束当前函数 | `return -limit;` |
| 局部变量 | 在自己的作用域中有效 | `double error` |
| `||` | 逻辑或 | `kp < 0 || max_torque < 0` |
| `!` | 逻辑非 | `!(std::cin >> ...)` |
| `main()` | 程序入口 | `int main() { ... }` |

## 3.1 迷你例子：函数返回值和打印的区别

```cpp
#include <iostream>

double square(double x) { // 返回 double，接收一个 double 参数
    return x * x;         // 只计算并返回，不打印
}

int main() {
    double y = square(3.0); // 调用 square，得到 9.0
    std::cout << y << "\n"; // main 负责打印
    return 0;
}
```

思考：把 `square(3.0)` 改成 `square(-2.0)` 会得到多少？答案：4。

## 3.2 从 C++ 基础到机器人控制软件

```text
今天：两个独立函数
  ↓
后续：一个可配置的 PID 类
  ↓
后续：控制器读取状态向量、输出关节命令
  ↓
后续：Eigen 矩阵 + 运动学 + 轨迹规划
  ↓
后续：LQR / QP / MPC / DDP / WBC 的算法模块
```

今天最关键的不是代码行数，而是养成“**计算逻辑写成函数，输入输出与计算分离**”的习惯。

---

# 4. 面试官追问（附参考答案）

**Q1：为什么不把限幅代码直接复制到 P 控制器？**  
A：提取函数可以减少重复，提高可读性，修改边界规则时只改一处，单元测试也更方便。

**Q2：`return` 和 `cout` 有什么区别？**  
A：`return` 把值交给调用者并结束函数；`cout` 只负责向终端输出。计算函数通常返回结果而不是直接打印。

**Q3：形参和实参有什么区别？**  
A：形参出现在函数定义中，如 `value`；实参出现在调用位置，如 `torque`。本题采用按值传递，函数接收到数值副本。

**Q4：如果把 `value` 改成 `100`，外部的 `torque` 会变成 100 吗？**  
A：不会。`double value` 是按值传递；修改形参不影响外部变量。后面学习引用和指针后可以讨论如何修改外部对象。

**Q5：为什么函数有时写 `double`，有时写 `void`？**  
A：`double` 表示函数返回一个双精度数值；`void` 表示不返回值。本课计算函数都需要把结果交回调用者，因此返回 `double`。

**Q6：为什么要对控制力矩限幅？**  
A：因为执行器存在物理与控制上的命令范围。限幅可以避免软件命令超出设定区间，但不能替代硬件安全保护。

**Q7：本题的时间复杂度和空间复杂度是多少？**  
A：两个函数都只做常数次计算与判断，时间 $O(1)$，额外空间 $O(1)$。

**Q8：为什么实际机器人还需要 PD 控制？**  
A：仅有 P 项可能缺少足够的速度阻尼；PD 引入与速度误差相关的项，可改善动态响应，但具体稳定性仍依赖系统模型与增益设计。

**Q9：能直接用 `std::clamp` 吗？**  
A：C++17 中 `<algorithm>` 提供 `std::clamp(value, low, high)`，但本课为了练习函数和分支逻辑，要求先自己实现。`std::clamp` 需要满足 `low <= high`。

**Q10：为什么机器人算法程序重视函数接口？**  
A：清晰的输入、输出和职责边界方便测试、复用、调试及模块集成；将来在 MPC/WBC 中，动力学计算、代价函数、约束和求解器也都需要明确接口。

---

# 5. 课后练习（先做，再看答案）

## 练习 A（5 分钟）：预测输出

不运行程序，写出下列函数调用的返回值：

```cpp
clampTorque(2.0, 3.0);
clampTorque(-8.0, 5.0);
clampTorque(0.0, 0.0);
computePControl(0.5, 1.0, 4.0, 3.0);
```

## 练习 B（10 分钟）：写一个误差函数

自己实现：

```cpp
double positionError(double target, double current);
```

要求返回 `target - current`，然后在 `computePControl()` 中调用它。

## 练习 C（10 分钟）：不使用 `else if` 写限幅

只使用两个 `if` 和一个 `return` 实现相同功能，验证所有边界输入都得到一致结果。

## 练习 D（思考题）：加一个角度单位转换函数

如果输入是角度制，写一个函数将其转换成弧度：

$$
q_{\mathrm{rad}}=q_{\mathrm{deg}}\frac{\pi}{180}.
$$

本题不要求使用任何第三方库；请先尝试用常量 `3.141592653589793`。

---

# 6. 课后练习参考答案

### 练习 A

```text
2
-5
0
-2
```

最后一题：误差 $0.5-1=-0.5$，原始力矩 $4\times(-0.5)=-2$，没有触发限幅。

### 练习 B

```cpp
double positionError(double target, double current) {
    return target - current;
}

// computePControl 函数体内可以改成：
double error = positionError(target, current);
double raw_torque = kp * error;
return clampTorque(raw_torque, max_torque);
```

### 练习 C

```cpp
double clampTorque(double value, double limit) {
    if (value > limit) {
        value = limit;
    }
    if (value < -limit) {
        value = -limit;
    }
    return value;
}
```

本题前提仍然是 `limit >= 0`。

### 练习 D

```cpp
double degreesToRadians(double degrees) {
    const double pi = 3.141592653589793;
    return degrees * pi / 180.0;
}
```

`const` 表示这里的 `pi` 初始化后不再修改。`const` 的系统用法安排在后续课程。

---

# 7. 今天如何验收自己是否学会

请尝试**不看答案**，在空白 `.cpp` 文件中写出：

- [ ] `#include <iostream>` 和 `int main()`。
- [ ] 一个接收两个 `double`、返回一个 `double` 的限幅函数。
- [ ] 正向超限、负向超限、正常范围三个分支。
- [ ] 一个 `computePControl()` 函数，在里面调用 `clampTorque()`。
- [ ] 使用 `std::cin` 读入四个数，使用 `std::cout` 打印结果。
- [ ] 能解释 `return`、形参、实参、按值传递。
- [ ] 能说出这两个函数的时间和空间复杂度都是 $O(1)$。

**建议练习顺序**：先阅读 10 分钟 → 手动输入并运行 15 分钟 → 关掉讲义自己重写 15 分钟 → 完成面试追问与练习 10–30 分钟。不要只看懂，一定要亲手编译。

---

# 8. 明日预告｜Day 03

**计划主题：函数参数传递、`const` 和引用的初步认识**。仍以机器人关节状态为背景，先写简单的关节误差计算，再比较按值传递和引用传递的行为，为后续 `std::vector`、Eigen 和大型控制器接口打基础。

> **每日学习路线**：C++ 基础 → 函数/引用/指针/类/STL → 机器人数学与运动学 → PID/轨迹规划/动力学 → LQR/QP/MPC/DDP/WBC。循序渐进，不为了追求难度而跳过基础。

---

**附：本讲义对应的两个独立源文件**：`01_clamp_torque.cpp`、`02_p_controller_function.cpp`。所有代码使用 C++17 编译，并通过文中的代表性示例测试。完整讲义为 UTF-8 编码，可在 VS Code、Typora 或 Obsidian 中阅读。
