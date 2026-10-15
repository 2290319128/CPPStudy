# 机器人运动控制 C++17 每日手撕训练｜Day 03

> **2026-10-15｜零基础阶段·第 3 天**  
> **主题：按值传递、`const`、引用 `&`｜机器人关节误差计算**  
> 建议用时：60～90 分钟。先读题并尝试自己写，再看完整代码和解释。

## 0. 今日学习目标与前后衔接

- **Day 01**：变量、`for`、`if`、`vector`；关节角统计与 P 控制器。
- **Day 02**：函数、参数、返回值；封装力矩限幅函数并复用到 P 控制器。
- **Day 03（今天）**：理解函数参数是“复制一份”还是“使用同一个对象”；掌握 `const` 和 `&`；计算单关节及多关节位置误差。
- **Day 04（预告）**：指针 `*` 与引用 `&` 的区别，结合传感器数据缓存和空指针检查。

今天两道题：

| 题目 | 机器人任务 | 主要 C++ 技能 | 难度 |
|---|---|---|---|
| 1. 单关节误差与按值传递 | 计算 $e=q_{ref}-q$，验证函数内部修改副本不影响外部 | `double`、函数、`const` 值参数、`return` | ★☆☆☆☆ |
| 2. 多关节误差向量 | 计算 $\mathbf e=\mathbf q_{ref}-\mathbf q$ | `const std::vector<double>&`、输出引用、`bool`、`resize` | ★★☆☆☆ |

**今日一句话重点**：`double x` 是一份副本；`const std::vector<double>& x` 是对原向量的只读引用；`std::vector<double>& out` 是可以修改原向量的引用。

---

## 1. 预备知识：函数传参到底发生了什么？

### 1.1 按值传递：`double x`

```cpp
void change(double x) {
    x = 10.0;
}

int main() {
    double a = 3.0;
    change(a);      // 调用时复制 a 的值给 x
    // 这里 a 仍然是 3.0
}
```

1. `a` 是 `main` 中的变量，值为 3.0。
2. 调用 `change(a)` 时，形参 `x` 得到一份独立的数值副本。
3. `x = 10.0` 只修改副本；调用者的 `a` 不变。
4. 对 `double` 这样的小对象，按值传递通常简单且高效。

### 1.2 `const` 值参数：`const double x`

```cpp
double error(const double target, const double current) {
    // target = 0.0;  // 错误：const 形参不可在函数内重新赋值
    return target - current;
}
```

`const` 限制的是**函数体内对该形参的修改**，并不意味着“按引用传递”。`const double x` 依然是按值传递，仍会复制一个 `double`。`const` 也不会让调用者传入的变量自动变成 `const`。

### 1.3 引用：`T&` 与 `const T&`

```cpp
void setZero(double& x) { x = 0.0; }      // 修改调用者的原变量
void print(const double& x) {            // 只读引用，不允许通过 x 修改
    std::cout << x << '\n';
}
```

- `double& x`：`x` 是对调用者变量的引用，修改 `x` 就是在修改原变量。
- `const double& x`：不复制该对象，且禁止**通过这个引用**修改对象。
- `const` 引用不等于对象在所有地方都不可能变化：如果原对象不是 `const`，其他代码仍可能通过其他途径修改它。
- 对 `double`，通常优先按值传递；对 `std::vector` 等较大的容器，通常优先使用 `const &` 做只读输入。

### 1.4 三种常见签名对照

| 形参写法 | 是否复制整个对象 | 能否通过该形参修改调用者对象 | 典型用途 |
|---|---|---|---|
| `double x` | 复制一个 `double` | 不能 | 小标量输入 |
| `const double x` | 复制一个 `double` | 不能 | 明确函数内部不修改局部形参 |
| `std::vector<double> v` | 复制整个向量元素 | 不能 | 需要独立副本时 |
| `const std::vector<double>& v` | 不复制向量元素 | 不能 | 大型只读输入 |
| `std::vector<double>& v` | 不复制向量元素 | 能 | 输出参数、原地修改 |

> 注意：引用变量通常必须在定义时绑定到对象；引用并非可随意重新指向别处的指针。不要把引用理解为“复制的数组”。

---

## 2. 题目 1｜单关节位置误差 + 按值传递实验

### 2.1 题目要求

机器人某关节有目标角度 `target`、当前角度 `current`，另给一个演示用的偏移 `offset`：

1. 写 `addOffsetByValue(double angle, double offset)`，在函数内部把 `angle` 加上 `offset` 并输出。
2. 回到 `main` 后输出原来的 `current`，观察它**没有改变**。
3. 写 `computeJointError(const double target, const double current)`，返回位置误差。
4. 输入三个数字，按示例输出。

**输入格式**：一行三个浮点数 `target current offset`，角度单位统一为 rad。

**输入示例**：

```text
1.2 0.8 0.5
```

**输出示例**：

```text
函数内的角度: 1.300
main中的角度: 0.800
关节位置误差: 0.400
```

### 2.2 机器人数学原理

关节位置误差定义为：

$$
e = q_{ref} - q
$$

示例：

$$
q_{ref}=1.2\ \mathrm{rad},\quad q=0.8\ \mathrm{rad}
$$

$$
e=1.2-0.8=0.4\ \mathrm{rad}
$$

这与 P 控制器的第一步完全一致：

$$
\tau=K_pe
$$

今天先把误差计算写成可复用函数；以后它会用于 PD、PID、MPC 状态跟踪误差等。

**重要物理边界**：对于连续旋转且角度按 $2\pi$ 周期等价的关节，直接相减可能不是最短角度误差；今天假设角度已在同一连续坐标下表达，不处理跨 $\pm\pi$ 的环绕误差。后续会专门学习角度归一化。

### 2.3 完整、可独立编译的 C++17 代码（逐行中文注释）

保存为 `day03_q1_value_const.cpp`：

```cpp
#include <iomanip>   // std::fixed 和 std::setprecision：控制输出格式
#include <iostream>  // std::cin 和 std::cout：标准输入输出

// 函数一：按值传递。angle 是调用者传入数值的一份副本。
// 在函数内修改 angle，不会修改 main 中的原变量。
void addOffsetByValue(double angle, double offset) {
    angle = angle + offset;  // 只修改函数内部的副本
    std::cout << "函数内的角度: " << angle << '\n';
}

// 函数二：计算单关节的位置误差 e = q_ref - q。
// const 限定本函数不能修改这两个形参（它们仍然是按值传递的副本）。
// double 是返回值类型：调用者能接收计算出的误差。
double computeJointError(const double target, const double current) {
    return target - current;  // 计算误差并返回给调用处
}

int main() {  // 程序入口
    double target = 0.0;  // 目标关节角，单位 rad
    double current = 0.0; // 当前关节角，单位 rad
    double offset = 0.0;  // 演示按值传递的角度偏移，单位 rad

    // 依次输入：目标角度 当前角度 演示偏移量
    if (!(std::cin >> target >> current >> offset)) {
        std::cerr << "输入错误：请输入三个数字\n";
        return 1;  // 非零返回值表示程序未正常完成
    }

    std::cout << std::fixed << std::setprecision(3); // 保留 3 位小数
    addOffsetByValue(current, offset);  // 传入 current 的副本
    std::cout << "main中的角度: " << current << '\n'; // 原值未改变

    const double error = computeJointError(target, current); // 接收返回值
    std::cout << "关节位置误差: " << error << '\n';
    return 0;  // 正常退出
}
```

### 2.4 按执行顺序逐步拆解

**第一步：引入头文件**

- `#include <iostream>`：使用 `std::cin` 输入、`std::cout` 输出、`std::cerr` 输出错误信息。
- `#include <iomanip>`：`std::fixed` 采用固定小数格式，`std::setprecision(3)` 显示三位小数。
- `std::` 是标准库命名空间限定符。入门阶段推荐保留，避免名称冲突。

**第二步：定义函数**

```cpp
void addOffsetByValue(double angle, double offset)
```

- `void`：没有返回值。
- `addOffsetByValue`：函数名，意为“按值传递并增加偏移”。
- `double angle`：第一个形参，是传入数值的副本。
- `double offset`：第二个形参，也是副本。
- `{ ... }`：函数体，定义具体执行步骤。

**第三步：修改副本**

```cpp
angle = angle + offset;
```

例如 `angle=0.8`、`offset=0.5`，函数内部变成 `1.3`。但调用者的 `current` 仍然是 `0.8`。

**第四步：定义带返回值的函数**

```cpp
double computeJointError(const double target, const double current) {
    return target - current;
}
```

- 最左边 `double` 是**返回值类型**。
- `const double target` 仍然是**按值传递**，`const` 只是禁止修改该形参。
- `return` 把结果交给调用位置，并结束当前函数执行。

**第五步：调用函数**

```cpp
addOffsetByValue(current, offset);
const double error = computeJointError(target, current);
```

- 第一行调用无返回值的函数；`current` 的值不变。
- 第二行调用有返回值的函数，并用 `error` 保存结果。
- `const double error` 表示接下来不能给 `error` 重新赋值。

**第六步：检查输入**

```cpp
if (!(std::cin >> target >> current >> offset)) {
    std::cerr << "输入错误：请输入三个数字\n";
    return 1;
}
```

- `std::cin >> ...` 依次读取三个数字；失败时流状态可用于判断。
- `!` 是逻辑非：读取失败时条件为真。
- `return 1` 从 `main` 返回非零状态，提示运行失败。
- `\n` 是换行符，不等于字符串 `"/n"`。

### 2.5 编译运行

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic day03_q1_value_const.cpp -o q1
./q1
```

Windows MinGW 环境下可编译为 `q1.exe`，再运行它。

### 2.6 边界测试

| 输入 | 应有结果（关节误差） | 为什么测试 |
|---|---:|---|
| `1.2 0.8 0.5` | `0.400` | 正常输入 |
| `-0.5 -0.2 0.1` | `-0.300` | 负角度 |
| `0 0 0` | `0.000` | 零误差 |
| `2 5 -1` | `-3.000` | 当前角度大于目标 |
| `1 abc 2` | 程序报输入错误、返回非零 | 输入校验 |

**复杂度**：两个函数都执行固定次数的运算，时间 $O(1)$，额外空间 $O(1)$。

### 2.7 常见错误

1. **误以为 `double angle` 会修改原变量**：它是副本，除非使用引用或指针等方式，否则调用者的原值不变。
2. **误以为 `const double` 就是 `const double&`**：前者按值复制，后者按引用传递。
3. **把 `=` 写成 `==`**：`=` 赋值；`==` 比较相等。
4. **忘记 `return`**：非 `void` 函数需要在正常返回路径提供匹配的返回值。
5. **混用角度制和弧度制**：机器人算法中务必约定单位，尤其后续 `sin`、`cos` 默认接收弧度。

---

## 3. 题目 2｜多关节误差向量 + `const` 引用与输出引用

### 3.1 题目要求

给定机器人 $N$ 个关节的目标角度和当前角度：

$$
\mathbf q_{ref}=[q_{ref,1},\ldots,q_{ref,N}]^T,\quad
\mathbf q=[q_1,\ldots,q_N]^T
$$

计算每个关节的误差 $e_i=q_{ref,i}-q_i$。

编写：

```cpp
bool computeJointErrors(const std::vector<double>& target,
                        const std::vector<double>& current,
                        std::vector<double>& errors);
```

要求：

- `target`、`current` 为**只读输入**，不复制整个向量；
- `errors` 为**输出引用**，函数直接修改调用者的结果容器；
- 两个输入向量长度不相同时，返回 `false`；长度相同时计算并返回 `true`；
- 主程序从标准输入读取 `n`、`n` 个目标角、`n` 个当前角，输出误差。

**输入示例**：

```text
3
1.2 0.0 -0.5
1.0 0.1 -0.2
```

**输出示例**：

```text
关节误差(rad): 0.200 -0.100 -0.300
```

### 3.2 数学原理逐项展开

$$
\mathbf e=\mathbf q_{ref}-\mathbf q
$$

对于 3 关节示例：

$$
\mathbf q_{ref}=\begin{bmatrix}1.2\\0.0\\-0.5\end{bmatrix},\qquad
\mathbf q=\begin{bmatrix}1.0\\0.1\\-0.2\end{bmatrix}
$$

逐项相减：

$$
\begin{aligned}
e_1 &= 1.2-1.0 = 0.2,\\
e_2 &= 0.0-0.1 = -0.1,\\
e_3 &= -0.5-(-0.2) = -0.3.
\end{aligned}
$$

所以：

$$
\boxed{\mathbf e=\begin{bmatrix}0.2\\-0.1\\-0.3\end{bmatrix}\ \mathrm{rad}}
$$

后续 PID 可以逐关节使用 $\tau_i=K_{p,i}e_i$；MPC 中的状态跟踪项常包含 $\mathbf e^TQ\mathbf e$。**但今天不引入矩阵库，也不求解优化问题。**

### 3.3 完整、可独立编译的 C++17 代码（逐行中文注释）

保存为 `day03_q2_const_ref.cpp`：

```cpp
#include <iomanip>  // std::fixed、std::setprecision
#include <iostream> // std::cin、std::cout、std::cerr
#include <vector>   // std::vector：存储多个关节的数据

// target 和 current：const 引用，只读访问，不复制整份向量。
// errors：非 const 引用，直接把计算结果写入调用者提供的向量。
// bool：成功返回 true；输入长度不匹配时返回 false。
bool computeJointErrors(const std::vector<double>& target,
                        const std::vector<double>& current,
                        std::vector<double>& errors) {
    if (target.size() != current.size()) { // 两组关节数据必须等长
        return false;                      // 长度不匹配，停止计算
    }

    errors.resize(target.size());         // 让输出向量拥有 n 个元素
    for (std::size_t i = 0; i < target.size(); ++i) {
        errors[i] = target[i] - current[i]; // e_i = q_ref,i - q_i
    }
    return true;                           // 计算成功
}

int main() {
    int n = 0;                             // 关节数量
    if (!(std::cin >> n) || n <= 0 || n > 100000) {
        std::cerr << "关节数量必须是 1 到 100000 的整数\n";
        return 1;
    }

    std::vector<double> target(n);         // 目标关节角向量
    std::vector<double> current(n);        // 当前关节角向量
    std::vector<double> errors;            // 输出向量，开始时为空

    for (int i = 0; i < n; ++i) {         // 读取 n 个目标角度
        if (!(std::cin >> target[i])) {
            std::cerr << "目标角度输入不足或格式错误\n";
            return 1;
        }
    }
    for (int i = 0; i < n; ++i) {         // 读取 n 个当前角度
        if (!(std::cin >> current[i])) {
            std::cerr << "当前角度输入不足或格式错误\n";
            return 1;
        }
    }

    if (!computeJointErrors(target, current, errors)) { // ! 表示逻辑非
        std::cerr << "错误：目标与当前关节向量长度不一致\n";
        return 1;
    }

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "关节误差(rad):";
    for (double e : errors) {             // 范围 for：逐个访问误差
        std::cout << ' ' << e;
    }
    std::cout << '\n';
    return 0;
}
```

### 3.4 函数签名逐字符解释

```cpp
bool computeJointErrors(const std::vector<double>& target,
                        const std::vector<double>& current,
                        std::vector<double>& errors)
```

- `bool`：返回类型只有 `true` 或 `false`，这里表示计算成功或失败。
- `std::vector<double>`：存放 `double` 元素的可变长度容器。
- `const`：禁止通过该形参修改向量。
- `&`：引用，**不会按值复制整份向量**。
- `target`、`current`：输入向量，仅读取。
- `errors`：输出向量；非 `const` 引用，允许改变它的元素和长度。
- 如果写成 `std::vector<double> errors`，函数内更新的是副本，调用者的向量不会自动得到结果。

### 3.5 关键语句详细解释

**检查长度**：

```cpp
if (target.size() != current.size()) {
    return false;
}
```

- `.size()` 返回元素个数，类型为无符号整数 `std::size_t`。
- `!=` 表示不等于。
- 长度不相等不能逐项相减，否则可能越界；本函数选择返回 `false`。
- 因为在修改 `errors` 之前检查，失败时原 `errors` 保持不变。

**为输出分配元素**：

```cpp
errors.resize(target.size());
```

- `resize(n)` 把向量的**元素数量**调整为 `n`。
- 新增的 `double` 元素会被值初始化为 0。
- `resize` 与 `reserve` 不同：`reserve(n)` 只保证容量至少为 `n`，不会把 `.size()` 变成 `n`，不能据此直接访问 `errors[i]`。

**逐关节计算**：

```cpp
for (std::size_t i = 0; i < target.size(); ++i) {
    errors[i] = target[i] - current[i];
}
```

- `std::size_t` 适合用于容器下标，避免有符号与无符号比较警告。
- `i=0` 从第 0 个关节开始。
- `i<target.size()` 避免访问末尾之后的元素。
- `++i` 让下标增加 1。
- `errors[i]` 直接写入调用者的输出向量。

**范围 `for` 循环**：

```cpp
for (double e : errors) {
    std::cout << ' ' << e;
}
```

逐个取出 `errors` 的元素并复制给局部变量 `e`，仅用于打印。如果想原地修改元素，才需要 `for (double& e : errors)`。

**输入边界**：

```cpp
if (!(std::cin >> n) || n <= 0 || n > 100000)
```

- `||` 是逻辑或，只要任一条件成立，就拒绝输入。
- 这里的 `100000` 是演示程序人为设置的保护上限，**不是机器人物理关节数上限**。
- 两个读取循环分别检查每个输入是否有效，避免未成功读取时继续计算。

### 3.6 编译运行

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic day03_q2_const_ref.cpp -o q2
./q2
```

在终端输入：

```text
3
1.2 0.0 -0.5
1.0 0.1 -0.2
```

得到：

```text
关节误差(rad): 0.200 -0.100 -0.300
```

### 3.7 边界测试与接口测试

| 场景 | 预期行为 | 目的 |
|---|---|---|
| `n=1`，目标 2，当前 1 | 输出 `1.000` | 单关节也适用 |
| `n=3`，两组都为 0 | 输出三个 `0.000` | 零误差 |
| `n=3`，包含负角度 | 正确逐项相减 | 符号正确 |
| `n=0` 或 `n<0` | 主程序报错并返回非零 | 无效长度 |
| 数据数量不足 | 主程序报错并返回非零 | 输入健壮性 |
| 函数直接收到两个不同长度的向量 | 返回 `false`，不修改 `errors` | 函数接口安全 |
| 输入向量不被修改 | 原 `target/current` 保持不变 | `const` 引用语义 |

**复杂度分析**：设关节数为 $N$。

- 长度比较：$O(1)$。
- `errors.resize(N)`：最坏需要初始化/搬移 $O(N)$ 个元素。
- 逐关节计算：$O(N)$。
- **总时间复杂度 $O(N)$**；结果向量存储需要 $O(N)$ 空间。若把已存在的结果缓冲区视为调用者提供的空间，循环本身额外使用 $O(1)$ 临时空间。
- 使用 `const &` 避免把输入的两份 `vector` 各复制一遍，减少不必要的时间和内存开销。

### 3.8 机器人实际工程中的注意事项

1. **不要在实时控制循环里反复动态扩容**：`resize` 在容量不足时可能分配内存，真实高频控制代码一般在初始化阶段预分配好数据缓冲区；今天为学习接口而保留 `resize`。
2. **位置误差并不总能直接相减**：旋转关节的角度环绕、关节方向定义和编码器零位都需要统一处理。
3. **输入和输出维度要一致**：机器人自由度和控制向量维度必须明确。
4. **不要错误理解 `const &`**：它不允许通过该引用修改向量，但不会保证程序其他地方绝对不能修改原向量。
5. **浮点数误差**：`0.1 + 0.2` 不一定在二进制浮点表示中严格等于 `0.3`；比较浮点数通常用容差而不是直接 `==`。

---

## 4. 两道题放在一起：数据流图

```text
                    Day 03：机器人关节误差计算

题 1：按值传递
main.current = 0.8
      │ 传入数值副本
      ▼
addOffsetByValue(angle=0.8, offset=0.5)
      │ angle 改为 1.3
      ▼
main.current 仍然是 0.8
      │
      ▼
computeJointError(1.2, 0.8) → return 0.4

题 2：引用传递
main.target ────── const & ────┐
main.current ───── const & ────┤ computeJointErrors
main.errors  ◀────       & ────┘
                              ↓
                      errors[i] = target[i]-current[i]
                      返回 true / false
```

**判断方法**：看函数签名，就能大致知道函数会不会复制大对象、会不会修改调用者的数据。

---

## 5. 面试官追问（含参考答案）

### Q1. `double x` 和 `double& x` 有什么区别？

**答**：`double x` 是按值传递，函数拿到一个副本，修改它不会改变调用者的变量；`double& x` 是引用，函数修改 `x` 会修改调用者的对象。

### Q2. `const double x` 和 `const double& x` 有什么区别？

**答**：前者仍然按值复制，只禁止函数内修改局部形参；后者通过只读引用访问原对象，不复制该对象。对于小标量通常按值更直观，对于较大的容器通常用 `const &`。

### Q3. 为什么函数输入向量常写 `const std::vector<double>&`？

**答**：避免按值传递时复制全部元素，同时在接口上表明函数不应通过这个形参修改输入数据。

### Q4. 为什么输出向量要写成 `std::vector<double>&`？

**答**：这样函数能直接写入调用者传入的容器；如果按值传递，函数内部修改的是副本。

### Q5. `const` 引用是否代表底层对象绝对不会改变？

**答**：不是。它仅禁止通过这个引用修改对象；若底层对象本身非 `const`，其他合法引用或代码仍可能修改它。多线程时还需要同步机制。

### Q6. `resize()` 和 `reserve()` 有什么区别？

**答**：`resize(n)` 改变向量的元素个数；`reserve(n)` 仅保证容量足够，不改变元素个数。`reserve` 后不能直接把尚不存在的元素当作 `v[i]` 使用。

### Q7. `bool` 返回值有什么用？

**答**：用于表达成功/失败等二元状态。今天维度不一致返回 `false`，调用者可据此停止使用无效结果。

### Q8. `std::size_t` 和 `int` 有什么区别？

**答**：`std::size_t` 是用于表示对象大小/容器下标的无符号整数类型，容器的 `.size()` 返回它；`int` 是有符号整数。直接混合比较可能产生编译警告或边界问题。

### Q9. 本题时间复杂度是多少？

**答**：单关节误差 $O(1)$；N 关节误差 $O(N)$。如果保存 N 个结果，需要 $O(N)$ 存储。

### Q10. 为什么机器人实时控制代码要避免每次 `resize`？

**答**：当 `resize` 导致重新分配时，执行时间不确定，会影响实时性。通常在初始化阶段预分配固定大小的缓冲区，并在控制循环内重复使用。

### Q11. 如果机器人关节角度从 $179^\circ$ 跳到 $-179^\circ$，直接相减有问题吗？

**答**：如果两角代表同一个周期角坐标，直接相减会得到接近 $358^\circ$ 的误差，但最短角度差只有约 $2^\circ$；需要角度归一化或专门的角度差函数。本课先不实现。

### Q12. 在多关节 P 控制器中，怎么复用今天的误差向量？

**答**：先求 $\mathbf e=\mathbf q_{ref}-\mathbf q$，再逐关节计算 $\tau_i=K_{p,i}e_i$，最后执行限幅；也可以写成矩阵形式 $\boldsymbol\tau=K_p\mathbf e$，其中 $K_p$ 为对角增益矩阵时对应独立关节 P 控制。

---

## 6. 课后练习：先自己写，再看答案

### 练习 A（5 分钟）｜预测输出

```cpp
void update(double x) { x += 2.0; }
int main() {
    double a = 1.0;
    update(a);
    std::cout << a << '\n';
}
```

问：输出多少？为什么？

**参考答案**：输出 `1`。`x` 是副本，函数内部 `x` 变成 3，但 `a` 仍为 1。

### 练习 B（10 分钟）｜改成引用版本

把练习 A 的函数改成能够修改 `a` 的版本。

**参考答案**：

```cpp
void update(double& x) { x += 2.0; }
```

此时 `a` 会变成 `3.0`。核心变化是增加 `&`。

### 练习 C（15 分钟）｜计算误差的平均绝对值

对题目 2 的输出 `errors`，实现：

$$
\mathrm{MAE}=\frac{1}{N}\sum_{i=1}^{N}|e_i|
$$

输入 `errors=[0.2,-0.1,-0.3]`，输出 $\mathrm{MAE}=0.2$。

**参考代码**（可以放在题 2 的 `main` 之前）：

```cpp
#include <cmath> // std::abs

double meanAbsoluteError(const std::vector<double>& errors) {
    if (errors.empty()) return 0.0; // 演示约定：空向量返回 0
    double sum = 0.0;
    for (double e : errors) {
        sum += std::abs(e);
    }
    return sum / static_cast<double>(errors.size());
}
```

- `std::abs` 求绝对值。
- `static_cast<double>(...)` 是显式类型转换，让除法的数值含义更清晰。
- 对空向量返回 0 是**本练习的约定**；严格数学上空集的平均值没有定义，工程中也可以选择报错。
- 时间复杂度 $O(N)$，额外空间 $O(1)$。

### 练习 D（面试延伸，选做）｜输出参数还是返回向量？

试着把题 2 改成直接 `return std::vector<double>`，比较两种接口的可读性。现代 C++ 中返回局部向量通常可以利用返回值优化或移动语义，并不必然发生昂贵复制；今天只要求知道两种接口都可以设计，后续再深入性能问题。

---

## 7. 复盘与自测清单

完成后检查自己是否能不看答案解释以下内容：

- [ ] 我能说出 `double x` 为什么不会修改调用者的变量。
- [ ] 我能解释 `const double x` 与 `const double& x` 的不同。
- [ ] 我能说出为什么大向量常用 `const std::vector<double>&` 输入。
- [ ] 我能用 `std::vector<double>&` 将结果写回调用者。
- [ ] 我能解释 `resize` 与 `reserve` 的区别。
- [ ] 我能独立写出 $e=q_{ref}-q$ 和 $\mathbf e=\mathbf q_{ref}-\mathbf q$。
- [ ] 我能在终端用 `g++ -std=c++17` 编译两份代码。
- [ ] 我能处理维度不一致、输入失败等边界情况。
- [ ] 我能说明时间复杂度 $O(1)$ 与 $O(N)$。

**建议学习方法**：先遮住代码，手写两个函数签名；再独立写函数体；最后自己构造 3 组测试。代码写不出来时先看“函数签名”和“数学公式”，不要立刻复制全部答案。

---

## 8. 文件目录与运行命令

```text
Robot_CPP_Day03/
├── Day03_机器人C++_按值传递_const与引用_完整精读.md
├── src/
│   ├── day03_q1_value_const.cpp
│   └── day03_q2_const_ref.cpp
├── cards/
│   ├── 01_封面.png
│   └── ...（共 8 张 1080×1440 图片）
├── Day03_8张卡片总览.jpg
└── tests/
    └── test_results.txt
```

从 `Robot_CPP_Day03/` 目录运行：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic src/day03_q1_value_const.cpp -o q1
g++ -std=c++17 -Wall -Wextra -Wpedantic src/day03_q2_const_ref.cpp -o q2
printf '1.2 0.8 0.5\n' | ./q1
printf '3\n1.2 0.0 -0.5\n1.0 0.1 -0.2\n' | ./q2
```

> **版权与使用**：本讲义与配套代码为教学示例，可用于个人学习和面试训练。实际机器人上运行前，需要根据设备单位、关节限位、实时性与安全约束重新设计和验证。
