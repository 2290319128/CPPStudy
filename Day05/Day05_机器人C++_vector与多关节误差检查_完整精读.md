# 机器人运动控制 C++17 面试手撕训练 · Day 05

> **日期**：2026-10-17　｜　**阶段**：C++ 零基础第 5 课　｜　**预计学习**：60–90 分钟  
> **本日主题**：`std::vector`、范围 `for`、关节误差 L2 范数、逐关节误差阈值检查  
> **承接知识**：Day 01 变量/循环；Day 02 函数；Day 03 `const` 引用；Day 04 指针与关节速度。  
> **编译标准**：C++17；仅使用标准库，无需 ROS、Eigen 或其他第三方依赖。

---

## 一、学习目标与路线定位

今天的核心不是背诵 `vector` 的语法，而是把它用在机器人控制中真实会出现的“多关节状态检查”上。完成后应该能：

1. 说清 `std::vector<double>`、`size()`、`[]`、`empty()` 的含义。
2. 分清 `for (double e : errors)` 和 `for (double& e : errors)`，解释按值与引用。
3. 使用 `const std::vector<double>&` 定义不会修改输入数据的算法函数。
4. 手写 L2 范数，解释为什么要平方、求和、开方。
5. 用索引循环比较目标角度与当前角度，找出最大误差关节，并执行阈值判断。
6. 理解“整体误差范数”与“单关节阈值”是不同的评价指标。

**本日两道题：**

| 题号 | 面试题 | 新知识 | 控制背景 | 难度 |
|---|---|---|---|---|
| 1 | 多关节误差向量的 L2 范数 | `vector`、范围 `for`、`sqrt` | 关节轨迹跟踪误差 | ★☆☆☆☆ |
| 2 | 多关节位置误差阈值检查 | 索引循环、`size_t`、`abs`、布尔判断 | 跟踪质量监测 | ★★☆☆☆ |

> **注意：** 本课的“阈值检测”是教学用的跟踪误差质量指标，不是可直接部署的工业机器人安全功能。实际安全系统需要硬件限位、冗余监测和安全认证等。

---

## 二、预备知识：为什么机器人算法大量使用 vector？

一个关节角度可以用 `double q` 表示；多个关节角度自然组成向量：

\[
\mathbf q=[q_0,q_1,\ldots,q_{n-1}]^{\mathsf T}.
\]

在 C++ 中，最直观的初学者写法是：

```cpp
std::vector<double> q = {0.1, 0.2, -0.3};
```

- `std::`：名称位于 C++ 标准库命名空间中。
- `vector`：动态数组容器，长度可在运行时确定。
- `<double>`：每个元素都是双精度浮点数。
- `{0.1, 0.2, -0.3}`：用初始化列表直接填充三个元素。
- `q.size()`：返回元素数量，这里是 `3`，返回类型为 `std::size_t`。
- `q[0]`：第一个元素；`q[2]`：第三个元素。索引从 **0** 开始。
- `q.empty()`：判断是否为空。
- `q.at(i)`：带边界检查的访问，越界会抛出异常；`q[i]` 不做自动边界检查。

**为什么不用裸指针？** 现阶段 `vector` 更方便管理动态长度和存储空间，也能减少手工分配内存的错误。`vector` 元素连续存储，未来与 Eigen、底层接口的互操作也有价值。

### 2.1 两种常用循环

```cpp
for (std::size_t i = 0; i < q.size(); ++i) {
    std::cout << q[i] << '\n'; // 需要索引时用普通 for
}

for (double value : q) {
    std::cout << value << '\n'; // 只需要值时用范围 for
}

for (double& value : q) {
    value += 0.1; // 引用到原元素，会修改 q
}

for (const double& value : q) {
    std::cout << value << '\n'; // 只读引用，不修改原元素
}
```

对于 `double` 这种很小的标量，按值读取 `for (double value : q)` 通常已经很合适；对于较大的对象，`const auto&` 常可避免复制。**不能机械地认为所有引用都比值传递更快。**

---

## 三、题目 1：手写多关节误差向量 L2 范数

### 3.1 面试题描述

机器人有 \(n\) 个关节，给定关节位置误差 \(e_i\)（单位 rad），计算整体误差向量的 L2 范数。输入第一行为关节数量 \(n\)，第二行为 \(n\) 个误差值。

**示例输入：**

```text
4
0.1 -0.2 0.2 0
```

**示例输出：**

```text
误差L2范数(rad): 0.300000
```

### 3.2 数学原理与逐步推导

目标角度与当前角度分别为：

\[
\mathbf q_{\rm ref}=[q_{\rm ref,0},\dots,q_{\rm ref,n-1}]^{\mathsf T},
\quad \mathbf q=[q_0,\dots,q_{n-1}]^{\mathsf T}.
\]

位置误差向量为：

\[
\mathbf e=\mathbf q_{\rm ref}-\mathbf q,
\qquad e_i=q_{\rm ref,i}-q_i.
\]

L2 范数的定义：

\[
\boxed{\|\mathbf e\|_2=\sqrt{\mathbf e^{\mathsf T}\mathbf e}
=\sqrt{\sum_{i=0}^{n-1}e_i^2}.}
\]

**为什么这样计算？** \(\mathbf e^{\mathsf T}\mathbf e\) 是每个误差分量的平方和，非负；再开平方使结果恢复到与误差相同的量纲（rad）。它可以描述整体跟踪误差大小，但不会告诉我们究竟哪个关节误差最大。

对示例：

\[
\|\mathbf e\|_2
=\sqrt{0.1^2+(-0.2)^2+0.2^2+0^2}
=\sqrt{0.01+0.04+0.04}
=\sqrt{0.09}=0.3\;\mathrm{rad}.
\]

### 3.3 算法思路

1. 创建长度为 \(n\) 的 `vector<double>`，读取所有误差。
2. 定义平方和 `sum_of_squares = 0.0`。
3. 使用范围 `for` 遍历每个误差，执行 `sum += e * e`。
4. 使用 `std::sqrt` 求平方根，返回结果。
5. 输出保留 6 位小数的范数。

### 3.4 完整可独立编译的 C++17 源代码（中文逐行注释）

> 文件名：`source/day05_q1_error_norm.cpp`。下面是完整源文件，不是省略版片段。

```cpp
#include <cmath>       // std::sqrt、std::isfinite：平方根与有限值检查
#include <iomanip>     // std::fixed、std::setprecision：控制输出格式
#include <iostream>    // std::cin、std::cout、std::cerr：输入输出
#include <vector>      // std::vector：动态数组

// 计算多关节位置误差向量的欧氏范数（L2 范数）。
// const std::vector<double>& 表示只读引用，避免复制整个数组。
double computeL2Norm(const std::vector<double>& errors) {
    double sum_of_squares = 0.0;  // 初始化平方和
    for (double error : errors) { // 范围 for：依次读取每个误差值
        sum_of_squares += error * error; // 累加每个分量的平方
    }
    return std::sqrt(sum_of_squares);    // 返回平方和的平方根
}

int main() {
    int n = 0;                        // 关节数量
    if (!(std::cin >> n) || n <= 0 || n > 100000) { // 检查输入和规模
        std::cerr << "错误：关节数量必须在 1~100000 之间。\n";
        return 1;                     // 非 0 表示运行失败
    }

    std::vector<double> errors(n);    // 创建 n 个误差元素
    for (double& error : errors) {    // 范围 for + 引用：修改数组元素
        if (!(std::cin >> error) || !std::isfinite(error)) { // 拒绝非法值
            std::cerr << "错误：关节误差必须是有限数字。\n";
            return 1;
        }
    }

    const double norm = computeL2Norm(errors); // 调用函数求范数
    std::cout << std::fixed << std::setprecision(6); // 保留 6 位小数
    std::cout << "误差L2范数(rad): " << norm << '\n';
    return 0;                         // 正常结束
}
```

### 3.5 逐段语法精读

| 语法 | 作用 | 为什么这么写 |
|---|---|---|
| `#include <vector>` | 引入动态数组 | 保存多关节误差 |
| `#include <cmath>` | 引入数学函数 | `std::sqrt`、`std::isfinite` |
| `std::vector<double> errors(n)` | 创建 n 个元素 | n 运行时由用户输入 |
| `for (double& error : errors)` | 遍历并引用元素 | 输入直接写入原数组 |
| `for (double error : errors)` | 遍历并复制每个标量 | 只读取，不修改数组 |
| `const std::vector<double>&` | 常量引用参数 | 减少数组复制，保证函数不通过该引用修改输入 |
| `sum_of_squares += error * error` | 平方累加 | 与数学定义逐项对应 |
| `std::sqrt(sum_of_squares)` | 求平方根 | 实现 L2 范数 |
| `std::fixed << std::setprecision(6)` | 固定小数格式 | 便于比较测试输出 |
| `!(std::cin >> n)` | 检查读取是否失败 | 避免使用无效输入 |
| `return 1` | 非零退出码 | 告知外部运行失败 |

**易错点 A：** `std::vector<double> errors(n)` 与 `std::vector<double> errors{n}` 不同。前者创建 **n 个元素**，后者创建 **一个元素，其值为 n**。

**易错点 B：** `for (double error : errors)` 中修改 `error` 不会修改数组，因为这里是复制；`for (double& error : errors)` 才能写回。

**易错点 C：** 不能把 `sum_of_squares` 声明为 `int`，否则小数部分会丢失。

### 3.6 编译与运行

在文件所在目录执行：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -O2 \
  day05_q1_error_norm.cpp -o day05_q1_error_norm
./day05_q1_error_norm
```

然后输入上面的示例数据。`-std=c++17` 选择语言标准，`-Wall -Wextra -Wpedantic` 打开常用警告，`-O2` 开启常规优化。

### 3.7 边界情况与复杂度

| 场景 | 输入示例 | 预期行为 |
|---|---|---|
| 全部为零 | `3 / 0 0 0` | 返回 0 |
| 只有一个负数 | `1 / -3` | 返回 3（范数非负） |
| 关节数量为 0 | `0` | 报错并退出 |
| 输入数量不足 | `2 / 1` | 报错并退出 |
| 数据为非法值 | 非数字/非有限数 | 报错并退出 |

遍历一次求平方和，**时间复杂度** \(O(n)\)；函数 `computeL2Norm` 仅使用一个累加变量，**额外空间复杂度** \(O(1)\)；主程序为了存储输入向量使用 \(O(n)\) 空间。

**数值提醒：** 此处为入门版。对于极大数值，`error * error` 可能溢出；严肃数值库可以采用缩放累加或稳定的 `hypot` 思路。这里先掌握基本算法。

---

## 四、题目 2：手写多关节误差阈值检查

### 4.1 面试题描述

给定 \(n\) 个关节的目标角度、当前角度，以及统一允许的绝对误差阈值 \(\varepsilon\ge 0\)，判断**是否所有关节**都满足误差不超过阈值，并输出最大绝对误差和对应的关节索引。

**示例输入：**

```text
3
1 0 -1
0.5 0.25 -0.75
0.25
```

**示例输出：**

```text
检查结果: FAIL
最大绝对误差(rad): 0.500000
最大误差关节索引: 0
```

说明：目标角度 `[1, 0, -1]`，当前角度 `[0.5, 0.25, -0.75]`，绝对误差 `[0.5, 0.25, 0.25]`，第 0 个关节超过 `0.25`，因此 `FAIL`。如果某个关节的误差**恰好等于**阈值，本课视为合格。

### 4.2 数学原理与逐步推导

每个关节的位置误差为：

\[
e_i=q_{\mathrm{ref},i}-q_i.
\]

逐关节的绝对误差为：

\[
a_i=|e_i|=|q_{\mathrm{ref},i}-q_i|.
\]

判断所有关节是否满足要求：

\[
\boxed{\mathrm{PASS}\iff \forall i,\;a_i\le\varepsilon.}
\]

最大绝对误差及其索引：

\[
a_{\max}=\max_{0\le i<n}a_i,
\qquad i^*=\underset{0\le i<n}{\operatorname{argmax}}\,a_i.
\]

若多个关节并列最大，本实现选择**最早出现的索引**，因为只有严格大于 `>` 才更新最大值。

### 4.3 为什么不能只看 L2 范数？

L2 范数是整体误差指标；逐关节阈值是分量约束。举例：

- 误差 `[0.2, 0.2, 0.2]`，L2 范数约为 `0.346`，但每个关节都满足 `|e_i| <= 0.25`。
- 误差 `[0.3, 0, 0]`，L2 范数是 `0.3`，却有一个关节超过 `0.25`。

因此，不能仅用 `norm <= 某个统一阈值` 代替逐关节检查（除非你明确设计了数学上合适的充分条件）。

### 4.4 算法思路

1. 读取两个长度相同的向量：目标角度 `target` 与当前角度 `current`。
2. 检查输入长度、数据是否有限以及阈值是否非负。
3. 使用**索引 `for`** 遍历：因为要同时访问 `target[i]` 和 `current[i]`。
4. 计算 `std::abs(target[i] - current[i])`。
5. 更新最大误差与对应索引，并检查是否大于阈值。
6. 返回 `bool` 表示是否全部通过，并通过引用参数返回最大误差和索引。

### 4.5 完整可独立编译的 C++17 源代码（中文逐行注释）

> 文件名：`source/day05_q2_joint_threshold.cpp`。此代码包含所有输入校验，直接可编译运行。

```cpp
#include <cmath>       // std::abs、std::isfinite
#include <iomanip>     // std::fixed、std::setprecision
#include <iostream>    // 输入输出
#include <vector>      // 动态数组

// 检查是否每个关节的位置误差都不超过同一个阈值。
// 只读引用避免复制输入向量；max_error 与 worst_index 为输出引用。
bool checkJointErrors(const std::vector<double>& target,
                      const std::vector<double>& current,
                      double threshold,
                      double& max_error,
                      std::size_t& worst_index) {
    if (target.empty() || target.size() != current.size() ||
        !std::isfinite(threshold) || threshold < 0.0) {
        return false; // 该示例的调用者在调用前验证这些前置条件
    }

    max_error = 0.0;                  // 最大绝对误差初值
    worst_index = 0;                 // 最大误差的关节编号（从 0 开始）
    bool all_within = true;          // 假设所有关节都满足阈值

    for (std::size_t i = 0; i < target.size(); ++i) { // 按索引配对两个数组
        const double error = std::abs(target[i] - current[i]); // 绝对误差
        if (error > max_error) {      // 更新最坏关节
            max_error = error;
            worst_index = i;
        }
        if (error > threshold) {      // 等于阈值视为合格
            all_within = false;
        }
    }
    return all_within;               // 返回整体检查结果
}

int main() {
    int n = 0;
    if (!(std::cin >> n) || n <= 0 || n > 100000) {
        std::cerr << "错误：关节数量必须在 1~100000 之间。\n";
        return 1;
    }

    std::vector<double> target(n);   // 目标角度数组
    std::vector<double> current(n);  // 当前角度数组
    for (double& q : target) {       // 读取目标角度
        if (!(std::cin >> q) || !std::isfinite(q)) {
            std::cerr << "错误：目标角度无效。\n";
            return 1;
        }
    }
    for (double& q : current) {      // 读取当前角度
        if (!(std::cin >> q) || !std::isfinite(q)) {
            std::cerr << "错误：当前角度无效。\n";
            return 1;
        }
    }

    double threshold = 0.0;          // 最大允许绝对误差
    if (!(std::cin >> threshold) || !std::isfinite(threshold) ||
        threshold < 0.0) {
        std::cerr << "错误：阈值必须为非负有限数字。\n";
        return 1;
    }

    double max_error = 0.0;          // 输出参数：最大误差
    std::size_t worst_index = 0;     // 输出参数：最坏关节编号
    const bool passed = checkJointErrors(target, current, threshold,
                                         max_error, worst_index);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "检查结果: " << (passed ? "PASS" : "FAIL") << '\n';
    std::cout << "最大绝对误差(rad): " << max_error << '\n';
    std::cout << "最大误差关节索引: " << worst_index << '\n';
    return 0;
}
```

### 4.6 逐段语法精读

| 语法 | 解释 | 面试时怎么说 |
|---|---|---|
| `std::size_t i` | 无符号的大小/索引类型 | 与 `vector.size()` 的返回类型匹配 |
| `target.size()` | 向量长度 | 用于边界检查与循环终止 |
| `target[i]` | 访问第 i 个目标角度 | 索引从 0 开始 |
| `std::abs(x)` | 绝对值 | 不区分正负误差的大小 |
| `bool all_within` | 布尔变量 | 保存是否全部通过 |
| `if (error > threshold)` | 严格超限 | 等于阈值时仍 PASS |
| `double& max_error` | 可写引用参数 | 让函数把结果写回调用者 |
| `std::size_t& worst_index` | 可写引用参数 | 返回最大误差的关节索引 |
| `passed ? "PASS" : "FAIL"` | 三元条件运算符 | 按布尔值选择输出文字 |
| `const std::vector<double>&` | 只读引用 | 避免复制输入向量 |

**函数契约说明：** `checkJointErrors()` 设计为“调用者已完成数据合法性验证”的内部计算函数；`main()` 在调用前已验证非空、同维度、有限值与非负阈值。函数内额外检查基本维度条件，但 `false` 同时可能代表“未通过阈值”或“输入不合法”，不能单凭返回值区分两种情况。真实项目应使用更清晰的状态类型或异常机制，这属于后续 `struct`/错误处理课程内容。

### 4.7 编译与运行

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -O2 \
  day05_q2_joint_threshold.cpp -o day05_q2_joint_threshold
./day05_q2_joint_threshold
```

### 4.8 边界测试与复杂度

| 场景 | 预期行为 |
|---|---|
| 所有关节误差为 0，阈值为 0 | PASS |
| 某关节误差恰好等于阈值 | PASS |
| 某关节误差超过阈值 | FAIL |
| 多个关节并列最大误差 | 返回第一个索引 |
| 阈值为负数 | 拒绝输入 |
| 目标/当前角度输入不足 | 拒绝输入 |
| 输入向量为空 | 拒绝输入 |

扫描 \(n\) 个关节一次：**时间复杂度** \(O(n)\)；`checkJointErrors` 函数额外空间 \(O(1)\)；主程序存储两个输入向量共 \(O(n)\) 空间。

**工程提醒：** 若角度是周期变量，例如连续旋转关节，直接相减可能产生跨 \(-\pi/\pi\) 的角度跳变；真实控制需根据关节定义处理角度差（如归一化到 \((-\pi,\pi]\)）。对于有机械限位的关节，通常使用其实际连续位置定义。本题暂不加入角度归一化，避免偏离入门重点。

---

## 五、把两道题联系起来：机器人控制中的误差监测

机器人控制循环中，通常先计算目标状态与测量状态的差：

\[
\mathbf e_k=\mathbf q_{\rm ref,k}-\mathbf q_k.
\]

接着可以做两件互补的事：

1. 计算 \(\|\mathbf e_k\|_2\)，作为日志、性能统计或控制代价中的一个量。
2. 检查每个 \(|e_{k,i}|\le\varepsilon_i\)，识别单关节跟踪偏差是否过大。

未来在 MPC 中，常见二次型跟踪代价为：

\[
\ell(\mathbf e)=\mathbf e^{\mathsf T}Q\mathbf e,
\]

当 \(Q=I\) 时，恰好是 \(\|\mathbf e\|_2^2\)，**注意不是 \(\|\mathbf e\|_2\) 本身**。以后学 Eigen、LQR、MPC 时会重新使用这个关系。

---

## 六、面试官追问（含参考答案）

### Q1. `std::vector` 和普通 C 数组有什么区别？

`vector` 自动管理动态容量，支持 `size()`、`push_back()` 等操作，通常更易用、更安全；普通数组长度通常固定且没有自带容量信息。`vector` 元素也是连续存储的。

### Q2. `vector.size()` 返回 `int` 吗？

不是，返回 `size_type`，通常是 `std::size_t`。因此索引循环可用 `std::size_t i`，避免有符号/无符号比较警告。

### Q3. `for (double e : errors)` 能修改数组元素吗？

不能。`e` 是每次复制出来的局部值；要修改原元素应写 `for (double& e : errors)`。

### Q4. `for (const double& e : errors)` 和按值读取有什么区别？

前者引用原元素且不可通过它修改；后者复制一个 `double`。对于小标量 `double`，按值读取通常简单高效；对于大型对象只读引用可能节省复制。

### Q5. `const std::vector<double>& errors` 为什么常见？

既避免复制整个向量，又防止函数通过这个引用修改调用者的数据，适合只读输入参数。

### Q6. L2 范数为什么要开平方？

平方和的量纲是 rad²，开平方后回到 rad。并且这符合欧氏距离的定义。很多优化问题直接使用平方范数，减少开平方操作并利于求导。

### Q7. L1、L2、L∞ 范数分别是什么？

\(\|e\|_1=\sum_i|e_i|\)；\(\|e\|_2=\sqrt{\sum_i e_i^2}\)；\(\|e\|_\infty=\max_i|e_i|\)。本日题 2 实际上计算了 L∞ 范数和其索引。

### Q8. 为什么第二题使用索引 `for`，而不是简单范围 `for`？

因为需要同时访问 `target[i]`、`current[i]`，还需要记录索引。C++17 标准范围 `for` 默认只遍历一个范围，不直接提供索引。

### Q9. 如果两个关节误差并列最大，返回哪个？

本实现使用 `>` 而非 `>=` 更新，因此保留第一次出现的最大值索引。

### Q10. `std::abs` 和 `std::sqrt` 在哪里定义？

在 `<cmath>` 中，调用时写 `std::abs`、`std::sqrt`。

### Q11. 这两道题的时间复杂度是多少？

都为 \(O(n)\)，因为各遍历 n 个元素一次；计算函数自身额外空间为 \(O(1)\)。输入向量存储占 \(O(n)\)。

### Q12. 能把第二题的 `return false` 当作“输入错误”吗？

不能。`false` 在本函数还代表“有一个或多个关节超出阈值”。工程上应把数据合法性和阈值判定分开，或者返回包含状态和统计量的结构体。

### Q13. 角度为什么推荐统一使用弧度？

三角函数、旋转矩阵、机器人动力学和大多数数学库都以弧度为默认单位；输入、输出和阈值必须使用一致单位。

### Q14. 是否可以把 L2 范数直接用作关节限位保护？

不可以。它是整体跟踪误差指标，不能直接替代关节位置、速度、力矩等硬约束或经认证的安全功能。

---

## 七、课后练习与参考解答

> 建议先自己写 10–15 分钟，再看参考答案。

### 练习 1（基础）：计算平均绝对误差 MAE

给定 \(n>0\) 个误差，计算 \(\mathrm{MAE}=\frac1n\sum_i|e_i|\)。

**参考答案：**

```cpp
#include <cmath>
#include <vector>
double meanAbsoluteError(const std::vector<double>& errors) {
    if (errors.empty()) return 0.0; // 本练习约定空向量返回 0
    double sum = 0.0;
    for (double e : errors) sum += std::abs(e);
    return sum / static_cast<double>(errors.size());
}
```

### 练习 2（基础）：找出第一个超限关节

返回第一个满足 \(|e_i|>\varepsilon\) 的索引；如果没有超限返回 `errors.size()`（哨兵值）。

**参考答案：**

```cpp
#include <cmath>
#include <vector>
std::size_t firstExceeded(const std::vector<double>& errors, double limit) {
    for (std::size_t i = 0; i < errors.size(); ++i) {
        if (std::abs(errors[i]) > limit) return i;
    }
    return errors.size();
}
```

### 练习 3（提高）：求 L∞ 范数

**参考答案：**

```cpp
#include <cmath>
#include <vector>
double infinityNorm(const std::vector<double>& errors) {
    double result = 0.0;
    for (double e : errors) {
        if (std::abs(e) > result) result = std::abs(e);
    }
    return result;
}
```

### 练习 4（思考）：为什么 MPC 代价常用平方误差？

**参考答案：** 对于 \(\ell=e^{\mathsf T}Qe\)，当 \(Q\) 对称时有 \(\nabla_e\ell=2Qe\)；二次型便于梯度、Hessian 和 QP/LQR 推导。它会对较大误差施加更强惩罚，但不能自动保证所有关节满足硬约束。

---

## 八、可复现测试记录

在 Linux `g++` 环境使用以下编译选项：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -O2 ...
```

测试脚本：`source/test_day05.py`；可执行：

```bash
python3 source/test_day05.py
```

已验证 **2 个程序编译通过、12 组测试通过**，覆盖正常输入、零误差、负误差、恰好等于阈值、非法 n、负阈值和输入不足等。

---

## 九、Day 05 学习检查卡

- [ ] 不看资料，能写出 `std::vector<double> errors(n)`。
- [ ] 能解释 `for (double e : errors)` 与 `for (double& e : errors)`。
- [ ] 能手写 `computeL2Norm()`，解释每一步数学意义。
- [ ] 能手写逐关节误差阈值检查，并输出最坏关节索引。
- [ ] 能解释 `size_t`、`abs`、`sqrt`、`const vector<double>&`。
- [ ] 能说出 L2 范数和逐关节误差阈值的差异。
- [ ] 能用 g++ 编译运行两道题。

**Day 06 预告**：`struct` 自定义数据结构与机器人 `JointState`（角度、速度、力矩），把分散的数组整理成更清晰的状态对象，为后续面向对象控制器做准备。
