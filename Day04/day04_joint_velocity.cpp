/*
题目2：多关节角速度估计
控制器每隔Δt秒读取一次多个关节的位置。若没有直接使用关节速度传感器，
可用相邻两次关节位置做最简单的后向差分速度估计。

实现接口：
bool estimateJointVelocities(const double* q_now,
                             const double* q_prev,
                             std::size_t n,
                             double dt,
                             double* dq);

要求：q_now、q_prev 是长度至少为 n 的只读数组；
dq 是长度至少为 n 的输出数组；指针不可为空、n>0、dt>0 
且数据必须有限。任何检查失败时，dq 原有内容保持不变。

数学原理：
在离散的采样下，令t_k-t_k-1=Δt
v_k=[q_k-q_k-1]/Δt

输入：第一行 n dt；第二行 n 个上一时刻角度；第三行 n 个当前时刻角度。
3 0.1
0.1 -0.2 1.0
0.15 -0.22 0.98
输出：
关节角速度(rad/s): 0.500 -0.200 -0.200
*/

#include <iostream>
#include "console_utf8.h"
#include <cmath>
#include <vector>
#include <iomanip>    // std::fixed、std::setprecision

//std::size_t 是 C++ 标准库定义的一种无符号整数类型，通常用来表示
//对象大小、数组长度、下标、循环计数 
//特点是：无符号：不能表示负数，只能表示 0 和正整数。
//平台相关：在 32 位系统上通常是 32 位无符号整数，在 64 位系统上通常是 64 位无符号整数。

bool estimateJointVelocities(const double* q_now,
                            const double* q_prev,
                            std::size_t n,
                            double dt,
                            double* dq){
    //q_now 类型是 const double*：指向 const double 的指针,表示不能通过 q_now 修改它指向的数据,只读。
    //dq 类型是 double*：指向 double 的指针，可以通过它修改指向的数据。

    // 必须先检查空指针、数组长度和采样周期。
    if (q_now == nullptr || q_prev == nullptr || dq == nullptr || 
        n == 0 || !std::isfinite(dt) || dt <= 0){
        return false;
    }

    // 第一遍：验证所有输入及计算结果，避免中途失败导致部分写入。
    for (std::size_t i=0; i < n; ++i){
        if (!std::isfinite(q_now[i]) || !std::isfinite(q_prev[i])){
            return false;
        }
        const double candidate = (q_now[i] - q_prev[i]) / dt;
        //q_now[i]等价于*(q_now + i)
        //从 q_now 保存的地址开始，向后移动 i 个 double，再解引用，取出那个位置的 double
        /*
        例如：
        double a[3] = {1.0, 2.0, 3.0};
        double* p = a;      // p 指向 a[0]
        p[1] = 9.0;         // 等价于 *(p + 1) = 9.0，a[1] 变成 9.0

        对于只读指针：
        const double* q = a;
        double x = q[0];    // 可以读
        // q[0] = 5.0;      // 错误，不能通过 const double* 修改数据
        */

        if (!std::isfinite(candidate)){
            return false;
        }
    }


    // 第二遍：确认全部合法后，再写入输出数组。
    for (std::size_t i = 0; i < n; ++i){
        dq[i] = (q_now[i] - q_prev[i]) / dt;
    }

    return true;
    
}

int main(){
    SetConsoleOutputCP(CP_UTF8);

    int count = 0;
    double dt;

    if (!(std::cin >> count >> dt) || count <= 0 || count > 1000){
        std::cerr << "输入错误\n";
        return 1;
    }

    const std::size_t n = static_cast<std::size_t>(count);
    //把已经检查为正数的 int count 显式转换成 std::size_t 类型，保存到一个只读变量 n 中，作为关节数量。
    //std::size_t 是 C++ 标准库定义的一种无符号整数类型，通常用来表示
    //对象大小、数组长度、下标、循环计数 
    //static_cast<std::size_t>(count) 是显式类型转换，把 int 类型的 count 转成 std::size_t 类型
    
    std::vector<double> q_prev(n), q_now(n), dq(n, 0.0);
    //定义一个名为 dq 的 std::vector<double> 对象，里面包含 n 个 double 元素，并且每个元素都被初始化为 0.0
    /*
    std::vector<double> a;        // 空的 vector，里面没有元素
    std::vector<double> b(5);     // 5 个元素，默认值初始化（double 为 0.0）
    std::vector<double> c(5, 2.5); // 5 个元素，每个都是 2.5
    */

    for (std::size_t i = 0; i < n; ++i){
        if (!(std::cin >> q_prev[i])){
            std::cerr << "上一时刻输入错误\n";
            return 1;
        }
    }

    for (std::size_t i = 0; i < n; ++i){
        if (!(std::cin >> q_now[i])){
            std::cerr << "当前时刻输入错误\n";
            return 1;
        }
    }

    //q_now 是一个 std::vector<double> 对象。vector 内部管理一段连续内存，用来存储元素。
    //.data()：例如 q_now.data() 返回连续存储区的首地址，类型是 double*；可以隐式转换为 const double* 
    //传给只读形参。若随后让 vector 扩容，之前保存的指针可能失效，因此本题在调用期间不改变数组容量。
    //q_now.data()等价于&q_now[0]
    //estimateJointVelocities(&q_now[0], &q_prev[0], n, dt, &dq[0]);
    if (!estimateJointVelocities(q_now.data(), q_prev.data(), n, dt, dq.data())){
        std::cerr << "速度估计失败\n";
        return 1;
    }

    std::cout << std::fixed << std::setprecision(3);

    std::cout << "关节角速度(rad/s):";

    for (std::size_t i = 0; i < n; ++i) {
        std::cout << ' ' << dq[i];
    }
    std::cout << '\n';

    return 0;
}