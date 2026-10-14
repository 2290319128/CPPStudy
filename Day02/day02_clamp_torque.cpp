/*
题目1：将力矩限幅写成一个函数
输入未限幅力矩 `value` 和非负限幅阈值 `limit`，
返回位于 [-limit, limit] 内的力矩。要求自己写 `if / else if`，暂时不使用 `std::clamp`。

输入：
一行两个浮点数 value limit，单位均为 N·m。 

输出：
`限幅后力矩: 数值`
*/


#include <iostream>   // 引入标准输入输出工具 std::cin 和 std::cout
#include "console_utf8.h" // 引入控制台 UTF-8 设置函数

//================第一步：定义函数====================
// 函数功能：把力矩限制在 [-limit, limit] 范围内。
// value：未限幅的力矩；limit：非负的最大允许力矩。
// 返回值：经过限幅后的力矩。
double clamp_torque(double value, double limit) {
    if (value > limit) {
        return limit;
    } else if (value < -limit) {
        return -limit;
    } else {
        return value;
    }
}

//================第二步：编写主函数====================
int main() {
    // 设置控制台输出编码为UTF-8
    SetConsoleOutputCP(CP_UTF8);
    double value, limit;

    if(!(std::cin >> value >> limit)) {
        std::cerr << "输入错误，请输入两个浮点数。" << std::endl;
        return 1; // 返回非零值表示错误
    }
    if (limit < 0) {
        std::cerr << "限幅阈值必须是非负数。" << std::endl;
        return 1; // 返回非零值表示错误
    }

    double clamped = clamp_torque(value, limit);
    std::cout << "限幅后力矩: " << clamped << std::endl;
    return 0;
}

//编译命令：g++ -std=c++14 -o clamp_torque day02_clamp_torque.cpp