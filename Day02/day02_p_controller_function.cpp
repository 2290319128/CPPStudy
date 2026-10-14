/*
题目2：用函数封装一维 P 控制器

计算机器人关节位置误差、比例控制力矩，并**调用题目 1 的 `clampTorque()` 函数**对输出进行限幅。

**输入**：`target current kp max_torque`，目标和当前角度单位为 rad，
`kp` 单位为 N·m/rad，力矩单位为 N·m。为方便入门，本题假设 `kp >= 0`、`max_torque >= 0`。

输入：1.2 0.8 10 3
输出：控制力矩: 3
*/
#include <iostream>
#include "console_utf8.h" // 引入控制台 UTF-8 设置函数

//================第一步：引入写好的限幅函数====================
double clamp_torque(double value, double limit) {
    if (value > limit) {
        return limit;
    } else if (value < -limit) {
        return -limit;
    } else {
        return value;
    }
}

//================第二步：定义 P 控制器函数====================
// 函数功能：计算 P 控制器输出力矩，并调用 clamp_torque
double p_controller(double target,double current,double kp,double max_torque){
    // 计算位置误差
    double error = target - current;
    // 计算比例控制力矩
    double torque = kp*error;
    // 调用 clamp_torque 函数对输出进行限幅
    double clamped_torque = clamp_torque(torque, max_torque);

    return clamped_torque;
}

//=================第三步：编写主函数====================
int main() {
    SetConsoleOutputCP(CP_UTF8); // 设置控制台输出编码为UTF-8
    double target, current, kp, max_torque;
    if(!(std::cin >> target >> current >> kp >> max_torque)) {
        std::cerr << "输入错误，请输入四个浮点数。" << std::endl;
        return 1; // 返回非零值表示错误
    }

    double controller_torque = p_controller(target, current, kp, max_torque);
    std::cout << "控制力矩: " << controller_torque << std::endl;

    return 0;
}

//编译命令： g++ -std=c++14 -o p_controller day02_p_controller_function.cpp