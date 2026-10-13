/*
题目2：手写一维机器人关节 P 控制器及力矩限幅

有一个机器人单关节位置控制任务，已知：

- 目标角度 $q_{ref}$，单位 rad；
- 当前角度 $q$，单位 rad；
- 比例增益 $K_p$，如果把输出定义为力矩，其单位是 $\mathrm{N\cdot m/rad}$；
- 最大允许输出力矩 $\tau_{\max}>0$，单位 $\mathrm{N\cdot m}$。

要求编写代码计算位置误差、未限幅力矩和限幅后的**最终输出力矩**。

输入：
1.2 0.8 10 3
依次对应 `target current kp max_torque`。

输出：
位置误差: 0.4
未限幅力矩: 4
最终输出力矩: 3
*/

#include <iostream> // 标准输入输出库：提供 std::cin、std::cout、std::cerr

int main(){
    double target = 0.0; // 目标角度
    double current = 0.0; // 当前角度
    double kp = 0.0; // 比例增益
    double max_torque = 0.0; // 最大允许输出力矩
    double error = 0.0; // 位置误差
    double torque_raw = 0.0; // 未限幅力矩
    double torque = 0.0; // 最终输出力矩
    //======== 第1步：读取输入 ========
    std::cin >> target >> current >> kp >> max_torque;

    //======== 第2步：计算位置误差 ========
    error = target - current;

    //======== 第3步：计算未限幅力矩 ========
    torque_raw = kp * error;
    torque = torque_raw;

    //======== 第4步：限幅 ========
    if (torque > max_torque) {
        torque = max_torque;
    } else if (torque < -max_torque) {
        torque = -max_torque;
    }

    //======== 第5步：输出结果 ========
    std::cout << "位置误差: " << error << "\n";
    std::cout << "未限幅力矩: " << torque_raw << "\n";
    std::cout << "最终控制力矩: " << torque << "\n";

    return 0;
}

// 编译：g++ -std=c++14 -g day01_p_controller.cpp -o p_controller
// 运行：.\p_controller