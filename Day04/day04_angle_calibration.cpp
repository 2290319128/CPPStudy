/*
题目1：关节编码器零位校准
假设一个关节编码器读出原始角度 raw_degree（单位：度），标定得到零位偏移 zero_offset_degree（单位：度）。
机器人运动学/控制程序希望拿到以弧度为单位的关节角 angle_rad。

要求编写函数：
bool calibrateAngle(const double* raw_degree,
                    double zero_offset_degree,
                    double* angle_rad);

1. 原始角度使用 const double* 输入，不得修改原始读数。
2. 校准后的角度通过 double* 输出。
3. 遇到空指针、NaN、无穷大等非法数据返回 false，且不能改变调用者原有输出。
4. 允许正常的正角度和负角度；本题不执行角度周期归一化。

数学推导：
原始角度q_raw，零位偏移q_0,零位矫正为：q_corrected=q_raw-q_0
因为180度=\pi rad，因此：
q_rad=(q_raw-q_0)*pi/180

示例：
输入：90 10
输出：校准角度(rad): 1.396263

输入：-170 -10
输出：校准角度(rad): -2.792527
*/

#include <iostream>
#include <cmath>
#include <iomanip>
#include "console_utf8.h"

bool calibrateAngle(const double* raw_degree, //注意这里raw_degree是地址
                    double zero_offset_degree,
                    double* angle_rad){
//onst double* raw_degree：raw_degree 是指向输入角度的地址。
//*raw_degree 读取角度。const 表示不能通过这个指针写 *raw_degree = ...
    
    if (raw_degree == nullptr || angle_rad == nullptr){
        return false;
    }

    if (!std::isfinite(*raw_degree) || (!std::isfinite(zero_offset_degree))){
        return false;
    }

    //constexpr double pi：定义编译期常量。
    constexpr double pi = 3.14159265358979323846;

    const double corrected_degree = *raw_degree - zero_offset_degree;
    const double result = corrected_degree * pi / 180;

    if (!std::isfinite(result)){
        return false;
    }

    *angle_rad = result;

    return true;
}

int main(){
    SetConsoleOutputCP(CP_UTF8);

    double raw_degree, zero_offset, angle_rad;

    if (!(std::cin >> raw_degree >> zero_offset)){
        std::cerr << "输入错误\n";
        return 1;
    }

    if (!(calibrateAngle(&raw_degree, zero_offset, &angle_rad))){
        std::cerr << "校准失败\n";
        return 1;
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "校准角度(rad):" << angle_rad << "\n";

    return 0;
}

//double* angle_rad：传入调用者变量的地址。
//*angle_rad = result; 修改调用者的变量，因此 main() 中 angle_rad 能接收到结果。
//&raw_degree 与 *raw_degree 不是一回事：前者在 main 中取局部变量的地址；后者在函数内部解引用一个指针形参。