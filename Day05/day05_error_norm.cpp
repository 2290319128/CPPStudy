/*
题目1：手写多关节误差向量 L2 范数
机器人有 n 个关节，给定关节位置误差e_i(单位 rad），计算整体误差向量的 L2 范数。
输入第一行为关节数量n，第二行为 n 个误差值。

示例输入：
4
0.1 -0.2 0.2 0
示例输出：
误差L2范数(rad): 0.300000
*/

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include "console_utf8.h"

// double computeNorm(std::vector<double> error, double norm_2){
//     double sum;
//     for (double value : error){
//         value = value * value;
//         sum += value;
//     }

//     norm_2 = std::sqrt(sum);
//     return norm_2;
// }

// const std::vector<double>& 表示只读引用，避免复制整个数组。
double computeNorm(const std::vector<double>& error){
    double sum;
    for (double value : error){
        value = value * value;
        sum += value;
    }

    return std::sqrt(sum);
}

int main(){
    SetConsoleOutputCP(CP_UTF8);

    int n;
    double sum, norm_2;
    std::cin >> n;

    std::vector<double> q(n);

    for (double& value : q){
        std::cin >> value;
    }

    // for (double value : q){
    //     value = value * value;
    //     sum += value;
    // }

    // norm_2 = std::sqrt(sum);

    norm_2 = computeNorm(q);


    std::cout << std::fixed << std::setprecision(6); // 保留 6 位小数
    std::cout << "误差L2范数(rad):" << norm_2 << "\n";

    return 0;
}