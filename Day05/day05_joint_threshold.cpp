/*
题目2：手写多关节误差阈值检查

给定n个关节的目标角度、当前角度，以及统一允许的绝对误差阈值 ε(≥0)，
判断是否所有关节都满足误差不超过阈值，并输出最大绝对误差和对应的关节索引。

示例输入：
3
1 0 -1
0.5 0.25 -0.75
0.25

示例输出：
检查结果: FAIL
最大绝对误差(rad): 0.500000
最大误差关节索引: 0
*/

#include <iostream>
#include <vector>
#include <iomanip>

bool pass(const std::vector<double>& target, const std::vector<double>& now,
    const double& eta, double& error_max, int& idx){

    double error;
    bool isPass = true;

    if (target.size() != now.size()){
        std::cerr << "维度不一致" << "\n";
        return 1;
    }
    
    for (std::size_t i = 0; i < target.size(); ++i){
        error = std::abs(target[i] - now[i]);

        if (error > eta){
            //return false; //不能这么写，这样当False直接跳出了，后面都执行不了
            isPass = false;
        }

        if (i == 0 || error > error_max){
            error_max = error;
            idx = i;
        }
    }
    
    return isPass;
}

int main(){
    int n, idx;
    double eta, error_max;

    std::cin >> n;

    std::vector<double> angle_target(n), angle_now(n);

    for (double& angle : angle_target){
        std::cin >> angle;
    }

    //std::cout << "angle_target:" << angle_target[0] << angle_target[1] << angle_target[2] << "\n";

    for (double& angle : angle_now){
        std::cin >> angle;
    }

    std::cin >> eta;

    bool isSatisfied = pass(angle_target, angle_now, eta, error_max, idx);

    // for (std::size_t i=0; i<n; ++i){
    //     double error = std::abs(angle_target[i] - angle_now[i]);
        
    //     if (error > eta){
    //         isSatisfied = false;
    //     }

    //     if (i == 0 || error > error_max){
    //         error_max = error;
    //         idx = i;
    //     }

    // }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "检查结果:" << (isSatisfied ? "PASS" : "FAIL") << "\n";
    std::cout << "最大绝对误差(rad):" << error_max << "\n";
    std::cout << "最大误差关节索引:" << idx << "\n";  


    return 0;
}