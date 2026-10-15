/*
题目2：多关节误差向量 + const引用与输出引用
给定机器人N个关节的目标角度和当前角度：
q_ref=[q_ref_1,...,q_ref_N]^T
q=[q_1,...q_N]
计算每个关节的误差e_i=q_ref_i-q_i

编写：
bool computeJointErrors(const std::vector<double>& target,
                        const std::vector<double>& current,
                        std::vector<double>& errors);
要求：
target、current 为只读输入，不复制整个向量；
errors 为输出引用，函数直接修改调用者的结果容器；
两个输入向量长度不相同时，返回 false；长度相同时计算并返回 true；
主程序从标准输入读取 n、n 个目标角、n 个当前角，输出误差。

输入示例：
3
1.2 0.0 -0.5
1.0 0.1 -0.2
输出示例：
关节误差(rad): 0.200 -0.100 -0.300
*/

#include <iostream>
#include <vector>
#include <iomanip>
#include "console_utf8.h"

bool conputeJointErrors(const std::vector<double>& target,
                        const std::vector<double>& current,
                        std::vector<double>& errors){

    //使用 const & 避免把输入的两份 vector 各复制一遍，减少不必要的时间和内存开销。
     if (target.size() != current.size()){
        return false;
     }

     errors.resize(target.size());
     //for (int i=0; i<target.size(); ++i){
     for (std::size_t i = 0; i < target.size(); ++i) {
    //std::size_t 是用于表示对象大小/容器下标的无符号整数类型，
    //容器的 .size() 返回它；int 是有符号整数。直接混合比较可能产生编译警告或边界问题。
        errors[i] = target[i] - current[i];
     }
     return true;
}

int main(){
    SetConsoleOutputCP(CP_UTF8);

    int n;
    std::cin >> n;

    std::vector<double> target(n), current(n), errors;

    for (int i = 0; i < n; ++i){
        if(!(std::cin >> target[i])){
            std::cerr << "第" << i+1 << "个目标角度格式错误\n";
            return 1;
        }
    }

    for (int i = 0; i < n; ++i){
        if(!(std::cin >> current[i])){
            std::cerr << "第" << i+1 << "个当前角度格式错误\n";
            return 1;
        }
    }
 
    if(!conputeJointErrors(target, current, errors)){  //注意逻辑
        std::cerr << "目标角度和当前角度向量长度不一致\n";
        return 1;
    }

    std::cout << std::fixed << std::setprecision(3);

    std::cout << "关节误差(rad):";
    for (double e : errors){
        //逐个取出 errors 的元素并复制给局部变量 e，仅用于打印。
        //如果想原地修改元素，才需要 for (double& e : errors)。
        std::cout << " " << e;
    }
    std::cout << "\n";

    return 0;
}