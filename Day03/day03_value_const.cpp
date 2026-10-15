/*
题目1：单关节误差与按值传递
机器人某关节有目标角度 target、当前角度 current，另给一个演示用的偏移 offset：
1. 写 addOffsetByValue(double angle, double offset)函数，在该函数内部把 angle 加上 offset 并输出。
2. 回到 main 后输出原来的 current，观察它没有改变。
3. 写 computeJointError(const double target, const double current)函数，返回位置误差。
4. 输入三个数字，按示例输出。

输入：一行三个浮点数 target current offset，角度单位统一为 rad。
========示例==========
输入：
1.2 0.8 0.5
输出：(保留三位小数)
函数内的角度: 1.300
main中的角度: 0.800
关节位置误差: 0.400
*/

#include <iostream>   //使用 std::cin 输入、std::cout 输出、std::cerr 输出错误信息。
#include <iomanip>    // std::fixed 和 std::setprecision：控制输出格式
#include "console_utf8.h"

void addOffsetByValue(double angle,double offset){
    angle = angle + offset;
    std::cout << "函数内的角度:" << angle << "\n";
}

double computeJointError(const double target,const double current){
    return target - current;
}

int main(){

    SetConsoleOutputCP(CP_UTF8);

    double target, current, offset;
    if(!(std::cin >> target >> current >> offset)){
        std::cerr << "输入不规范\n";
        return 1;
    }

    std::cout << std::fixed << std::setprecision(3); // 保留 3 位小数

    addOffsetByValue(current,offset);
    std::cout << "main中的角度:" << current << "\n";
    //在函数内部是副本，除非使用引用或指针等方式，否则调用者的原值不变。

    double error = computeJointError(target,current);
    std::cout << "关节位置误差:" << error << "\n";

    return 0;
}


/*
double x	                    复制一个 double
const double x	                复制一个 double             明确函数内部不修改局部形参
std::vector<double> v	        复制整个向量元素
const std::vector<double>& v	不复制向量元素	
std::vector<double>& v	        不复制向量元素               能通过该形参修改调用者对象
*/


/*
void change(double x) {
    x = 10.0;
}

int main() {
    double a = 3.0;
    change(a);      // 调用时复制 a 的值给 x，函数内部复制了，但是外面是不变的
    // 这里 a 仍然是 3.0
}
*/


/*
double error(const double target, const double current) {
    // target = 0.0;  // 错误：const 形参不可在函数内重新赋值
    return target - current;
}
*/

/*
void setZero(double& x) { x = 0.0; }      // 修改调用者的原变量
void print(const double& x) {            // 只读引用，不允许通过 x 修改
    std::cout << x << '\n';
}
*/