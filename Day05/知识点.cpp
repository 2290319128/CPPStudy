#include <vector>
#include <iostream>

// vector
std::vector<double> q = {0.1, 0.2, -0.3};
/*
std::：名称位于 C++ 标准库命名空间中。
vector：动态数组容器，长度可在运行时确定。
<double>：每个元素都是双精度浮点数。
{0.1, 0.2, -0.3}：用初始化列表直接填充三个元素。
q.size()：返回元素数量，这里是 3，返回类型为 std::size_t。
q[0]：第一个元素；q[2]：第三个元素。索引从 0 开始。
q.empty()：判断是否为空。
q.at(i)：带边界检查的访问，越界会抛出异常；q[i] 不做自动边界检查。
*/

// 循环
int main(){
    for (std::size_t i = 0; i < q.size(); ++i) {
        std::cout << q[i] << '\n'; // 需要索引时用普通 for
        //能拿到索引 i，适合需要索引的场景（如与另一个数组 r[i] 联动、
        //打印序号、访问 q[i+1]、逆序 i = q.size(); i-- > 0 等）
    }

    for (double value : q) {
        std::cout << value << '\n'; // 只需要值时用范围 for
        // 遍历并复制每个标量，只读取，不修改数组
        // 每次迭代把当前元素拷贝到局部变量 value
        // 不能通过 value 修改容器（改的是副本）
    }

    for (double& value : q) {
        value += 0.1; // 引用到原元素，会修改 q
        // 遍历并引用元素，输入直接写入原数组
        // value 是当前元素的引用，修改 value 就是修改 q 中的原元素。
        // 可以原地修改容器元素
    }

    for (const double& value : q) {
        std::cout << value << '\n'; // 只读引用，不修改原元素
        // value 是当前元素的只读引用（不拷贝）
    }

    return 0;
}

/*

*/