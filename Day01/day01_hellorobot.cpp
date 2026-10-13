#include <iostream>

#include "console_utf8.h"

int main(){
    setup_utf8_console();
    std::cout << "Hello Robot!\n" << std::endl;
    return 0;
}

// 编译：g++ -std=c++17 -g day01_hellorobot.cpp -o hello
// 运行：.\hello
