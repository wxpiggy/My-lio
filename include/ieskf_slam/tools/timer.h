#include <chrono>
#include <iostream>
#include <functional>

template <typename Func>
void Timer(Func&& func, const std::string& task_name = "Task") {
    auto start = std::chrono::steady_clock::now();
    func();
    auto end = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << task_name << " 耗时: " << elapsed_ms << " 毫秒" << std::endl;
}