#include <chrono>
#include <iostream>
#include <vector>
#include <functional>
#include <map>

template<typename T>
concept Callable = std::invocable<T>;

template<typename T>
auto timer(T f)
{
    const auto start = std::chrono::high_resolution_clock::now();
    std::invoke(f);
    const auto end = std::chrono::high_resolution_clock::now();
    return end - start;
}



// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.
int main()
{
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    std::vector<int> v(100000000);
    for (auto &i: v)
    {
        i = rand();
    }
    auto a = timer([&v]()
    {
        std::sort(v.begin(), v.end(), [](auto a, auto b)
        {
            return a < b;
        });
    });
    std::cout << a << std::endl;

    std::map<int, int> m;
    return 0;
    // TIP See CLion help at <a href="https://www.jetbrains.com/help/clion/">jetbrains.com/help/clion/</a>. Also, you can try interactive lessons for CLion by selecting 'Help | Learn IDE Features' from the main menu.
}