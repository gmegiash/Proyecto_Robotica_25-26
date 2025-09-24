#include <chrono>
#include <iostream>
#include <vector>
#include <functional>
#include <map>
#include <execution>
#include <algorithm>
#include <tuple>

template<typename T>
concept Callable = std::invocable<T>;

template<typename T>
auto timer(T f)
{
    const auto start = std::chrono::high_resolution_clock::now();
    std::invoke(f);
    const auto end = std::chrono::high_resolution_clock::now();

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    return ms;
}

template<typename... Args>
int print(Args... args)
{
    return sizeof...(args);

}

// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.
int main()
{
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    std::vector<int> v(1000000000);
    for (auto &i: v)
    {
        i = rand();
    }
    auto a = timer([&v]()
    {
        std::sort(std::execution::par, v.begin(), v.end(), [](auto a, auto b)
        {
            return a < b;
        });
    });
    std::cout << a << std::endl;


    std::map<int, int> m;
    for (int i=0; i<10000000; i++)
    {
        m[i] = i;
    }

    auto b = timer([&m]()
    {
       m.find(rand() % 10000000);
    });
    std::cout << b << std::endl;

    std::cout << print(1, "hello", 3.14) << std::endl; // Outputs: 1 hello 3.14
    return 0;
    // TIP See CLion help at <a href="https://www.jetbrains.com/help/clion/">jetbrains.com/help/clion/</a>. Also, you can try interactive lessons for CLion by selecting 'Help | Learn IDE Features' from the main menu.
}