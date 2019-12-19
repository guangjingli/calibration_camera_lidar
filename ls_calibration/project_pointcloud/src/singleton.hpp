#ifndef SINGLETON_HPP
#define SINGLETON_HPP

#include <utility>


class Singleton
{
public:
    template<typename T, typename... Args>
    static T& getInstance(Args ... args)
    {
        static T value(std::forward<Args>(args)...);
        return value;
    }

    Singleton(Singleton const&) = delete;
    Singleton& operator=(Singleton const&) = delete;

    Singleton(Singleton &&) = delete;
    Singleton& operator=(Singleton &&) = delete;
private:
    Singleton();
    ~Singleton();
};

#endif