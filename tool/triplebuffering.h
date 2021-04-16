#ifndef TRIPLEBUFFERING_H
#define TRIPLEBUFFERING_H
#include <functional>

template<typename T>
class TripleBuffering
{
public:
    TripleBuffering()
    {
        Front = new T;
        Middle = new T;
        Back = new T;
    }
    ~TripleBuffering()
    {
        delete Front;
        delete Middle;
        delete Back;
    }

    T* read()
    {
        if(MiddleIsNew == true)
        {
            swap(Front,Middle);
            MiddleIsNew = false;
            return Front;
        }
        else
        {
            return Front;
        }
    }
    void writeOver(void)
    {
        swap(Back,Middle);
        MiddleIsNew = true;
    }
public:
    T* Back = nullptr;
private:
    T* Front = nullptr;
    T* Middle = nullptr;
    bool MiddleIsNew = false;
};

#endif // TRIPLEBUFFERING_H
