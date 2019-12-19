//
// Created by lgj on 2019/11/29.
//

#ifndef DECISIONMAKER_CYCARRAY_H
#define DECISIONMAKER_CYCARRAY_H
#include <mutex>
#include <memory>
#include <vector>

template <typename T,
        template <typename ELEM,
        typename ALLOC = std::allocator<ELEM> >
        class CONT = std::vector>
class CycArray
{
public:
    CycArray();
    CycArray(unsigned short size);
    ~CycArray();

    void update(const T& item);
    bool try_pop_lastest(T&);
    std::shared_ptr<T> try_pop_lastest();
    std::size_t try_pop_last(unsigned short, std::vector<T> &);
    void clear();

    CycArray(CycArray const&) = delete;
    CycArray& operator=(CycArray const&) = delete;

    CycArray(CycArray &&) = delete;
    CycArray& operator=(CycArray &&) = delete;
private:
    mutable std::mutex mut;
    unsigned short TSize;//总长度
    unsigned short ASize;//有效数据长度
    int rear;//最新数据位置
    CONT<T> Elems;
};

template <typename T, template <typename,typename> class CONT>
CycArray<T,CONT>::CycArray() : TSize(3),ASize(0),rear(-1),Elems(TSize){

}

template <typename T, template <typename,typename> class CONT>
CycArray<T,CONT>::CycArray(unsigned short size) : TSize(size),ASize(0),rear(-1),Elems(TSize) {

}

template <typename T, template <typename,typename> class CONT>
CycArray<T,CONT>::~CycArray() {
}

template <typename T, template <typename,typename> class CONT>
void CycArray<T,CONT>::update(const T &item) {
    std::lock_guard<std::mutex> lk(mut);
    if (++rear >= TSize)                     //到数组结尾
        rear = 0;
    Elems[rear] = item;
    if (++ASize > TSize) ASize = TSize;
}

template <typename T, template <typename,typename> class CONT>
bool CycArray<T,CONT>::try_pop_lastest(T &value) {
    std::lock_guard<std::mutex> lk(mut);
    if (rear >= 0){
        value = Elems[rear];
        return true;
    } else{
        return false;
    }
}

template <typename T, template <typename,typename> class CONT>
std::shared_ptr<T> CycArray<T,CONT>::try_pop_lastest() {
    std::lock_guard<std::mutex> lk(mut);
    if (rear >= 0){
        auto res(std::make_shared<T>(Elems[rear]));
        return res;
    } else{
        return std::shared_ptr<T>();
    }
}//

template <typename T, template <typename,typename> class CONT>
std::size_t CycArray<T,CONT>::try_pop_last(unsigned short num, std::vector<T> &res) {
    std::lock_guard<std::mutex> lk(mut);
    if (0 == ASize || 0 == num)
    {
        return 0;
    }
    else
    {
        int j = 0;
        unsigned short len = std::min(ASize,num);
        for (int i = 0; i < len; ++i)
        {
            j = rear - i;
            if (j < 0)
                j += TSize;
            res.push_back(Elems[j]);
        }
        return len;
    }
}

template <typename T, template <typename,typename> class CONT>
void CycArray<T,CONT>::clear() {
    std::lock_guard<std::mutex> lk(mut);
    rear = -1;
    ASize = 0;
    for (int i = 0; i < TSize; i++)
    {
        Elems[i] = T();
    }
}

#endif //DECISIONMAKER_CYCARRAY_H
