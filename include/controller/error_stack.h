/**
 * @file error_stack.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_ERROR_STACK_H_
#define PICO_MOTORS_ERROR_STACK_H_

#include <numeric>

#define INT_SIZE 5

class ErrorStack
{
public:
    void add(int32_t value)
    {
        error.insert(error.begin(), value);

        if (error.size() > INT_SIZE)
            error.pop_back();
    }

    int32_t get()
    {
        return std::accumulate(error.begin(), error.end(), 0);
    }

    bool isFull()
    {
        return error.size() == INT_SIZE;
    }

    void clear()
    {
        error.clear();
    }

private:
    std::vector<int32_t> error;
};

#endif //PICO_MOTORS_ERROR_STACK_H_
