/**
 * @file error_stack.h
 * @author Noa Sendlhofer
 */

#ifndef PICO_MOTORS_ERROR_STACK_H_
#define PICO_MOTORS_ERROR_STACK_H_

#include <numeric>
#include "enum_definitions.h"

class ErrorStack
{
public:
    void add(int32_t value)
    {
        error.insert(error.begin(), value);

        if (error.size() > ERROR_STACK_INTEGRAL_SIZE)
            error.pop_back();
    }

    int32_t get()
    {
        return std::accumulate(error.begin(), error.end(), 0);
    }

    bool isFull()
    {
        return error.size() == ERROR_STACK_INTEGRAL_SIZE;
    }

    void clear()
    {
        error.clear();
    }

private:
    std::vector<int32_t> error;
};

#endif //PICO_MOTORS_ERROR_STACK_H_
