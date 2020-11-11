
// #define LIBCOPP_MACRO_ENABLE_STD_COROUTINE 1

#include <libcopp/future/std_coroutine_task.h>
#include "Types.hpp"

copp::future::task_t<int> test() {
    // ... any code
    co_return 321;
    co_return 123;
}
