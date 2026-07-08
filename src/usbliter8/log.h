#pragma once

#define DEBUG_ENABLED   (0)

#include <stdio.h>

// #define INFO(fmt, ...) \
//     printf(fmt "\n", ##__VA_ARGS__)

#define INFO(fmt, ...) \
    (void)0

#if DEBUG_ENABLED

#define DEBUG(fmt, ...) \
    printf("%s(): " fmt "\n", __func__, ##__VA_ARGS__)

#else

#define DEBUG(fmt, ...) \
    (void)0

#endif
