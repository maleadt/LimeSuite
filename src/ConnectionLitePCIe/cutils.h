#include <inttypes.h>
#include <math.h>
#include <immintrin.h>

static inline int sub_mod_int(int a, int b, int m)
{
    a -= b;
    if (a < 0)
        a += m;
    return a;
}

static inline int add_mod_int(int a, int b, int m)
{
    a += b;
    if (a >= m)
        a -= m;
    return a;
}
