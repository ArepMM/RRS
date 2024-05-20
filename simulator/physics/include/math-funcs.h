#ifndef     MATH_FUNCS_H
#define     MATH_FUNCS_H

#include    <cmath>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
template <typename T>
T hs_p(T x)
{
    if ( x > static_cast<T>(0) )
        return static_cast<T>(1);
    else
        return static_cast<T>(0);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
template <typename T>
T hs_n(T x)
{
    if ( x < static_cast<T>(0) )
        return static_cast<T>(1);
    else
        return static_cast<T>(0);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
template <typename T>
T sign(T x)
{
    if (x == static_cast<T>(0))
        return static_cast<T>(0);
    else
    {
        if (x > static_cast<T>(0))
            return static_cast<T>(1);
        else
            return static_cast<T>(-1);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
template <typename T>
T cut(T x, T min, T max)
{
    if (x > max)
        return max;

    if (x < min)
        return min;

    return x;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
template <typename T>
T pf(T x)
{
    return hs_p(x) * x;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
template <typename T>
T nf(T x)
{
    return hs_n(x) * (-x);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
template <typename T>
T dead_zone(T x, T min, T max)
{
    if (x > max)
        return x - max;

    if (x < min)
        return x - min;

    return static_cast<T>(0);
}

#endif // MATH_FUNCS_H
