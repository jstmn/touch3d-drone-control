/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name: 

  DynamicsMath.h

Description:

  Math classes and routines.

*******************************************************************************/

#ifndef dynamicsMath_H_
#define dynamicsMath_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <vector>
#include <assert.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

/* vector of doubles - arbitrary length
   simple wrapper on top of std::vector that adds
   basic math operations. */
class nvectord : public std::vector<double>
{
public:

    nvectord()
    {
    }

    nvectord(const int size)
        : std::vector<double>(size)
    {
    }

    nvectord& operator*=(double d)
    {
        for (iterator i = begin(); i != end(); ++i)
        {
            (*i) *= d;
        }
        return *this;
    }
};

inline nvectord operator*(const nvectord& v1, const nvectord& v2)
{
    const int size = v1.size();
    assert(size == v2.size());
    nvectord result(size);
    for (int i = 0; i < size; i++)
    {
        result[i] = v1[i] * v2[i];
    }
    return result;
}

inline nvectord operator+(const nvectord& v1, const nvectord& v2)
{
    const int size = v1.size();
    assert(size == v2.size());
    nvectord result(size);
    for (int i = 0; i < size; i++)
    {
        result[i] = v1[i] + v2[i];
    }
    return result;
}

inline nvectord operator-(const nvectord& v1, const nvectord& v2)
{
    const int size = v1.size();
    assert(size == v2.size());
    nvectord result(size);
    for (int i = 0; i < size; i++)
    {
        result[i] = v1[i] - v2[i];
    }
    return result;
}

inline nvectord operator/(const nvectord& v1, const nvectord& v2)
{
    const int size = v1.size();
    assert(size == v2.size());
    nvectord result(size);
    for (int i = 0; i < size; i++)
    {
        result[i] = v1[i] / v2[i];
    }
    return result;
}

inline nvectord operator*(const nvectord& v1, double d)
{
    const int size = v1.size();
    nvectord result(size);
    for (int i = 0; i < size; i++)
    {
        result[i] = v1[i] * d;
    }
    return result;
}

inline nvectord operator*(double d, const nvectord& v1)
{
    return operator*(v1, d);
}

inline nvectord operator/(const nvectord& v1, double d)
{
    const int size = v1.size();
    nvectord result(size);
    for (int i = 0; i < size; i++)
    {
        result[i] = v1[i] / d;
    }
    return result;
}

#endif // dynamicMath_H_

/******************************************************************************/
