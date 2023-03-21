/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name:

  PointManipulationAfx.cpp

Description: 

  Compilation module for precompiled header

*******************************************************************************/

#ifndef PointManipulationAfx_H_
#define PointManipulationAfx_H_

#ifdef WIN32

// identifier was truncated to '255' characters in the debug information.

#pragma warning( disable: 4786 )

#endif // WIN32

#include <iostream>
#include <HD/hd.h>
#include <HDU/hduVector.h>

#if defined(WIN32) || defined(linux)
# include <GL/glut.h>
#elif defined(__APPLE__)
# include <GLUT/glut.h>
#endif

#endif // PointManipulationAfx_H_

/******************************************************************************/
