//---------------------------------------------------------------------
//
// Copyright 2008, SensAble Technologies, Inc.
//
// File: QHWin32Class.h
//
// Author: Hari Vasudevan	
//
// Created: 09/11/2008
//
// Description: The Win32Class interfaces with the OpenGL class and creates 
// a Win32 window for rendering scenes in the uAPI
//
//---------------------------------------------------------------------

#ifndef WIN32CLASS_H
#define WIN32CLASS_H


#include "ExportDef.h"
#include "Globals.h"
#include "Cursor.h"
#include "Sphere.h"
#include "DeviceSpace.h"
#include "QHRenderer.h"




/*! \brief This a global function to start the rendering loops

This function sets the QuickHaptics rendering engine in motion. The function is invoked after defining all objects, effects and callbacks in the main/WinMain
function*/
void QHAPI qhStart(void);


/*! \brief This class defines Win32 specific display parameters. 

In addition to other function required to correctly display all primitives with their
associated transforms this class also supports multiple rendering windows. 
This feature is not available in glut windowing.
*/
class QHAPI QHWin32:public QHRenderer
{
	
	HINSTANCE	hInstance;
		WNDCLASS wc;
		HWND hWnd;
		HDC hDC;
		HGLRC hRC;
		static MSG msg;
		static int m_WindowNumber;
		static bool m_InitialSettings;
		bool m_InitialWorkspace;
		bool m_HapticSpaceFlag;

		friend LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
		friend void qhStart(void);

		
		void enableOpenGL(HWND hWnd, HDC * hDC, HGLRC * hRC);// enable OpenGL for the window
		void disableOpenGL(HWND hWnd, HDC hDC, HGLRC hRC);// Disable OpenGL
		void create(void);
		
		static const char* getErrorCodeName(HDerror errorCode);
		static void win32ErrorCatcher(void);
		static void win32ReshapeFunc(int width, int height, HWND hWnd);
		static WPARAM win32MainLoop(void);
		///This function checks if the user has made any changes to the title of the window. And sets that string as the title
        void checkAndSetTitle(void);


		void (*m_pGraphicLoopCallbackPointer)(void);

	

	public:

		QHWin32(void);//Setup window with constructor
		~QHWin32(void);//Destroy window with destructor

		/*! Search for window by name

		This function will search for the widow by "name". The name for the window can be set using the QHRenderer::setName function.*/
        static QHWin32* searchWindow(char* WindowName);	
        
        ///This function returns the windows handlers to the programmer.
        void getWindowParameters(HINSTANCE*	hInstance, WNDCLASS* wc, HWND* hWnd, HDC* hDC, HGLRC* hRC);
		
		/*! \brief Function used in a multiple window application to identify the haptic window
		
		The hapticWindow function is used when the application uses multiple windows. This function must be set by the programmer for EACH window.
		The function helps the QuickHaptics uAPI identify if the window is a haptic window or just a graphic window. The direction of reaction force
		is determined by the active haptic window. \n
		Note: \n
		1) ONE window MUST be declared true and all other windows MUST be declared false\n
		2) All windows should NOT be declared false.*/
		void hapticWindow(bool m_HapticSpaceFlag);

		/*! \brief This function defines a graphic callback that is invoked on every graphic frame.	
		
		There can exist only ONE graphic callback function per window. Multiple calls to the preDrawCallback function will schedule 
		the function declared last as the Graphics Callback*/
		void preDrawCallback(void (*m_pGraphicLoopCallbackPointer)(void));
		
		
};


LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

#endif

