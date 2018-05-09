#pragma once
#if defined (WIN32)
    #if defined (_MSC_VER)
        #pragma warning(disable: 4251)
    #endif
    #if defined(BOX2D_DLL)
        #define  BOX2D_API __declspec(dllexport)
    #else
        #define  BOX2D_API __declspec(dllimport)
    #endif
#else
    #define BOX2D_API
#endif
