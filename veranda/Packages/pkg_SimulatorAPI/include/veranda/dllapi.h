#pragma once

#include <QtGlobal>

#if defined (WIN32)
    #if defined (_MSC_VER)
        #pragma warning(disable: 4251)
    #endif
    #if defined(veranda_DLL)
        #define  veranda_API Q_DECL_EXPORT
    #else
        #define  veranda_API Q_DECL_IMPORT
    #endif
#else
    #define veranda_API
#endif
