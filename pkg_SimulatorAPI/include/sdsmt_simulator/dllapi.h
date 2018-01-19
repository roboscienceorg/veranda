#pragma once

#include <QtGlobal>

#if defined (WIN32)
    #if defined (_MSC_VER)
        #pragma warning(disable: 4251)
    #endif
    #if defined(SDSMT_SIMULATOR_DLL)
        #define  SDSMT_SIMULATOR_API Q_DECL_EXPORT
    #else
        #define  SDSMT_SIMULATOR_API Q_DECL_IMPORT
    #endif
#else
    #define SDSMT_SIMULATOR_API
#endif
