/**
 * Copyright and license notice ......
 */

#ifndef PIPELINE_API_H
    #define PIPELINE_API_H
    #if _WIN32
        #ifdef SolARModulePipelineOpenCV_API_DLLEXPORT
            #define PIPELINE_EXPORT_API __declspec(dllexport)
        #else //Pipeline_API_DLLEXPORT
            #define PIPELINE_EXPORT_API __declspec(dllimport)
        #endif //Pipeline_API_DLLEXPORT
    #else //_WIN32
        #define PIPELINE_EXPORT_API
    #endif //_WIN32
    #include "SolARModulePipelineOpenCV_traits.h"
#endif //PIPELINE_API_H
