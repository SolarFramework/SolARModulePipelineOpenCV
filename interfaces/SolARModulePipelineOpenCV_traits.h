
#ifndef MODULE_PIPELINE_OPENCV_TRAITS_H
#define MODULE_PIPELINE_OPENCV_TRAITS_H

#include "xpcf/api/IComponentManager.h"

namespace SolAR {
	namespace MODULES {
        namespace PIPELINE {
            class Pipeline;
		}
	}
}

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::PIPELINE::Pipeline,
                             "4bf453a3-5dee-451a-907c-472a278be26c","PipelineOPenCV",
                             "SolAR::MODULES::PIPELINE::pipeline definition")




#endif // MODULE_PIPELINE_OPENCV_TRAITS_H

