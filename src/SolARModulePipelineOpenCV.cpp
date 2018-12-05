#include "xpcf/module/ModuleFactory.h"

#include "Pipeline.h"

namespace xpcf=org::bcom::xpcf;

XPCF_DECLARE_MODULE("6430e0af-f3f9-4643-baf6-a7e21c581d1c", "SolarModulePipelineOpenCV", "SolarModulePipelineOpenCV")


extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID,SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
     xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
     errCode = xpcf::tryCreateComponent<SolAR::MODULES::PIPELINE::Pipeline>(componentUUID,interfaceRef);
     return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::MODULES::PIPELINE::Pipeline)
XPCF_END_COMPONENTS_DECLARATION
