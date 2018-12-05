/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>

#include <boost/log/core.hpp>


#include "Pipeline.h"
#include "SolARModuleOpencv_traits.h"
#include "SolARModuleTools_traits.h"

#include "xpcf/xpcf.h"
#include "xpcf/threading/SharedBuffer.h"
#include "xpcf/threading/BaseTask.h"
#include <xpcf/threading/DropBuffer.h>


namespace xpcf  = org::bcom::xpcf;

using namespace SolAR;
using namespace SolAR::MODULES::TOOLS;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::datastructure;
using namespace SolAR::api;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::PIPELINE::Pipeline)

namespace SolAR {
	namespace MODULES {
        namespace PIPELINE {


			Pipeline::Pipeline():ConfigurableBase(xpcf::toUUID<Pipeline>()){
               addInterface<api::pipeline::IIPipeline>(this);
			   SRef<xpcf::IPropertyMap> params = getPropertyRootNode();
			   LOG_DEBUG(" Pipeline constructor");
			}

            Pipeline::~Pipeline()
            {
                LOG_DEBUG(" Pipeline destructor")
            }

            FrameworkReturnCode Pipeline::init(const char* confFileName) {
                std::cout << "Pipeline Init: confFileName = " << confFileName << "\n";

                SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

                if (xpcfComponentManager->load(confFileName) != org::bcom::xpcf::_SUCCESS){

                    LOG_ERROR("Failed to load the configuration file {}", confFileName)
                    return FrameworkReturnCode::_ERROR_;

                }

                camera = xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
                kpDetector = xpcfComponentManager->create<SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
                marker = xpcfComponentManager->create<SolARMarker2DNaturalImageOpencv>()->bindTo<input::files::IMarker2DNaturalImage>();
                matcher = xpcfComponentManager->create<SolARDescriptorMatcherKNNOpencv>()->bindTo<features::IDescriptorMatcher>();
                basicMatchesFilter = xpcfComponentManager->create<SolARBasicMatchesFilter>()->bindTo<features::IMatchesFilter>();
                geomMatchesFilter = xpcfComponentManager->create<SolARGeometricMatchesFilterOpencv>()->bindTo<features::IMatchesFilter>();
                homographyEstimation = xpcfComponentManager->create<SolARHomographyEstimationOpencv>()->bindTo<solver::pose::I2DTransformFinder>();
                homographyValidation = xpcfComponentManager->create<SolARHomographyValidation>()->bindTo<solver::pose::IHomographyValidation>();
                keypointsReindexer = xpcfComponentManager->create<SolARKeypointsReIndexer>()->bindTo<features::IKeypointsReIndexer>();
                poseEstimation = xpcfComponentManager->create<SolARPoseEstimationPnpOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D3D>();

                return FrameworkReturnCode::_SUCCESS;
            }

            FrameworkReturnCode Pipeline::stop() {
                std::cout << "Pipeline Stop: \n";
                return FrameworkReturnCode::_SUCCESS;
            }

            FrameworkReturnCode Pipeline::update() {
                std::cout << "Pipeline Update: \n";
                return FrameworkReturnCode::_SUCCESS;
            }

/*
 *
 *
 *
 */
            FrameworkReturnCode Pipeline::start() {

                std::function<void(void)> getCameraImages = [&](){
                    SRef<Image> view;
                    if (camera->getNextImage(view) == SolAR::FrameworkReturnCode::_ERROR_LOAD_IMAGE) {
                        m_stopFlag = true;
                        return;
                    }
                    if(m_workingBufferCamImages.empty())
                         m_workingBufferCamImages.push(view);
                };
                std::cout << "Pipeline Start: \n";

                if (camera->start() != FrameworkReturnCode::_SUCCESS)
                {
                    LOG_ERROR("Camera cannot start");
                    return FrameworkReturnCode::_ERROR_;
                }

                xpcf::DelegateTask taskGetCameraImages(getCameraImages);
                taskGetCameraImages.start();

                return FrameworkReturnCode::_SUCCESS;
            }
		}
	}
}
