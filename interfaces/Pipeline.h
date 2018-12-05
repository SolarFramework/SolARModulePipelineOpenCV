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

#ifndef MODULEPIPELINE_H
#define MODULEPIPELINE_H

#include "api/pipeline/IPipeline.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolARModulePipelineOpenCVAPI.h"

#include "api/input/devices/ICamera.h"
#include "api/input/files/IMarker2DNaturalImage.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I2DTransformFinder.h"
#include "api/solver/pose/IHomographyValidation.h"
#include "api/features/IKeypointsReIndexer.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"


#include <xpcf/threading/DropBuffer.h>

using namespace SolAR::api;

namespace SolAR {
	namespace MODULES {
		namespace PIPELINE {

            class PIPELINE_EXPORT_API Pipeline : public org::bcom::xpcf::ConfigurableBase,
                                                 public api::pipeline::IIPipeline
			{
				public:
					Pipeline();
                    ~Pipeline();

//                    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;
                    void unloadComponent () override final;
                    FrameworkReturnCode init(const char* configXml) override;
                    FrameworkReturnCode update() override;
                    FrameworkReturnCode start() override;
                    FrameworkReturnCode stop() override;

				private:

                    SRef<input::devices::ICamera> camera;
                    SRef<input::files::IMarker> marker;
                    SRef<features::IKeypointDetector> kpDetector;
                    SRef<features::IDescriptorMatcher> matcher;
                    SRef<features::IMatchesFilter> basicMatchesFilter;
                    SRef<features::IMatchesFilter> geomMatchesFilter;
                    SRef<solver::pose::I2DTransformFinder> homographyEstimation ;
                    SRef<solver::pose::IHomographyValidation> homographyValidation ;
                    SRef<features::IKeypointsReIndexer> keypointsReindexer;
                    SRef<solver::pose::I3DTransformFinderFrom2D3D> poseEstimation;

                    bool m_stopFlag;
                    org::bcom::xpcf::DropBuffer< SRef<Image> >  m_workingBufferCamImages;

            };


		}
	}

}

#endif // MODULEPIPELINE_H
