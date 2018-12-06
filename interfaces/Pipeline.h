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
#include "api/geom/IImage2WorldMapper.h"
#include "api/geom/I2DTransform.h"


#include <xpcf/threading/DropBuffer.h>
#include "xpcf/threading/BaseTask.h"


namespace xpcf  = org::bcom::xpcf;

using namespace SolAR::api;

namespace SolAR::MODULES::PIPELINE {

    class PIPELINE_EXPORT_API Pipeline : public org::bcom::xpcf::ConfigurableBase,
                                         public api::pipeline::IIPipeline
    {
        public:
            Pipeline();
            ~Pipeline();

//                    org::bcom::xpcf::XPCFErrorCode onConfigured() override final;
            void unloadComponent () override final;
            SolAR::api::pipeline::PipelineReturnCode init(const char* configXml) override;
            SolAR::api::pipeline::PipelineReturnCode update(std::pair<SRef<Image>, Transform3Df>& pipeLineOut) override;
            SolAR::api::pipeline::PipelineReturnCode start() override;
            SolAR::api::pipeline::PipelineReturnCode stop() override;

        private:

            SRef<input::devices::ICamera> m_camera;
            SRef<input::files::IMarker2DNaturalImage> m_marker;
            SRef<features::IKeypointDetector> m_kpDetector;
            SRef<features::IDescriptorMatcher> m_matcher;
            SRef<features::IMatchesFilter> m_basicMatchesFilter;
            SRef<features::IMatchesFilter> m_geomMatchesFilter;
            SRef<solver::pose::I2DTransformFinder> m_homographyEstimation ;
            SRef<solver::pose::IHomographyValidation> m_homographyValidation ;
            SRef<features::IKeypointsReIndexer> m_keypointsReindexer;
            SRef<solver::pose::I3DTransformFinderFrom2D3D> m_poseEstimation;
            SRef<features::IDescriptorsExtractor> m_descriptorExtractor;
            SRef<geom::IImage2WorldMapper> m_img_mapper;
            SRef<geom::I2DTransform> m_transform2D;

            SRef<Image> m_refImage;
            SRef<Image> m_camImage;
            bool m_stopFlag;
            int m_countFrame;

            SRef<DescriptorBuffer>  m_refDescriptors,  m_camDescriptors;
            std::vector<DescriptorMatch>  m_matches;

            Transform2Df  m_Hm;
            std::vector< SRef<Keypoint> >  m_refKeypoints,  m_camKeypoints;  // where to store detected keypoints in ref image and camera image

            std::vector<SRef <Point2Df>> m_refImgCorners;

            Transform3Df m_pose;

            // Threads : input/output buffers
            org::bcom::xpcf::DropBuffer< SRef<Image> >  m_workingBufferCamImages;
            org::bcom::xpcf::DropBuffer< std::pair<SRef<Image>, Transform3Df> > m_pipelineOutBuffer;

            xpcf::DelegateTask* m_taskGetProcessOneView;


    };

}

#endif // MODULEPIPELINE_H
