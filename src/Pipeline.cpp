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
#include <xpcf/threading/DropBuffer.h>


namespace xpcf  = org::bcom::xpcf;

using namespace SolAR;
using namespace SolAR::MODULES::TOOLS;
using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::datastructure;
using namespace SolAR::api;
using namespace SolAR::api::solver::pose;
using namespace SolAR::api::pipeline;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::PIPELINE::Pipeline)


namespace SolAR::MODULES::PIPELINE {

    /*
    *
    *******************************************************************************************************
    *
    */
    Pipeline::Pipeline():ConfigurableBase(xpcf::toUUID<Pipeline>()){
       addInterface<api::pipeline::IIPipeline>(this);
       SRef<xpcf::IPropertyMap> params = getPropertyRootNode();

       m_countFrame=0;
       LOG_DEBUG(" Pipeline constructor");
    }

    /*
    *
    *******************************************************************************************************
    *
    */
    Pipeline::~Pipeline()
    {
        LOG_DEBUG(" Pipeline destructor")
    }

    /*
    *
    *******************************************************************************************************
    *
    */
    PipelineReturnCode Pipeline::init(const char* confFileName) {
        LOG_INFO("Pipeline Init: confFileName = {} \n",confFileName);

        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

        if (xpcfComponentManager->load(confFileName) != org::bcom::xpcf::_SUCCESS){

            LOG_ERROR("Failed to load the configuration file {}", confFileName)
            return PipelineReturnCode::_INIT_FAILED;

        }

        m_camera = xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
        m_kpDetector = xpcfComponentManager->create<SolARKeypointDetectorOpencv>()->bindTo<features::IKeypointDetector>();
        m_marker = xpcfComponentManager->create<SolARMarker2DNaturalImageOpencv>()->bindTo<input::files::IMarker2DNaturalImage>();
        m_matcher = xpcfComponentManager->create<SolARDescriptorMatcherKNNOpencv>()->bindTo<features::IDescriptorMatcher>();
        m_basicMatchesFilter = xpcfComponentManager->create<SolARBasicMatchesFilter>()->bindTo<features::IMatchesFilter>();
        m_geomMatchesFilter = xpcfComponentManager->create<SolARGeometricMatchesFilterOpencv>()->bindTo<features::IMatchesFilter>();
        m_homographyEstimation = xpcfComponentManager->create<SolARHomographyEstimationOpencv>()->bindTo<solver::pose::I2DTransformFinder>();
        m_homographyValidation = xpcfComponentManager->create<SolARHomographyValidation>()->bindTo<solver::pose::IHomographyValidation>();
        m_keypointsReindexer = xpcfComponentManager->create<SolARKeypointsReIndexer>()->bindTo<features::IKeypointsReIndexer>();
        m_poseEstimation = xpcfComponentManager->create<SolARPoseEstimationPnpOpencv>()->bindTo<solver::pose::I3DTransformFinderFrom2D3D>();
        m_descriptorExtractor =  xpcfComponentManager->create<SolARDescriptorsExtractorAKAZE2Opencv>()->bindTo<features::IDescriptorsExtractor>();
        m_img_mapper = xpcfComponentManager->create<SolARImage2WorldMapper4Marker2D>()->bindTo<geom::IImage2WorldMapper>();
        m_transform2D = xpcfComponentManager->create<SolAR2DTransform>()->bindTo<geom::I2DTransform>();

        // load marker
        LOG_INFO("LOAD MARKER IMAGE ");
        m_marker->loadMarker();
        m_marker->getImage(m_refImage);

        // detect keypoints in reference image
        LOG_INFO("DETECT MARKER KEYPOINTS ");
        m_kpDetector->detect(m_refImage, m_refKeypoints);

        // extract descriptors in reference image
        LOG_INFO("EXTRACT MARKER DESCRIPTORS ");
        m_descriptorExtractor->extract(m_refImage, m_refKeypoints, m_refDescriptors);
        LOG_INFO("EXTRACT MARKER DESCRIPTORS COMPUTED");

        if (m_camera->start() != FrameworkReturnCode::_SUCCESS)
        {
            LOG_ERROR("Camera cannot start");
            return PipelineReturnCode::_INIT_FAILED;
        }

        // initialize pose estimation
        m_poseEstimation->setCameraParameters(m_camera->getIntrinsicsParameters(), m_camera->getDistorsionParameters());

        // initialize image mapper with the reference image size and marker size

        int refImageWidth,refImageheight;
        float worldWidth,worldHeight;
        refImageWidth=m_refImage->getSize().width;
        refImageheight=m_refImage->getSize().height;
        worldWidth=m_marker->getSize().width;
        worldHeight=m_marker->getSize().height;

        LOG_INFO(" worldWidth : {} worldHeight : {} \n",worldWidth,worldHeight)

        m_img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalWidth")->setIntegerValue(m_refImage->getSize().width);
        m_img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("digitalHeight")->setIntegerValue(m_refImage->getSize().height);
        m_img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("worldWidth")->setFloatingValue(m_marker->getSize().width);
        m_img_mapper->bindTo<xpcf::IConfigurable>()->getProperty("worldHeight")->setFloatingValue(m_marker->getSize().height);


        Point2Df corner0(0, 0);
        Point2Df corner1((float)m_refImage->getWidth(), 0);
        Point2Df corner2((float)m_refImage->getWidth(), (float)m_refImage->getHeight());
        Point2Df corner3(0, (float)m_refImage->getHeight());
        m_refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner0));
        m_refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner1));
        m_refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner2));
        m_refImgCorners.push_back(xpcf::utils::make_shared<Point2Df>(corner3));

        m_stopFlag=false;

        std::function<void(void)> processOneView = [&](){

            if( m_stopFlag == true)
                return;

            if (m_camera->getNextImage(m_camImage) == SolAR::FrameworkReturnCode::_ERROR_LOAD_IMAGE) {
                m_stopFlag = true;
                return;
            }

            std::cout << ++m_countFrame << "\n";

            // detect keypoints in camera image
            m_kpDetector->detect(m_camImage, m_camKeypoints);

            /* extract descriptors in camera image*/
            m_descriptorExtractor->extract(m_camImage, m_camKeypoints, m_camDescriptors);

            /*compute matches between reference image and camera image*/
            m_matcher->match(m_refDescriptors, m_camDescriptors, m_matches);

            /* filter matches to remove redundancy and check geometric validity */
            m_basicMatchesFilter->filter(m_matches, m_matches, m_refKeypoints, m_camKeypoints);
            m_geomMatchesFilter->filter(m_matches, m_matches, m_refKeypoints, m_camKeypoints);

            std::vector <SRef<Point2Df>> ref2Dpoints;
            std::vector <SRef<Point2Df>> cam2Dpoints;
            std::vector <SRef<Point3Df>> ref3Dpoints;
            Transform2Df Hm;
            std::vector <SRef<Point2Df>> markerCornersinCamImage;
            std::vector <SRef<Point3Df>> markerCornersinWorld;


            if (m_matches.size()> 10) {
                // reindex the keypoints with established correspondence after the matching
                m_keypointsReindexer->reindex(m_refKeypoints, m_camKeypoints, m_matches, ref2Dpoints, cam2Dpoints);

                // mapping to 3D points
                m_img_mapper->map(ref2Dpoints, ref3Dpoints);

                Transform2DFinder::RetCode res = m_homographyEstimation->find(ref2Dpoints, cam2Dpoints, Hm);
                //test if a meaningful matrix has been obtained
                if (res == Transform2DFinder::RetCode::TRANSFORM2D_ESTIMATION_OK)
                {
                    //poseEstimation->poseFromHomography(Hm,pose,objectCorners,sceneCorners);
                    // vector of 2D corners in camera image
                    m_transform2D->transform(m_refImgCorners, Hm, markerCornersinCamImage);
                    // draw circles on corners in camera image
                    //overlay2DComponent->drawCircles(markerCornersinCamImage, 10, 5, kpImageCam);

                    /* we verify is the estimated homography is valid*/
                    if (m_homographyValidation->isValid(m_refImgCorners, markerCornersinCamImage))
                    {
                        // from the homography we create 4 points at the corners of the reference image
                        // map corners in 3D world coordinatesf
                        m_img_mapper->map(m_refImgCorners, markerCornersinWorld);

                        // pose from solvePNP using 4 points.
                        /* The pose could also be estimated from all the points used to estimate the homography */
                        m_poseEstimation->estimate(markerCornersinCamImage, markerCornersinWorld, m_pose);

                        /* The pose last parameter can not be 0, so this is an error case*/
                        if (m_pose(3, 3) != 0.0)
                        {
                            if(m_pipelineOutBuffer.empty())
                                 m_pipelineOutBuffer.push(std::make_pair(m_camImage,m_pose));
                            LOG_INFO("valid pose detected for this frame");
                        }
                        else
                        {
                            /* The pose estimated is false: error case*/
                            LOG_INFO("no pose detected for this frame");
                        }
                    }
                    else /* when homography is not valid*/
                        LOG_INFO("Wrong homography for this frame");
                }
            }
            return;
        };

        m_taskGetProcessOneView = new xpcf::DelegateTask(processOneView);

        return PipelineReturnCode::_SUCCESS;
    }
    /*
    *
    *******************************************************************************************************
    *
    */
    PipelineReturnCode Pipeline::update(std::pair<SRef<Image>, Transform3Df>& pipeLineOut) {

        std::pair<SRef<Image>, Transform3Df>  element;

        if (!m_pipelineOutBuffer.tryPop(element) ){
            return PipelineReturnCode::_UPDATE_EMPTY;
        }

        pipeLineOut=element;

        return PipelineReturnCode::_UPDATE_OK;
    }
    /*
    *
    *******************************************************************************************************
    *
    */
    PipelineReturnCode Pipeline::start() {

        m_stopFlag=false;
        m_taskGetProcessOneView->start();
        LOG_INFO("Pipeline has Started: \n");
        return PipelineReturnCode::_SUCCESS;
    }
    /*
    *
    *******************************************************************************************************
    *
    */
    PipelineReturnCode Pipeline::stop() {

        m_stopFlag=true;
        m_taskGetProcessOneView->stop();
        LOG_INFO("Pipeline has stopped: \n");
        return PipelineReturnCode::_SUCCESS;
    }

}
