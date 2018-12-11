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

#include <Windows.h>
#include <iostream>
#include <string>
#include <vector>


#include <boost/log/core.hpp>
#include "api/display/IImageViewer.h"
#include "api/display/I3DOverlay.h"
#include "api/input/files/IMarker2DNaturalImage.h"

#include <thread>
#include <chrono>

// ADD MODULES TRAITS HEADERS HERE


// ADD XPCF HEADERS HERE
#include "xpcf/xpcf.h"

// ADD COMPONENTS HEADERS HERE


#include "core/Log.h"


using namespace SolAR;


#include "api/input/devices/ICamera.h"
#include "SolARModuleOpencv_traits.h"

using namespace SolAR::MODULES::OPENCV;
using namespace SolAR::api;
namespace xpcf  = org::bcom::xpcf;


#include "api/pipeline/IPipeline.h"
#include "SolARModulePipelineOpenCV_traits.h"

#ifdef linux

int main(int argc, char **argv) {

    std::cout << "hello\n";

    LOG_ADD_LOG_TO_CONSOLE();

    #if NDEBUG
        boost::log::core::get()->set_logging_enabled(false);
    #endif

    SRef<org::bcom::xpcf::IComponentManager> xpcfComponentManager = org::bcom::xpcf::getComponentManagerInstance();

    /* instantiate component manager*/
    /* this is needed in dynamic mode */

    std::string configxml = std::string("conf_TestModulePipelineOpenCV.xml");
    if (argc == 2)
        configxml = std::string(argv[1]);
    if (xpcfComponentManager->load(configxml.c_str()) != org::bcom::xpcf::_SUCCESS){

        LOG_ERROR("Failed to load the configuration file {}", configxml.c_str())
        return -1;

    }
    else{

        LOG_INFO("success to load the configuration file {}", configxml.c_str())
        std::cout << "success to load the configuration file \n";

    }

    auto pipeline=xpcfComponentManager->create<SolAR::MODULES::PIPELINE::Pipeline>()->bindTo<SolAR::api::pipeline::IIPipeline>();
    SRef<Image> camImage;
    Transform3Df pose;
    SolAR::api::pipeline::PipelineReturnCode retCode;

    auto camera = xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    auto imageViewerResult = xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
    auto overlay3DComponent = xpcfComponentManager->create<SolAR3DOverlayBoxOpencv>()->bindTo<display::I3DOverlay>();
    auto marker = xpcfComponentManager->create<SolARMarker2DNaturalImageOpencv>()->bindTo<input::files::IMarker2DNaturalImage>();

    std::pair<SRef<SolAR::datastructure::Image>, SolAR::datastructure::Transform3Df> result;


    LOG_INFO("LOAD MARKER IMAGE ");
    SRef<Image> refImage;

    marker->loadMarker();
    marker->getImage(refImage);
    // NOT WORKING ! Set the size of the box to the size of the natural image marker
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getWidth(),0);
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getHeight(),1);
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getHeight()/2.0f,2);

    CamCalibration intrinsic_param = camera->getIntrinsicsParameters();
    CamDistortion  distorsion_param = camera->getDistorsionParameters();
    overlay3DComponent->setCameraParameters(intrinsic_param, distorsion_param);


    pipeline->init("conf_NaturalImageMarker.xml");
    pipeline->start();
    clock_t start, end;
    int count = 0;
    start = clock();


    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        retCode=pipeline->update(result);
        if(retCode==SolAR::api::pipeline::PipelineReturnCode::_UPDATE_OK){
            count++;
            camImage = result.first->copy();
            pose = result.second;

            if(pose(3,3)!=0.f){
                LOG_INFO("a valid pose is available \n");
                overlay3DComponent->draw(pose, camImage);
                LOG_DEBUG("pose.matrix():\n {} \n",pose.matrix())
            }
            if (imageViewerResult->display(camImage) == SolAR::FrameworkReturnCode::_STOP){
                retCode=pipeline->stop();
                break;
            }
        }

    }

    // display stats on frame rate
    end = clock();
    double duration = double(end - start) / CLOCKS_PER_SEC;
    printf("\n\nElasped time is %.2lf seconds.\n", duration);
    printf("Number of processed frame per second : %8.2f\n", count / duration);

    return 0;
}


#else

typedef int(WINAPI* FNPTR_INIT)(const char *);
typedef int(WINAPI* FNPTR_START)(void);
typedef int(WINAPI* FNPTR_STOP)(void);
typedef int(WINAPI* FNPTR_UPDATE)(unsigned char* &imageData ,int& width, int& height, unsigned int& type, float* &pose);


int main(int argc, char **argv) {  //windows

    std::cout << "hello\n";

    LOG_ADD_LOG_TO_CONSOLE();

    #if NDEBUG
        boost::log::core::get()->set_logging_enabled(false);
    #endif

    //Enter the dll path here, after building "GetEvenOdd.sln" project. (Don't forget to out double backslash(\\) in the Path
#if NDEBUG
        HMODULE dll = LoadLibrary(L"C:\\SolARFramework\\SolARLibraries\\bcomBuild\\SolARModulePipelineOpenCV\\0.1.0\\lib\\x86_64\\shared\\release\\SolARModulePipelineOpenCV.dll");
#else
        HMODULE dll = LoadLibrary(L"C:\\SolARFramework\\SolARLibraries\\bcomBuild\\SolARModulePipelineOpenCV\\0.1.0\\lib\\x86_64\\shared\\debug\\SolARModulePipelineOpenCV.dll");
#endif

    if(!dll)
    {
        std::cout<<"\nCould Not Load the Library";
        return EXIT_FAILURE;
    }

    //Resolve the function address
    std::string configFileName=std::string("conf_NaturalImageMarker.xml");

    FNPTR_INIT fInit = (FNPTR_INIT)GetProcAddress(dll, "init");
    if(!fInit){
        std::cout<<"\nCould not locate the function : init \n ";
        return EXIT_FAILURE;
    }


    FNPTR_START fStart = (FNPTR_START)GetProcAddress(dll, "start");
    if(!fStart){
        std::cout<<"\nCould not locate the function : start";
        return EXIT_FAILURE;
    }

    FNPTR_STOP fStop = (FNPTR_STOP)GetProcAddress(dll, "stop");
    if(!fStop){
        std::cout<<"\nCould not locate the function : stop";
        return EXIT_FAILURE;
    }

    char* image;
    float* pose1[4][4];
    FNPTR_UPDATE fUpdate = (FNPTR_UPDATE)GetProcAddress(dll, "update");
    if(!fUpdate){
        std::cout<<"\nCould not locate the function : update ";
        return EXIT_FAILURE;
    }

    SRef<org::bcom::xpcf::IComponentManager> xpcfComponentManager = org::bcom::xpcf::getComponentManagerInstance();


    /* instantiate component manager*/
    /* this is needed in dynamic mode */

    std::string configxml = std::string("conf_TestModulePipelineOpenCV.xml");
    if (argc == 2)
        configxml = std::string(argv[1]);
    if (xpcfComponentManager->load(configxml.c_str()) != org::bcom::xpcf::_SUCCESS){

        LOG_ERROR("Failed to load the configuration file {}", configxml.c_str())
        if(dll)
            FreeLibrary(dll);

        return -1;

    }
    else{

        LOG_INFO("success to load the configuration file {}", configxml.c_str())
        std::cout << "success to load the configuration file \n";

    }


// the following is used only for display purpose [begin]

    auto camera = xpcfComponentManager->create<SolARCameraOpencv>()->bindTo<input::devices::ICamera>();
    auto imageViewerResult = xpcfComponentManager->create<SolARImageViewerOpencv>()->bindTo<display::IImageViewer>();
    auto overlay3DComponent = xpcfComponentManager->create<SolAR3DOverlayBoxOpencv>()->bindTo<display::I3DOverlay>();
    auto marker = xpcfComponentManager->create<SolARMarker2DNaturalImageOpencv>()->bindTo<input::files::IMarker2DNaturalImage>();

    LOG_INFO("LOAD MARKER IMAGE ");
    SRef<Image> refImage;

    marker->loadMarker();
    marker->getImage(refImage);

    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getWidth(),0);
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getHeight(),1);
    overlay3DComponent->bindTo<xpcf::IConfigurable>()->getProperty("size")->setFloatingValue(marker->getHeight()/2.0f,2);

    CamCalibration intrinsic_param = camera->getIntrinsicsParameters();
    CamDistortion  distorsion_param = camera->getDistorsionParameters();
    overlay3DComponent->setCameraParameters(intrinsic_param, distorsion_param);

// the following is used only for display purpose [end]


    SRef<Image> camImage;
    Transform3Df pose;

    fInit("conf_NaturalImageMarker.xml");
    fStart();
    clock_t start, end;
    int count = 0;
    start = clock();

    unsigned char* r_imageData;
    int width;
    int height;
    unsigned int type;
    float* r_pose;

    int retCode;
    while(true){
        retCode=fUpdate(r_imageData ,width,height,type,r_pose);
        if(retCode==0){
            count++;
            LOG_INFO(" w : {} , h :{} \n",width,height);
            for(int i=0;i<4;i++)
                for(int j=0;j<4;j++)
                    pose(i,j)=r_pose[i+4*j];
            camImage=xpcf::utils::make_shared<Image>(r_imageData,width,height,SolAR::Image::LAYOUT_BGR,SolAR::Image::INTERLEAVED,SolAR::Image::TYPE_8U);

            if(pose(3,3)!=0.f){
                LOG_INFO("a valid pose is available \n");
                overlay3DComponent->draw(pose, camImage);
                LOG_INFO("pose.matrix():\n {} \n",pose.matrix())
            }
            if (imageViewerResult->display(camImage) == SolAR::FrameworkReturnCode::_STOP){
                fStop();
                break;
            }
        }

    }

    // display stats on frame rate
    end = clock();
    double duration = double(end - start) / CLOCKS_PER_SEC;
    printf("\n\nElasped time is %.2lf seconds.\n", duration);
    printf("Number of processed frame per second : %8.2f\n", count / duration);

    if(dll)
        FreeLibrary(dll);

    return 0;
}

#endif


