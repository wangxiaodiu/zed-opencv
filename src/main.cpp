///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2016, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////




/***************************************************************************************************
 ** This sample demonstrates how to grab images and depth/disparity map with the ZED SDK          **
 ** Both images and depth/disparity map are displayed with OpenCV                                 **
 ** Most of the functions of the ZED SDK are linked with a key press event (using OpenCV)         **
 ***************************************************************************************************/


#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>

// darknet api header files
#include "arapaho.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#define MAX_OBJECTS_PER_FRAME (100)
#include <string>
// darkent end

#include <opencv2/opencv.hpp>

#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

//
// Some configuration inputs
//
static char INPUT_DATA_FILE[]    = "input.data"; 
static char INPUT_CFG_FILE[]     = "input.cfg";
static char INPUT_WEIGHTS_FILE[] = "input.weights";
static char INPUT_IMAGE_FILE[]   = "input.jpg";

typedef struct mouseOCVStruct {
    float* data;
    uint32_t step;
    cv::Size _image;
    cv::Size _resize;
    std::string name;
    std::string unit;
} mouseOCV;

mouseOCV mouseStruct;

static void onMouseCallback(int32_t event, int32_t x, int32_t y, int32_t flag, void * param) {
    if (event == CV_EVENT_LBUTTONDOWN) {
        mouseOCVStruct* data = (mouseOCVStruct*) param;

        int y_int = (y * data->_image.height / data->_resize.height);
        int x_int = (x * data->_image.width / data->_resize.width);

        float* ptr_image_num = (float*) ((int8_t*) data->data + y_int * data->step);
        float dist = ptr_image_num[x_int];

        if (isValidMeasure(dist))
            printf("\n%s : %2.2f %s\n", data->name.c_str(), dist, data->unit.c_str());
        else {
            if (dist == TOO_FAR)
                printf("\n%s is too far.\n", data->name.c_str(), dist, data->unit.c_str());
            else if (dist == TOO_CLOSE)
                printf("\n%s is too close.\n", data->name.c_str(), dist, data->unit.c_str());
            else
                printf("\n%s not avaliable\n", data->name.c_str(), dist, data->unit.c_str());
        }
    }
}

int main(int argc, char **argv) {

    if (argc > 3) {
        std::cout << "Only the path of a SVO or a InitParams file can be passed in arg." << std::endl;
        return -1;
    }

    // Quick check input arguments
    bool readSVO = false;
    std::string SVOName;
    bool loadParams = false;
    std::string ParamsName;
    if (argc > 1) {
        std::string _arg;
        for (int i = 1; i < argc; i++) {
            _arg = argv[i];
            if (_arg.find(".svo") != std::string::npos) {
                // If a SVO is given we save its name
                readSVO = true;
                SVOName = _arg;
            }
            if (_arg.find(".ZEDinitParam") != std::string::npos) {
                // If a parameter file is given we save its name
                loadParams = true;
                ParamsName = _arg;
            }
        }
    }

    sl::zed::Camera* zed;

    if (!readSVO) // Live Mode
        zed = new sl::zed::Camera(sl::zed::HD720);
    else // SVO playback mode
        zed = new sl::zed::Camera(SVOName);

    // Define a struct of parameters for the initialization
    sl::zed::InitParams params;

    if (loadParams) // A parameters file was given in argument, we load it
        params.load(ParamsName);

    // Enables verbosity in the console
    params.verbose = true;


    sl::zed::ERRCODE err = zed->init(params);
    std::cout << "Error code : " << sl::zed::errcode2str(err) << std::endl;
    if (err != sl::zed::SUCCESS) {
        // Exit if an error occurred
        delete zed;
        return 1;
    }

    // Save the initialization parameters
    // The file can be used later in any zed based application
    params.save("MyParam");

    char key = ' ';
    int viewID = 0;
    int confidenceThres = 100;

    bool displayDisp = true;
    bool displayConfidenceMap = false;

    int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;

    cv::Mat disp(height, width, CV_8UC4);
    cv::Mat anaglyph(height, width, CV_8UC4);
    cv::Mat confidencemap(height, width, CV_8UC4);

    cv::Size displaySize(720*2, 404*2);
    // cv::Size displaySize(416, 416);
    cv::Mat dispDisplay(displaySize, CV_8UC4);
    cv::Mat anaglyphDisplay(displaySize, CV_8UC4);
    cv::Mat confidencemapDisplay(displaySize, CV_8UC4);

    sl::zed::SENSING_MODE dm_type = sl::zed::STANDARD;

    // Mouse callback initialization
    sl::zed::Mat depth;
    zed->grab(dm_type);
    depth = zed->retrieveMeasure(sl::zed::MEASURE::DEPTH); // Get the pointer
    // Set the structure
    mouseStruct._image = cv::Size(width, height);
    mouseStruct._resize = displaySize;
    mouseStruct.data = (float*) depth.data;
    mouseStruct.step = depth.step;
    mouseStruct.name = "DEPTH";
    mouseStruct.unit = unit2str(params.unit);

    // The depth is limited to 20 METERS, as defined in zed::init()
    zed->setDepthClampValue(10000);

    // Create OpenCV Windows
    // NOTE: You may encounter an issue with OpenGL support, to solve it either
    // 	use the default rendering by removing ' | cv::WINDOW_OPENGL' from the flags
    //	or recompile OpenCV with OpenGL support (you may also need the gtk OpenGL Extension
    //	on Linux, provided by the packages libgtkglext1 libgtkglext1-dev)
    cv::namedWindow(mouseStruct.name, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(mouseStruct.name, onMouseCallback, (void*) &mouseStruct);
    cv::namedWindow("VIEW", cv::WINDOW_AUTOSIZE);

    //cv::resizeWindow("VIEW", 10240, 7200);
    // cv::resizeWindow(mouseStruct.name, 10240, 7200);

    std::cout << "Press 'q' to exit" << std::endl;

    // Jetson only. Execute the calling thread on core 2
    sl::zed::Camera::sticktoCPUCore(2);

    sl::zed::ZED_SELF_CALIBRATION_STATUS old_self_calibration_status = sl::zed::SELF_CALIBRATION_NOT_CALLED;

    /******************** DARKNET-CPP API *****************************/
    // darknet-cpp api
    const bool DEBUG = true;
    box* boxes = 0;
    std::string * labels;
    ArapahoV2* p = new ArapahoV2();
    if(!p)
      {
        return -1;
      }
    ArapahoV2Params ap;
    ap.datacfg = INPUT_DATA_FILE;
    ap.cfgfile = INPUT_CFG_FILE;
    ap.weightfile = INPUT_WEIGHTS_FILE;
    ap.nms = 0.4;
    ap.maxClasses = 2;
    // Always setup before detect
    int expectedW = 0, expectedH = 0;
    bool ret = false;
    ret = p->Setup(ap, expectedW, expectedH);
    if(false == ret)
      {
        printf("Setup failed!\n");
        if(p) delete p;
        p = 0;
        return -1;
      }
    ArapahoV2ImageBuff arapahoImage;
    cv::Size imageSize(416, 416);
    cv::Mat image(imageSize, CV_8UC4);
    int numObjects = 0;
    /******************** DARKNET-CPP API END **************************/

    // Loop until 'q' is pressed
    while (key != 'q') {
        // Disparity Map filtering
        zed->setConfidenceThreshold(confidenceThres);

        // Get frames and launch the computation
        bool res = zed->grab(dm_type);

        if (!res) {
            if (old_self_calibration_status != zed->getSelfCalibrationStatus()) {
                std::cout << "Self Calibration Status : " << sl::zed::statuscode2str(zed->getSelfCalibrationStatus()) << std::endl;
                old_self_calibration_status = zed->getSelfCalibrationStatus();
            }

            depth = zed->retrieveMeasure(sl::zed::MEASURE::DEPTH); // Get the pointer

            // The following is the best way to retrieve a disparity map / image / confidence map in OpenCV Mat.
            // If the buffer is not duplicated, it will be replaced by a next retrieve (retrieveImage, normalizeMeasure, getView...)
            // Disparity, depth, confidence are 32F buffer by default and 8UC4 buffer in normalized format (displayable grayscale)


            // -- The next part is about displaying the data --

            // Normalize the disparity / depth map in order to use the full color range of gray level image
            if (displayDisp)
                slMat2cvMat(zed->normalizeMeasure(sl::zed::MEASURE::DISPARITY)).copyTo(disp);
            else
                slMat2cvMat(zed->normalizeMeasure(sl::zed::MEASURE::DEPTH)).copyTo(disp);

            // To get the depth at a given position, click on the disparity / depth map image
            cv::resize(disp, dispDisplay, displaySize);

            if (displayConfidenceMap) {
                slMat2cvMat(zed->normalizeMeasure(sl::zed::MEASURE::CONFIDENCE)).copyTo(confidencemap);
                cv::resize(confidencemap, confidencemapDisplay, displaySize);
                imshow("confidence", confidencemapDisplay);
            }

            // 'viewID' can be 'SIDE mode' or 'VIEW mode'
            if (viewID >= sl::zed::LEFT && viewID < sl::zed::LAST_SIDE)
                slMat2cvMat(zed->retrieveImage(static_cast<sl::zed::SIDE> (viewID))).copyTo(anaglyph);
            else
                slMat2cvMat(zed->getView(static_cast<sl::zed::VIEW_MODE> (viewID - (int) sl::zed::LAST_SIDE))).copyTo(anaglyph);

            cv::resize(anaglyph, anaglyphDisplay, displaySize);

            /**************DARKNET API**************************/
            cv::resize(anaglyph, image, imageSize);
            // printf("Image data = %p, w = %d, h = %d\n", image.data, image.size().width, image.size().height);
            arapahoImage.bgr = image.data;
            arapahoImage.w = image.size().width;
            arapahoImage.h = image.size().height;
            arapahoImage.channels = 4;
            // Detect the objects in the image
            p->Detect(arapahoImage,
                      0.24,
                      0.5,
                      numObjects);
            if(DEBUG){
              printf("Detected %d objects\n", numObjects);
            }
            if(numObjects > 0 && numObjects < MAX_OBJECTS_PER_FRAME) // Realistic maximum
              {
                boxes = new box[numObjects];
                labels = new std::string[numObjects];
                if(!boxes)
                  {
                    printf("Nothing detected\n");
                    // if(p) delete p;
                    // p = 0;
                    // return -1;
                  }
                p->GetBoxes(boxes,
                            numObjects,
                            labels);
                int left, right, top, bot;
                cv::Scalar rect_color(255,0,0);
                for(int i=0; i<numObjects; ++i){
                  if(DEBUG) {
                    std::cout << "labes:" << labels[i];
                    printf("Box #%d: x,y,w,h = [%f, %f, %f, %f]\n", i, boxes[i].x, boxes[i].y, boxes[i].w, boxes[i].h);
                    std::cout << std::endl;
                  }
                  left  = (boxes[i].x-boxes[i].w/2.)*displaySize.width;
                  right = (boxes[i].x+boxes[i].w/2.)*displaySize.width;
                  top   = (boxes[i].y-boxes[i].h/2.)*displaySize.height;
                  bot   = (boxes[i].y+boxes[i].h/2.)*displaySize.height;
                  cv::rectangle(dispDisplay, cv::Point(left, bot), cv::Point(right, top), rect_color, 5);
                  cv::rectangle(anaglyphDisplay, cv::Point(left, bot), cv::Point(right, top), rect_color, 5);
                }
              }
            /**************DARKNET API**************************/

            imshow("VIEW", anaglyphDisplay);
            imshow(mouseStruct.name, dispDisplay);

            key = cv::waitKey(5);

            // Keyboard shortcuts
            switch (key) {
                case 'b':
                    if (confidenceThres >= 10)
                        confidenceThres -= 10;
                    break;
                case 'n':
                    if (confidenceThres <= 90)
                        confidenceThres += 10;
                    break;
                    // From 'SIDE' enum
                case '0': // Left
                    viewID = 0;
                    std::cout << "Current View switched to Left (rectified/aligned)" << std::endl;
                    break;
                case '1': // Right
                    viewID = 1;
                    std::cout << "Current View switched to Right (rectified/aligned)" << std::endl;
                    break;
                    // From 'VIEW' enum
                case '2': // Side by Side
                    viewID = 10;
                    std::cout << "Current View switched to Side by Side mode" << std::endl;
                    break;
                case '3': // Overlay
                    viewID = 11;
                    std::cout << "Current View switched to Overlay mode" << std::endl;
                    break;
                case '4': // Difference
                    viewID = 9;
                    std::cout << "Current View switched to Difference mode" << std::endl;
                    break;
                case '5': // Anaglyph
                    viewID = 8;
                    std::cout << "Current View switched to Anaglyph mode" << std::endl;
                    break;
                case 'c':
                    displayConfidenceMap = !displayConfidenceMap;
                    break;
                case 's':
                    dm_type = (dm_type == sl::zed::SENSING_MODE::STANDARD) ? sl::zed::SENSING_MODE::FILL : sl::zed::SENSING_MODE::STANDARD;
                    std::cout << "SENSING_MODE " << sensing_mode2str(dm_type) << std::endl;
                    break;
                case 'd':
                    displayDisp = !displayDisp;
                    break;
            }
        } else key = cv::waitKey(5);
    }

    delete zed;
    return 0;
}
