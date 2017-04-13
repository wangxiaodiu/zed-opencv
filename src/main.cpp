///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
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

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <sl/Core.hpp>
#include <sl/defines.hpp>

// darknet api header files and configuration
#include <string>
#include <fstream>
#include <ctime>
#include "arapaho.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#define MAX_OBJECTS_PER_FRAME (100)

static char INPUT_DATA_FILE[]    = "input.data"; 
static char INPUT_CFG_FILE[]     = "input.cfg";
static char INPUT_WEIGHTS_FILE[] = "input.weights";

cv::Size displaySize(720*3, 404*3);
void normalizeBoxes(box& box)
{
  float &x = box.x; float &y = box.y; float &w = box.w; float &h = box.h; 
  if(x>1) x = 1; if(y>1) y = 1; if(w>1) w = 1; if(h>1) h = 1; 
  if(x<0) x = 0; if(y<0) y = 0; if(w<0) w = 0; if(h<0) h = 0;
}

void mapcolor(float dist, cv::Scalar & color)
{
  if(dist < 30.0)
    color = cv::Scalar(255,255,255);
  else if(dist < 50.0)
    color = cv::Scalar(0,0,255); // pure red
  else if(dist < 100.0)
    color = cv::Scalar(0,150,150);
  else if(dist < 200.0)
    color = cv::Scalar(0,255,0);
  else if(dist < 500.0)
    color = cv::Scalar(150,150,0);
  else
    color = cv::Scalar(255,0,0);
}
// darkent end

using namespace sl;

typedef struct mouseOCVStruct {
    Mat depth;
    cv::Size _resize;
} mouseOCV;

mouseOCV mouseStruct;

static void onMouseCallback(int32_t event, int32_t x, int32_t y, int32_t flag, void * param);
cv::Mat slMat2cvMat(sl::Mat& input);

int main(int argc, char **argv) {

    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_HD720;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = UNIT_CENTIMETER;
    init_params.depth_minimum_distance = 30 ; // Set the minimum depth perception distance at 30cm

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
        return 1;

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD; // Use STANDARD sensing mode

    // Create sl and cv Mat to get ZED left image and depth image
    // Best way of sharing sl::Mat and cv::Mat :
    // Create a sl::Mat and then construct a cv::Mat using the ptr to sl::Mat data.
    Resolution image_size = zed.getResolution();
    sl::Mat image_zed(image_size,sl::MAT_TYPE_8U_C4); // Create a sl::Mat to handle Left image
	cv::Mat image_ocv = slMat2cvMat(image_zed);
	sl::Mat depth_image_zed(image_size, MAT_TYPE_8U_C4);
	cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);

    // Create OpenCV images to display (lower resolution to fit the screen)
    cv::Mat image_ocv_display(displaySize, CV_8UC4);
    cv::Mat depth_image_ocv_display(displaySize, CV_8UC4);

    // Mouse callback initialization
    mouseStruct.depth.alloc(image_size, MAT_TYPE_32F_C1);
    mouseStruct._resize = displaySize;

    // Give a name to OpenCV Windows
    cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Depth", onMouseCallback, (void*) &mouseStruct);

    // Jetson only. Execute the calling thread on 2nd core
    Camera::sticktoCPUCore(2);

    /******************** DARKNET-CPP API *****************************/
    // darknet-cpp api
    const bool DEBUG = true;
    box* boxes = 0;
    std::string * labels;
    ArapahoV2* p = new ArapahoV2();
    if(!p) {
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
    if(false == ret) {
        printf("Setup failed!\n");
        if(p) delete p;
        p = 0;
        return -1;
    }
    ArapahoV2ImageBuff arapahoImage;
    cv::Size imageSize(416, 416);
    cv::Mat image;
    int numObjects = 0;
    /******************** DARKNET-CPP API END **************************/

    // Create file and related time stamp
    std::ofstream file_depth;
    time_t last_write_time, now_time;
    time(&last_write_time);

    // Loop until 'q' is pressed
    char key = ' ';
    while (key != 'q') {

        // Grab and display image and depth
        if (zed.grab(runtime_parameters) == SUCCESS) {

            zed.retrieveImage(image_zed, VIEW_LEFT); // Retrieve the left image
            zed.retrieveImage(depth_image_zed, VIEW_DEPTH); //Retrieve the depth view (image)
            zed.retrieveMeasure(mouseStruct.depth, MEASURE_DEPTH); // Retrieve the depth measure (32bits)

            // Resize image with OpenCV
            cv::resize(image_ocv, image_ocv_display, displaySize);
            cv::resize(depth_image_ocv, depth_image_ocv_display, displaySize);

            /**************DARKNET API**************************/
            cv::resize(image_ocv, image, imageSize);
            arapahoImage.bgr = image.data;
            arapahoImage.w = image.size().width;
            arapahoImage.h = image.size().height;
            arapahoImage.channels = 4;
            // Detect the objects in the image
            p->Detect(arapahoImage, 0.24, 0.5, numObjects);
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
                  }
                p->GetBoxes(boxes, numObjects, labels);

                // get depth measure
                sl::Mat point_cloud;
                zed.retrieveMeasure(point_cloud,MEASURE_XYZRGBA);

                // file IO
                time(&now_time);
                if(now_time-last_write_time > 1) {
                  file_depth.open("/tmp/depth.txt");
                }

                // draw the bounding boxes and info
                cv::Scalar text_color(255,255,255);
                for(int i=0; i<numObjects; ++i){
                  if(DEBUG) {
                    std::cout << labels[i] << ',';
                    printf("Box #%d: x,y,w,h = [%f, %f, %f, %f]\n", i, boxes[i].x, boxes[i].y, boxes[i].w, boxes[i].h);
                    std::cout << std::endl;
                  }

                  normalizeBoxes(boxes[i]);
                  int left  = (boxes[i].x-boxes[i].w/2.)*displaySize.width;
                  int right = (boxes[i].x+boxes[i].w/2.)*displaySize.width;
                  int top   = (boxes[i].y-boxes[i].h/2.)*displaySize.height;
                  int bot   = (boxes[i].y+boxes[i].h/2.)*displaySize.height;
                  if(left < 0) left = 0;
                  if(right > displaySize.width) right = displaySize.width-1;
                  if(top < 0) top =0;
                  if(bot > displaySize.height) bot = displaySize.height-1;

                  // extract depth info
                  int center_x = point_cloud.getWidth() * (left+right)/(2.0*displaySize.width);
                  int center_y = point_cloud.getHeight() * (bot+top)/(2.0*displaySize.height);
                  sl::float4 point_depth;
                  if(DEBUG) {
                    std::cout << left << ' '<< right << ' '<< top << ' '<< bot << std::endl;
                    std::cout << center_x << ',' << center_y << std::endl;
                    }
                  point_cloud.getValue(center_x, center_y, &point_depth);
                  int x = point_depth.x;
                  int y = point_depth.y;
                  int z = point_depth.z;
                  float distance = sqrt(x*x + y*y + z*z); // Measure the distance

                  // draw
                  cv::Scalar rect_color;
                  mapcolor(distance, rect_color);
                  std::stringstream stream;
                  stream << std::fixed << std::setprecision(1) << distance;
                  std::string info = stream.str();
                  info += "cm";

                  cv::rectangle(image_ocv_display, cv::Point(left, bot), cv::Point(right, top), rect_color, 5);
                  cv::putText(image_ocv_display, labels[i]+","+info, cv::Point(left, top), 0, 1, text_color, 2, CV_AA);
                  // cv::putText(image_ocv_display, info, cv::Point(left, bot), 0, 1, text_color, 2, CV_AA);

                  cv::rectangle(depth_image_ocv_display, cv::Point(left, bot), cv::Point(right, top), rect_color, 5);
                  cv::putText(depth_image_ocv_display, labels[i]+","+info, cv::Point(left, top), 0, 1, text_color, 2, CV_AA);
                  // cv::putText(depth_image_ocv_display, info, cv::Point(left, bot), 0, 1, text_color, 2, CV_AA);

                  //write file
                  if(now_time-last_write_time > 1) {
                    if(distance >= 30.0)
                    file_depth << labels[i] << ',' << distance << std::endl;
                  }

                }

                // file clean up and reset
                if(now_time-last_write_time > 1) {
                  file_depth.close();
                  last_write_time = now_time;
                }

                //clean up
                delete[] boxes;
                delete[] labels;
              }
            /**************DARKNET API**************************/

            // Display with OpenCV
            imshow("Image", image_ocv_display);
            imshow("Depth", depth_image_ocv_display);

            key = cv::waitKey(10);
        }
    }

    zed.close();
    delete p;
    return 0;
}

static void onMouseCallback(int32_t event, int32_t x, int32_t y, int32_t flag, void * param) {
    if (event == CV_EVENT_LBUTTONDOWN) {
        mouseOCVStruct* data = (mouseOCVStruct*) param;
        int y_int = (y * data->depth.getHeight() / data->_resize.height);
        int x_int = (x * data->depth.getWidth() / data->_resize.width);

        sl::float1 dist;
        data->depth.getValue(x_int, y_int, &dist);

        std::cout << std::endl;
        if (isValidMeasure(dist))
            std::cout << "Depth at (" << x_int << "," << y_int << ") : " << dist << "m";
        else {
            std::string depth_status;
            if (dist == TOO_FAR) depth_status = ("Depth is too far.");
            else if (dist == TOO_CLOSE) depth_status = ("Depth is too close.");
            else depth_status = ("Depth not available");
            std::cout << depth_status;
        }
        std::cout << std::endl;
    }
}


cv::Mat slMat2cvMat(sl::Mat& input) {
	//convert MAT_TYPE to CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
		case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
		case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
		case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
		case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
		case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
		case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
		case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
		case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
		default: break;
	}

	// cv::Mat data requires a uchar* pointer. Therefore, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	//cv::Mat and sl::Mat will share the same memory pointer
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}


