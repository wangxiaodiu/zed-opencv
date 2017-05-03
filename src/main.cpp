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

// some debug toggle
const bool image_save_toggle = true;
const bool DEBUG = false;

// togle depth image show and calculation
bool depth_toggle = true;

cv::Size displaySize(720*4, 404*4);
void normalizeBoxes(box& box)
{
  float &x = box.x; float &y = box.y; float &w = box.w; float &h = box.h; 
  if(x>1) x = 1; if(y>1) y = 1; if(w>1) w = 1; if(h>1) h = 1; 
  if(x<0) x = 0; if(y<0) y = 0; if(w<0) w = 0; if(h<0) h = 0;
}

void drawintendedpath(cv::Mat &img)
{
  cv::Scalar path_color(0,255,0); // pure green
  int thickness = 5;
  int w = img.size().width;
  int h = img.size().height;

  cv::Point center(w/2-1, h/2-1);

  int x_l = 1000;
  cv::line(img, cv::Point(x_l, h-1), center, path_color, thickness);

  int x_r = 1880;
  cv::line(img, cv::Point(x_r, h-1), center, path_color, thickness);
}

void drawgrids(cv::Mat & img, int row, int col)
{
  cv::Scalar grid_color(255,255,255);
  int thickness = 5;
  int w = img.size().width; int w_gap = w/col;
  int h = img.size().height; int h_gap = h/row;
  for(int x=1; x<col; ++x){
    cv::line(img, cv::Point(float(x)*w/col,0), cv::Point(float(x)*w/col, h-1), grid_color, thickness);
  }
  for(int y=1; y<row; ++y){
    cv::line(img, cv::Point(0,float(y)*h/row), cv::Point(w-1,float(y)*h/row), grid_color, thickness);
  }
}

void mapcolor(float dist, cv::Scalar & color)
{
  if(dist < 2)
    color = cv::Scalar(0,0,0);
  else if(dist < 4.0)
    color = cv::Scalar(0,0,255); // pure red
  else if(dist < 6.0)
    color = cv::Scalar(80,127,255); // orange
  else if(dist < 8.0)
    color = cv::Scalar(0,255,255); //yellow
  else if(dist < 10.0)
    color = cv::Scalar(0,255,0); // green
  else if(dist < 12.0)
    color = cv::Scalar(255,0,0); // blue
  else
    color = cv::Scalar(128,0,128); // PURPLE
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
    init_params.camera_resolution = RESOLUTION_HD2K;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = UNIT_FOOT;
    init_params.depth_minimum_distance = 2 ; // Set the minimum depth perception distance at 2 feet

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
        return 1;

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_FILL; // Use STANDARD sensing mode

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
    // cv::Mat depth_ocv = slMat2cvMat(mouseStruct.depth);

    // Give a name to OpenCV Windows
    cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Depth", onMouseCallback, (void*) &mouseStruct);

    // Jetson only. Execute the calling thread on 2nd core
    Camera::sticktoCPUCore(2);

    /******************** DARKNET-CPP API *****************************/
    // darknet-cpp api
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
    time_t last_write_time, now_time, fps_now, fps_last;
    time(&last_write_time);
    time(&fps_last);
    fps_now = fps_last;
    int fps_cnt = 0;

    // Loop until 'q' is pressed
    int img_cnt=0;
    char key = ' ';
    while (key != 'q') {

      // toggle depth
      if(key == 'd'){
        if(!depth_toggle)
        {
          cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
          cv::setMouseCallback("Depth", onMouseCallback, (void*) &mouseStruct);
        }
        depth_toggle = true;
      }
      if(key == 'x'){
        if(depth_toggle) {
          cvDestroyWindow("Depth");
        }
        depth_toggle = false;
      }
        // Grab and display image and depth
        if (zed.grab(runtime_parameters) == SUCCESS) {

            zed.retrieveImage(image_zed, VIEW_LEFT); // Retrieve the left image
            if(depth_toggle){
              zed.retrieveImage(depth_image_zed, VIEW_DEPTH); //Retrieve the depth view (image)
              zed.retrieveMeasure(mouseStruct.depth, MEASURE_DEPTH); // Retrieve the depth measure (32bits)
            }

            // Resize image with OpenCV
            cv::resize(image_ocv, image_ocv_display, displaySize);
            if(depth_toggle){
              cv::resize(depth_image_ocv, depth_image_ocv_display, displaySize);
            }

            // draw 3*5 grid on image
            drawgrids(image_ocv_display, 3, 5);
            drawintendedpath(image_ocv_display);

            /**************DARKNET API**************************/
            cv::resize(image_ocv, image, imageSize);
            arapahoImage.bgr = image.data;
            arapahoImage.w = image.size().width;
            arapahoImage.h = image.size().height;
            arapahoImage.channels = 4;
            p->Detect(arapahoImage, 0.24, 0.5, numObjects); // Detect the objects in the image
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
                  // int pc_h = point_cloud.getHeight();
                  // int pc_w = point_cloud.getWidth();
                  int pc_h = mouseStruct.depth.getHeight();
                  int pc_w = mouseStruct.depth.getWidth();
                  int center_x = pc_w * (left+right)/(2.0*displaySize.width);
                  int center_y = pc_h * (bot+top)/(2.0*displaySize.height);
                  sl::float4 point_depth;
                  if(DEBUG) {
                    // std::cout << pc_h << ',' << pc_w << std::endl;
                    std::cout << left << ' '<< right << ' '<< top << ' '<< bot << std::endl;
                    std::cout << center_x << ',' << center_y << std::endl;
                    }

                  // Measure the distance
                  double distance = 0;
                  float z;
                  int cnt = 0;
                  for(int xi = -1; xi<=1; ++xi){
                    if(center_x + xi < pc_w && center_x + xi > 0)
                      for(int yi = -1; yi <= 1; ++yi){
                        if(center_y + yi < pc_h && center_y + yi >0){
                          //point_cloud.getValue(center_x+xi, center_y+yi, &point_depth);
                          // float &z = point_depth.z;
                          mouseStruct.depth.getValue(center_x+xi, center_y+yi, &z);
                          if(z >= 2.0 && z <= 65){
                            if(DEBUG){
                              std::cout << z << distance << std::endl;
                            }
                            distance += z;
                            cnt++;
                          }
                        }
                      }
                  }
                  if(cnt) distance /= cnt;
                  else distance = 0;
                  if(DEBUG) {
                    std::cout << distance << "feet" << std::endl;
                  }


                  // draw
                  if(distance >=2.0) {
                    cv::Scalar rect_color;
                    mapcolor(distance, rect_color);
                    std::stringstream stream;
                    stream << std::fixed << std::setprecision(1) << distance;
                    std::string info = stream.str();
                    info += "feet";

                    cv::rectangle(image_ocv_display, cv::Point(left, bot), cv::Point(right, top), rect_color, 5);
                    cv::putText(image_ocv_display, labels[i]+","+info, cv::Point(left, top), 0, 1, text_color, 2, CV_AA);

                    if(depth_toggle){
                      cv::rectangle(depth_image_ocv_display, cv::Point(left, bot), cv::Point(right, top), rect_color, 5);
                      cv::putText(depth_image_ocv_display, labels[i]+","+info, cv::Point(left, top), 0, 1, text_color, 2, CV_AA);
                    }
                  }

                  //write file
                  if(now_time-last_write_time > 1) {
                    if(distance >= 2.0)
                    file_depth << labels[i] << ',' << distance << std::endl;
                  }
                }

                // file clean up and reset
                if(now_time-last_write_time > 1) {
                  file_depth.close();
                  last_write_time = now_time;
                  if(image_save_toggle)
                    {
                      std::string img_path="./";
                      img_path += std::to_string(++img_cnt);
                      imwrite(img_path+".jpg", image_ocv_display);
                      if(depth_toggle){
                        imwrite(img_path+".depth.jpg", depth_image_ocv_display);
                      }
                    }
                }

                //clean up
                delete[] boxes;
                delete[] labels;
              }
            /**************DARKNET API**************************/

            // Display with OpenCV
            imshow("Image", image_ocv_display);
            if(depth_toggle){
              imshow("Depth", depth_image_ocv_display);
            }

            key = cv::waitKey(10);
        }

        //measure time
        time(&fps_now);
        if(fps_now>fps_last){
          if(fps_now-fps_last==1){
            std::cout << "FPS:" << fps_cnt << std::endl;
          }
          fps_cnt = 0;
          fps_last = fps_now;
        } else{
          ++fps_cnt;
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


