///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
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

/***********************************************************************************************
 ** This sample demonstrates how to use the ZED SDK with OpenCV.                              **
 ** Depth and images are captured with the ZED SDK, converted to OpenCV format and displayed. **
 ***********************************************************************************************/
#include <iostream>
// time for fps
#include <time.h>
 // ZED includes
#include <sl/Camera.hpp>
// OpenCV includes
#include <opencv2/opencv.hpp>
// OpenCV dep
#include <opencv2/cvconfig.h>

using namespace std;
using namespace sl;

cv::Mat slMat2cvMat(Mat& input);
cv::cuda::GpuMat slMat2cvMatGPU(Mat& input);

void saveGrayScaleImage(Camera& zed, std::string filename);
static void empty( int, void* );

int main(int argc, char **argv) {
    // imtermediate img
    cv::Mat img_blur;
    cv::Mat img_canny;
    cv::Mat img_binary;
    cv::Mat img_dil;
    cv::Mat approx;
    cv::Mat image_zed_gray_scale;
    cv::Mat img_result_templ_matching;
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    //setting up template for opening space
    cv::Mat temp(50, 80, CV_8U, cv::Scalar(0, 0, 0));
    double min_val, max_val;
    cv:: Point min_loc, max_loc;
    // color
    cv::Scalar red = cv::Scalar(0, 0, 256);
    cv::Scalar blue = cv::Scalar(256, 0, 0);

    //safe zone for flighting
    cv::Point safe_zone;
    safe_zone.x = 80;
    safe_zone.y = 50;

    int count_save = 0;
    // Create a ZED camera object
    Camera zed;
    // set up fps counter 
    int initial_time = 0;
    int final_time = 0;
    int frame_counter = 0;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::VGA;
    init_params.depth_mode = DEPTH_MODE::PERFORMANCE;
    init_params.coordinate_units = UNIT::METER;
    init_params.camera_fps = 100;
    if (argc > 1) init_params.input.setFromSVOFile(argv[1]);

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        printf("%s\n", toString(err).c_str());
        zed.close();
        return 1; // Quit if an error occurred
    }


    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getCameraInformation().camera_resolution;
    int new_width = image_size.width / 2;
    int new_height = image_size.height / 2;

    Resolution new_image_size(new_width, new_height);

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    Mat image_zed(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);
    Mat depth_image_zed_gpu(new_width, new_height, MAT_TYPE::U8_C4, sl::MEM::GPU); // alloc sl::Mat to store GPU depth image
    cv::cuda::GpuMat depth_image_ocv_gpu = slMat2cvMatGPU(depth_image_zed_gpu); // create an opencv GPU reference of the sl::Mat
    cv::Mat depth_image_ocv; // cpu opencv mat for display purposes
    Mat point_cloud;
    Mat confidence_map; // confidence filtering
    cv::Mat confidence_map_ocv = slMat2cvMat(confidence_map);
    // Loop until 'q' is pressed
    char key = ' ';
    int frame_counter_for_space = 0;
    cv::Mat black_frame(188, 336, CV_8U, cv::Scalar(0, 0, 0));
    cv::Mat img_binary_combined = black_frame.clone();
    cv::Point top_left;
    cv::Point bottom_right;
    cv::Point center_rect_tf;
    center_rect_tf.x = (336 / 2) - (80 / 2);
    center_rect_tf.y = (188 / 2) - (50 / 2);
    cv::Point region_rect_tf;
    region_rect_tf.x = (336 / 2) - (84 / 2);
    region_rect_tf.y = (188 / 2) - (50 / 2);
    int fps;
    while (true) {
        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, new_image_size);
            cv::cvtColor(image_ocv, image_zed_gray_scale, cv::COLOR_BGR2GRAY);

            zed.retrieveImage(depth_image_zed_gpu, VIEW::DEPTH, MEM::GPU, new_image_size);

            // Edge Detection:
            depth_image_ocv_gpu.download(depth_image_ocv);
            // cv::GaussianBlur(image_zed_gray_scale, img_blur, cv::Size(5, 5), 1);
            cv::threshold(depth_image_ocv, img_binary, 100, 255, cv::THRESH_BINARY);
            // cv::Canny(img_blur, img_canny, threshold1, threshold2);
            // cv::Canny(img_binary, img_canny, threshold1, threshold2);
            // find matching template
            cv::Mat gray;
            cv::cvtColor(img_binary, gray, cv::COLOR_BGR2GRAY);
            cv::Mat cropped_center = img_binary_combined(cv::Rect(center_rect_tf.x, center_rect_tf.y, 80, 50));
            frame_counter_for_space++;
            if(frame_counter_for_space < (fps / 10)) {
                cv::bitwise_or(img_binary_combined, gray, img_binary_combined);
            }
            else {
                frame_counter_for_space = 0;
                cv::matchTemplate(cropped_center, temp, img_result_templ_matching, cv::TM_SQDIFF);
                if(cv::countNonZero(img_result_templ_matching) < 1){
                    // cv::rectangle(img_binary, cv::Rect(center_rect_tf.x, center_rect_tf.y, 80, 50), blue, 2);
                    cv::minMaxLoc(img_result_templ_matching, &min_val, &max_val, &min_loc, &max_loc);
                    top_left = center_rect_tf + min_loc;
                    bottom_right = top_left + safe_zone;
                    cv::rectangle(img_binary, top_left, bottom_right, blue, 2);
                }
                else {
                        cv::matchTemplate(gray, temp, img_result_templ_matching, cv::TM_SQDIFF);
                        cv::minMaxLoc(img_result_templ_matching, &min_val, &max_val, &min_loc, &max_loc);
                        top_left = min_loc;
                        bottom_right = top_left + safe_zone;
                    }
                    // cv::matchTemplate(img_binary_combined, temp, img_result_templ_matching, cv::TM_SQDIFF);
                    // blank the img for another comnination of img.
                    cv::bitwise_and(img_binary_combined, black_frame, img_binary_combined);
                }
            cv::rectangle(img_binary, top_left, bottom_right, red, 2);
            cv::imshow("Depth", img_binary);
            // img_binary_combined.empty();
            // cv::imshow("Real Img", img_binary_combined);
            // cv::imshow("Canny", img_canny);
            // cv::imshow("Binary", img_binary);

            // FPS counter:
            frame_counter++;
            final_time = time(NULL);
            if(final_time - initial_time > 0) {
                fps = frame_counter / (final_time - initial_time);
                cout << "FPS: " << fps <<endl;
                // cout << img_binary.size << endl;
                // cout << "Obstacle count: " << drawed_contours_count << endl;
                cout << endl;
                // drawed_contours_count = 0;
                frame_counter = 0;
                initial_time = final_time;
            }
    }
    key = cv::waitKey(1);
    if (key == 'q') {break;}
}

    // sl::Mat GPU memory needs to be free before the zed
    depth_image_zed_gpu.free();
    zed.close();
    return 0;
}

// Mapping between MAT_TYPE and CV_TYPE
int getOCVtype(sl::MAT_TYPE type) {
    int cv_type = -1;
    switch (type) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv_type;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input) {
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::cuda::GpuMat slMat2cvMatGPU(Mat& input) {
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::cuda::GpuMat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(MEM::GPU), input.getStepBytes(sl::MEM::GPU));
}

// Function that save Gray Scale Image
void saveGrayScaleImage(Camera& zed, std::string filename) {
    sl::Mat image_sbs;
    zed.retrieveImage(image_sbs, VIEW::DEPTH);

    auto state = image_sbs.write(filename.c_str());

    if (state == sl::ERROR_CODE::SUCCESS)
        std::cout << "Gray Scale image has been save under " << filename << std::endl;
    else
        std::cout << "Failed to save image... Please check that you have permissions to write at this location (" << filename << "). Re-run the sample with administrator rights under windows" << std::endl;
}

//Function that do nothing when trackbars change
static void empty( int, void* ){

}