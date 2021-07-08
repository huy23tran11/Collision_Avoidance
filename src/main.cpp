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
bool is_masked_img();
void space_finding();
void space_finding_by_region(cv::Mat &img_binary);
void fps_counter(int &frame_counter, int &final_time, int &initial_time);


int frame_w = 336;
int frame_h = 188;
cv::Mat img_result_templ_matching;
cv::Mat img_binary_formatted;
cv::Mat img_binary;
cv::Mat image_ocv_copy;

// color
cv::Scalar red = cv::Scalar(0, 0, 256);
cv::Scalar blue = cv::Scalar(256, 0, 0);
// cv::Point region_rect_tf;
// region_rect_tf.x = (frame_w / 2) - (84 / 2);
// region_rect_tf.y = (frame_h / 2) - (50 / 2);
int fps;

int main(int argc, char **argv) {
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
    runtime_parameters.sensing_mode = SENSING_MODE::FILL;

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
    while (true) {
        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, new_image_size);
            image_ocv_copy = image_ocv.clone();
            zed.retrieveImage(depth_image_zed_gpu, VIEW::DEPTH, MEM::GPU, new_image_size);
            depth_image_ocv_gpu.download(depth_image_ocv);

            // find matching template
            // img_binary = depth_image_ocv.clone();
            cv::threshold(depth_image_ocv, img_binary, 80, 255, cv::THRESH_BINARY); //convert depth img to binary 

            space_finding_by_region(img_binary);
            cv::imshow("Depth", img_binary);
            cv::imshow("Real Img", image_ocv_copy);

            // FPS counter:
            fps_counter(frame_counter, final_time, initial_time);
        }
    key = cv::waitKey(0);
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

// Function that save Gray Scale Image for testing
void saveGrayScaleImage(Camera& zed, std::string filename) {
    sl::Mat image_sbs;
    zed.retrieveImage(image_sbs, VIEW::DEPTH);

    auto state = image_sbs.write(filename.c_str());

    if (state == sl::ERROR_CODE::SUCCESS)
        std::cout << "Gray Scale image has been save under " << filename << std::endl;
    else
        std::cout << "Failed to save image... Please check that you have permissions to write at this location (" << filename << "). Re-run the sample with administrator rights under windows" << std::endl;
}

// this function will masked every 10% of the fps frame by a bitwise_or 
// then it will return true if it complete
// bool is_masked_img() {
//     if(frame_counter_for_space < (fps / 10)) {
//         frame_counter_for_space++;
//         cv::bitwise_or(img_binary_combined, img_binary_formatted, img_binary_combined);
//         return false;
//     }
//     else {
//         frame_counter_for_space = 0;
//         return true;
//     }
// }

// finding space, prioritize the center fisrt, if the center is blocked, looked for the nearest space around

// return true if space detected in the region, mode is from 0 to 3 which aera corespoinding to each area.
void space_finding_by_region(cv::Mat &img_binary) {
    cv::Point frame_center;
    cv::Point center_frame_tf;
    cv::Mat img_binary_formatted;
    cv::Point top_left;
    cv::Point bottom_right;
    vector<cv::Point> space_loc_tf;
    int templ_h = 50;
    int templ_w = 80;
    cv::Mat templ(templ_h, templ_w, CV_8U, cv::Scalar(0, 0, 0));
    double min_val, max_val;
    cv:: Point min_loc, max_loc;
    cv::Mat black_frame(frame_h, frame_w, CV_8U, cv::Scalar(0, 0, 0));
    cv::Point center_rect_tf;
    cv::Point templ_top_left_adjustment_from_center;

    templ_top_left_adjustment_from_center.x = templ_w;
    templ_top_left_adjustment_from_center.y = templ_h;
    frame_center.x = frame_w / 2;
    frame_center.y = frame_h / 2;
    center_rect_tf.x = (frame_w / 2) - (templ_w / 2);
    center_rect_tf.y = (frame_h / 2) - (templ_h / 2);

    cv::cvtColor(img_binary, img_binary_formatted, cv::COLOR_BGR2GRAY);
    cv::Point br_adjustment;
    br_adjustment.x = templ_w;
    br_adjustment.y = templ_h;
    cv::matchTemplate(img_binary_formatted, templ, img_result_templ_matching, cv::TM_SQDIFF);
    cv::minMaxLoc(img_result_templ_matching, &min_val, &max_val, &min_loc, &max_loc);
    // cout << img_result_templ_matching << endl;
    for(int y_value = 0; y_value < img_result_templ_matching.rows; y_value++) {
        for(int x_value = 0; x_value < img_result_templ_matching.cols; x_value++) {
            if( img_result_templ_matching.at<float>(y_value, x_value) == 0) {
                space_loc_tf.push_back( cv::Point(x_value, y_value));
                // cout << img_result_templ_matching.at<double>(y_value, x_value) << endl;
            }
        }
    }
    if(space_loc_tf.empty()) {
        cout << "No space!!!" << endl;
    }
    else {
        double dist[space_loc_tf.size()];
        for(int index = 0; index < space_loc_tf.size(); index++) {
            // dist[index] = sqrt((space_loc_tf.at(index).x) * (space_loc_tf.at(index).x) + (space_loc_tf.at(index).y) * (space_loc_tf.at(index).x)); 
            dist[index] = sqrt((space_loc_tf.at(index).x - center_frame_tf.x) * (space_loc_tf.at(index).x - center_frame_tf.x) + (space_loc_tf.at(index).y - center_frame_tf.y) * (space_loc_tf.at(index).y - center_frame_tf.y));
        }
        cout << center_rect_tf  << endl;

        double min_dist = dist[0];
        int min_dist_index = 0;
        for(int i = 0; i < space_loc_tf.size(); i++) {
            if(dist[i] < min_dist) {
                min_dist = dist[i];
                min_dist_index = i;
            }
        }
        // cout << min_dist << endl;

        top_left = space_loc_tf.at(min_dist_index);
        bottom_right = top_left + templ_top_left_adjustment_from_center;
        cv::rectangle(img_binary, top_left, bottom_right, blue, 2);
        cv::rectangle(image_ocv_copy, top_left, bottom_right, blue, 2);

    }
}

// fps counter show on screen
void fps_counter(int &frame_counter, int &final_time, int &initial_time) {
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