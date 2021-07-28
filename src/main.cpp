/***********************************************************************************************
 ** This sample collision avoidance algorithm using the ZED SDK with OpenCV.                 **
 ** Depth and images are captured with the ZED SDK, processed by OpenCV and manuver drones.  **
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
// python interfaces
#include <python3.6m/Python.h>
// time for video and img
#include <ctime>

using namespace std;
using namespace sl;

cv::Mat slMat2cvMat(Mat& input);
cv::cuda::GpuMat slMat2cvMatGPU(Mat& input);
bool is_masked_img();// this function will masked every 10% of the fps frame by a bitwise_or then it will return true if it complete used to filter out noise, NOT USED
bool finding_best_space(cv::Mat &img_binary, cv::Rect &templ_rect, cv::Rect &center_Rect); // find best space that closet to the center then return true, return false if there is no space at all
void finding_all_avaiable_space(cv::Mat img_result_templ_matching, vector<cv::Point> &space_loc_tf); // find all avaible space in a vector called in the finding_best_space() func
void fps_counter(int &fps, int &frame_counter, int &final_time, int &initial_time);// fps counter show on screen
int finding_closest_space_to_center(vector<cv::Point> &space_loc_tf, cv::Point center_frame_tf); //from all avaible space, find the closet space to the center then return the index of that space rectangle in the vector
bool check_rect_matched(cv::Rect templ_rect, cv::Rect center_rect); // check if the center rectangle and the best rectangle without obstacle match with 5 pixel error due to noise
void manuver(bool is_space, bool is_matched, cv::Rect &templ_rect, cv::Rect &center_rect, bool &is_moving_to_target); //calling python function to manuver the plane
void put_text(cv::Mat img, bool is_space, bool is_matched, cv::Rect &templ_rect, cv::Rect &center_rect); // put guidance text on img


// Python interpreter
PyObject *pName, *pMod, *pDict, *result;
PyObject *pFunc, *pArgs, *pVal;
int main(int argc, char **argv) {
    // Python interpreter
	Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('/home/nvidia/Visual_Code_Workspace/Collision_Avoidance/src')"); // path of the PyMovement.py
    pName = PyUnicode_FromString("PyMovement");
    pMod = PyImport_Import(pName);
    if (pMod == NULL) {
        PyErr_Print();
        std::exit(1);
    }
    pDict = PyModule_GetDict(pMod);
    pArgs = NULL;

    //Connection
    pFunc = PyDict_GetItemString(pDict, "connectionFunc");  //PyObject to call the connection function
    PyObject_CallObject(pFunc, pArgs);  //Call the connection function from the Python Script

    // set Home Location
    pFunc = PyDict_GetItemString(pDict, "setLocation");  //PyObject to call the setLocation function
    PyObject_CallObject(pFunc, pArgs);  //Call the function from the Python Script

	//Arm_and_Takeoff
    pFunc = PyDict_GetItemString(pDict, "arm_and_takeoff");  //PyObject to call the setLocation function
    PyObject_CallObject(pFunc, pArgs);  //Call the function from the Python Script

    //set Tartget location
    pFunc = PyDict_GetItemString(pDict, "setTargetLoc");  //PyObject to call the setTargetLoc function
    PyObject_CallObject(pFunc, pArgs);  //Call the function from the Python Script

    //go to Tartget location
    // pFunc = PyDict_GetItemString(pDict, "goToTargetLoc");  //PyObject to call the goToTargetLoc function
    // PyObject_CallObject(pFunc, pArgs);  //Call the function from the Python Script
    bool is_moving_to_target = false; // moving flag, if call goToTargetLoc before camera is on turn this to true


    // imtermediate img and point
    cv::Rect templ_rect;
    cv::Rect center_rect;
    cv::Mat img_binary; // convert to binary to easily process

    // color
    cv::Scalar red = cv::Scalar(0, 0, 256);
    cv::Scalar blue = cv::Scalar(256, 0, 0);

    bool is_matched;
    bool is_space;

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
    init_params.depth_minimum_distance = 4;
    init_params.depth_maximum_distance = 5;
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
    int new_width = image_size.width; // used to be image_size.width / 2, I dont delete this line since lots of place use new_width
    int new_height = image_size.height; // same reason

    Resolution new_image_size(new_width, new_height);

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    Mat image_zed(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);
    Mat depth_image_zed_gpu(new_width, new_height, MAT_TYPE::U8_C4, sl::MEM::GPU); // alloc sl::Mat to store GPU depth image
    cv::cuda::GpuMat depth_image_ocv_gpu = slMat2cvMatGPU(depth_image_zed_gpu); // create an opencv GPU reference of the sl::Mat
    cv::Mat depth_image_ocv; // cpu opencv mat for display purposes
    Mat point_cloud;
    int fps = 0;
    // for saving img
    string final_real_img;
    string final_binary_img;

    //video capture
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo);
    std::string str(buffer);
    cv::VideoWriter video_real("/home/nvidia/Desktop/video_real/real " + str + ".avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(new_width, new_height)); // fps 30 => maybe it moves fast
    cv::VideoWriter video_binary("/home/nvidia/Desktop/video_binary/binary " + str + ".avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(new_width, new_height)); // fps 30 => maybe it moves fast
    cv::Mat img_real_for_video;
    cv::Mat img_binary_for_video;
    
    // Loop until 'q' is pressed
    char key = ' ';
    while (true) {
        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, new_image_size);
            zed.retrieveImage(depth_image_zed_gpu, VIEW::DEPTH, MEM::GPU, new_image_size);
            depth_image_ocv_gpu.download(depth_image_ocv);
            cv::threshold(depth_image_ocv, img_binary, 5, 255, cv::THRESH_BINARY); //convert depth img to binary

            is_space = finding_best_space(img_binary, templ_rect, center_rect);

            cv::rectangle(image_ocv, templ_rect.tl(), templ_rect.br(), blue, 2); // real img
            cv::rectangle(image_ocv, center_rect.tl(), center_rect.br(), red, 2); // real img
            cv::rectangle(img_binary, templ_rect.tl(), templ_rect.br(), blue, 2); // binary img 
            cv::rectangle(img_binary, center_rect.tl(), center_rect.br(), red, 2); // binary img

            is_matched = check_rect_matched(templ_rect, center_rect);

            put_text(img_binary, is_space, is_matched, templ_rect, center_rect);
            put_text(image_ocv, is_space, is_matched, templ_rect, center_rect);

            manuver(is_space, is_matched, templ_rect, center_rect, is_moving_to_target); // guidance on binary img

            // save img
            // time (&rawtime);
            // timeinfo = localtime(&rawtime);
            // strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo);
            // std::string str(buffer);
            // final_real_img = "/home/nvidia/Desktop/img_real/real_img_" + str + ".jpg";
            // final_binary_img = "/home/nvidia/Desktop/img_binary/binary_img_" + str + ".jpg";
            // if(frame_counter == 0) { // take pics 1 pic / second, depends on frame rate if frame rate is reseted means 1s
            //     cv::imwrite(final_real_img, image_ocv);
            //     cv::imwrite(final_binary_img, img_binary);
            // }

            // FPS counter:
            fps_counter(fps, frame_counter, final_time, initial_time);
            string fps_str = "FPS: " + std::to_string(fps);
            cv::putText(image_ocv, fps_str, cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1, red, 1);
            cv::putText(img_binary, fps_str, cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1, red, 1);

            //show img
            cv::imshow("Real", image_ocv);
            cv::imshow("Depth", img_binary);

            // save video
            // cv::cvtColor(image_ocv, img_real_for_video, cv::COLOR_RGB2BGR);
            // cv::cvtColor(img_binary, img_binary_for_video, cv::COLOR_RGB2BGR);
            // video_real.write(img_real_for_video);
            // video_binary.write(img_binary_for_video);
        }
    key = cv::waitKey(1);
    if (key == 'q') {break;}
    }

    // sl::Mat GPU memory needs to be free before the zed
    video_binary.release();
    video_real.release();
    depth_image_zed_gpu.free();
    zed.close();
    Py_Finalize();
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


bool finding_best_space(cv::Mat &img_binary, cv::Rect &templ_rect, cv::Rect &center_Rect) {
    cv::Mat img_bianry_cropped; // cropped img to only detect horizontal regions;
    int frame_w = img_binary.cols;
    int frame_h = img_binary.rows;
    int templ_w = 200; // dimension of the drone at 5 m with 50% bigger 133 / 100 * 150, recommend 300 for safer avoiding
    int templ_h = 83; // dimension of the drone at 5 m with 50% bigger 55 / 100 * 150
    cv::Mat img_result_templ_matching; // output of matchTemplate function, this is NOT an img, it's an matrix that store the matching result for every pixel, the smaller the number in this matrix the more match
    cv::Mat img_binary_formatted; // binary img need to be formatted to be used for template matching algothsm
    cv::Mat templ(templ_h, templ_w, CV_8U, cv::Scalar(0, 0, 0)); // create a template that all black
    cv::Point center_frame_tf; // the top left point of the frame in the center
    center_frame_tf.x = (frame_w / 2) - (templ_w / 2);
    center_frame_tf.y = (frame_h / 2) - (templ_h / 2);
    center_Rect = cv::Rect(center_frame_tf.x, center_frame_tf.y, templ_w, templ_h);
    cv::Point cropped_frame_tf;
    vector<cv::Point> space_loc_tf;

    cv::cvtColor(img_binary, img_binary_formatted, cv::COLOR_BGR2GRAY); // format binary img for matchTemplate function
    img_bianry_cropped = img_binary_formatted(cv::Rect(0, center_frame_tf.y, frame_w, templ_h)); // create a frame fit the templ, so that it can only slide left and right
    cropped_frame_tf.x = 0;
    cropped_frame_tf.y = center_frame_tf.y;
    // cv::matchTemplate(img_binary_formatted, templ, img_result_templ_matching, cv::TM_SQDIFF);
    cv::matchTemplate(img_bianry_cropped, templ, img_result_templ_matching, cv::TM_SQDIFF);
    // cv::minMaxLoc(img_result_templ_matching, &min_val, &max_val, &min_loc, &max_loc);
    // cout << img_result_templ_matching << endl;

    finding_all_avaiable_space(img_result_templ_matching, space_loc_tf);

    if(space_loc_tf.empty()) {
        return false;
    }
    else {

        int min_dist_index = finding_closest_space_to_center(space_loc_tf, center_frame_tf);
        // top_left =  space_loc_tf.at(min_dist_index); // for main frame
        cv::Point top_left =  cropped_frame_tf + space_loc_tf.at(min_dist_index); //for cropped frame
        templ_rect = cv::Rect(top_left.x, top_left.y, templ_w, templ_h);
        return true;
    }
}

void finding_all_avaiable_space(cv::Mat img_result_templ_matching, vector<cv::Point> &space_loc_tf){
    for(int y_value = 0; y_value < img_result_templ_matching.rows; y_value++) {
        for(int x_value = 0; x_value < img_result_templ_matching.cols; x_value++) {
            if( img_result_templ_matching.at<float>(y_value, x_value) == 0) {
                space_loc_tf.push_back( cv::Point(x_value, y_value));
                // cout << img_result_templ_matching.at<double>(y_value, x_value) << endl;
            }
        }
    }
}

int finding_closest_space_to_center(vector<cv::Point> &space_loc_tf, cv::Point center_frame_tf) {
    double dist[space_loc_tf.size()];
    for(int index = 0; index < space_loc_tf.size(); index++) {
        dist[index] = sqrt((space_loc_tf.at(index).x - center_frame_tf.x) * (space_loc_tf.at(index).x - center_frame_tf.x) + (space_loc_tf.at(index).y - center_frame_tf.y) * (space_loc_tf.at(index).y - center_frame_tf.y));
        // cout << dist[index] << endl;
    }
    double min_dist = dist[0];
    int min_dist_index = 0;
    for(int i = 0; i < space_loc_tf.size(); i++) {
        if(dist[i] < min_dist) {
            min_dist = dist[i];
            min_dist_index = i;
        }
    }
    return min_dist_index;
}


void fps_counter(int &fps, int &frame_counter, int &final_time, int &initial_time) {
    frame_counter++;
    final_time = time(NULL);
    if(final_time - initial_time > 0) {
        fps = frame_counter / (final_time - initial_time);
        cout << "FPS: " << fps <<endl;
        cout << endl;
        frame_counter = 0;
        initial_time = final_time;
    }
}

void manuver(bool is_space, bool is_matched, cv::Rect &templ_rect, cv::Rect &center_rect, bool &is_moving_to_target) {
    cv::Scalar red = cv::Scalar(0, 0, 256);
    cv::Scalar blue = cv::Scalar(256, 0, 0);
    if(!is_space) {
        if(is_moving_to_target) {
            pFunc = PyDict_GetItemString(pDict, "stop");  //PyObject to call the connection function
            PyObject_CallObject(pFunc, pArgs);  //Call the function from the Python Script
            is_moving_to_target = false;
        }
    }
    else if(is_matched) {
        if(!is_moving_to_target) {
            pFunc = PyDict_GetItemString(pDict, "goToTargetLoc");  //PyObject to call the connection function
            PyObject_CallObject(pFunc, pArgs);  //Call the function from the Python Script
            is_moving_to_target = true;
        }
    } else if(!is_matched) {
        if(templ_rect.x - center_rect.x > 0) {
            pFunc = PyDict_GetItemString(pDict, "slide_right");  //PyObject to call the connection function
            PyObject_CallObject(pFunc, pArgs);  //Call the function from the Python Script
            is_moving_to_target = false;
        } else {
            pFunc = PyDict_GetItemString(pDict, "slide_left");  //PyObject to call the connection function
            PyObject_CallObject(pFunc, pArgs);  //Call the function from the Python Script
            is_moving_to_target = false;
        }
    } else {
        if(is_moving_to_target) {
            pFunc = PyDict_GetItemString(pDict, "stop");  //PyObject to call the connection function
            PyObject_CallObject(pFunc, pArgs);  //Call the function from the Python Script
            is_moving_to_target = false;
        }
    }
}

void put_text(cv::Mat img, bool is_space, bool is_matched, cv::Rect &templ_rect, cv::Rect &center_rect) {
    cv::Scalar red = cv::Scalar(0, 0, 256);
    cv::Scalar blue = cv::Scalar(256, 0, 0);
    if(!is_space) {
        cv::putText(img, "STOP", cv::Point(200, 50), cv::FONT_HERSHEY_PLAIN, 4, red, 3);
        cv::putText(img, "Obstacle Detected", cv::Point(50, 300), cv::FONT_HERSHEY_PLAIN, 4, red, 3);
    }
    else if(is_matched) {
        cv::putText(img, "go to location", cv::Point(100, 50), cv::FONT_HERSHEY_PLAIN, 4, blue, 3);
    } else if(!is_matched) {
        if(templ_rect.x - center_rect.x > 0) {
            cv::putText(img, "slide right", cv::Point(200, 50), cv::FONT_HERSHEY_PLAIN, 4, blue, 3);
            cv::putText(img, "Obstacle Detected", cv::Point(50, 300), cv::FONT_HERSHEY_PLAIN, 4, red, 3);
        } else {
            cv::putText(img, "slide left", cv::Point(200, 50), cv::FONT_HERSHEY_PLAIN, 4, blue, 3);
            cv::putText(img, "Obstacle Detected", cv::Point(50, 300), cv::FONT_HERSHEY_PLAIN, 4, red, 3);
        }
    } else {
        cv::putText(img, "STOP, cannot figure what to do", cv::Point(200, 50), cv::FONT_HERSHEY_PLAIN, 4, red, 3);
    }
}

bool check_rect_matched(cv::Rect templ_rect, cv::Rect center_rect) {
    int error = 5;
    return (templ_rect.tl().x < center_rect.tl().x + error && templ_rect.tl().x > center_rect.tl().x - error);
}


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
