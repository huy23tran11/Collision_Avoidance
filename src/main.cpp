/***********************************************************************************************
 ** This sample collision avoidance algothmism how to use the ZED SDK with OpenCV.                              **
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
// #include <python3.6m/Python.h>

using namespace std;
using namespace sl;

cv::Mat slMat2cvMat(Mat& input);
cv::cuda::GpuMat slMat2cvMatGPU(Mat& input);
void saveGrayScaleImage(Camera& zed, std::string filename);
bool is_masked_img();
bool finding_best_space(cv::Mat &img_binary, cv::Rect &templ_rect, cv::Rect &center_Rect);
void finding_all_avaiable_space(cv::Mat img_result_templ_matching, vector<cv::Point> &space_loc_tf);
int fps_counter(int &frame_counter, int &final_time, int &initial_time);// fps counter show on screen
int finding_closest_space_to_center(vector<cv::Point> &space_loc_tf, cv::Point center_frame_tf);
bool check_rect_matched(cv::Rect templ_rect, cv::Rect center_rect);
void manuver(cv::Mat img, bool is_space, bool is_matched, cv::Rect &templ_rect, cv::Rect &center_rect);


//global variables setting up template for opening space

int main(int argc, char **argv) {
    // Python interpreter
	// PyObject *pName, *pMod, *pDict;
	// Py_Initialize();
    // PyRun_SimpleString("import sys");
    // PyRun_SimpleString("sys.path.append('/home/nguyen/Visual-Studio-Workspace/Collision_Avoidance/src')");
    // pName = PyUnicode_FromString("PyMovement");
    // pMod = PyImport_Import(pName);
    // if (pMod == NULL) {
    //     PyErr_Print();
    //     std::exit(1);
    // }
    // pDict = PyModule_GetDict(pMod);
    // PyObject *pFunc, *pArgs, *pVal;
    // pArgs = NULL;

    // //Connection
    // pFunc = PyDict_GetItemString(pDict, "connectionFunc");  //PyObject to call the connection function
    // PyObject_CallObject(pFunc, pArgs);  //Call the connection function from the Python Script

	// //Arm
	// pFunc = PyDict_GetItemString(pDict, "arm");  //PyObject to call the connection function
    // PyObject_CallObject(pFunc, pArgs);  //Call the connection function from the Python Script
	// // cin.ignore();

	// //Takeoff
	// pFunc = PyDict_GetItemString(pDict, "takeoff");
	// pArgs = PyTuple_New(1);             //Create a PyObject for the arguments
	// pVal = PyFloat_FromDouble(6.5); //Set the value of pVal to the altitude
	// PyTuple_SetItem(pArgs, 0, pVal);   //Set the first parameter to the altitude
	// PyObject_CallObject(pFunc, pArgs);
    // std::exit(1);

    // imtermediate img and point
    cv::Rect templ_rect;
    cv::Rect center_rect;
    cv::Mat img_binary; // convert to binary to easily prrcessed

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
    int new_width = image_size.width ;
    int new_height = image_size.height ;

    Resolution new_image_size(new_width, new_height);

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    Mat image_zed(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);
    Mat depth_image_zed_gpu(new_width, new_height, MAT_TYPE::U8_C4, sl::MEM::GPU); // alloc sl::Mat to store GPU depth image
    cv::cuda::GpuMat depth_image_ocv_gpu = slMat2cvMatGPU(depth_image_zed_gpu); // create an opencv GPU reference of the sl::Mat
    cv::Mat depth_image_ocv; // cpu opencv mat for display purposes
    Mat point_cloud;
    int img_counter = 0;
    int fps;
    string final_real_img;
    string final_binary_img;
    // Loop until 'q' is pressed
    char key = ' ';
    while (true) {
        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, new_image_size);
            // image_ocv_copy = image_ocv.clone();
            zed.retrieveImage(depth_image_zed_gpu, VIEW::DEPTH, MEM::GPU, new_image_size);
            depth_image_ocv_gpu.download(depth_image_ocv);
            cv::threshold(depth_image_ocv, img_binary, 80, 255, cv::THRESH_BINARY); //convert depth img to binary

            is_space = finding_best_space(img_binary, templ_rect, center_rect);

            cv::rectangle(image_ocv, templ_rect.tl(), templ_rect.br(), blue, 2); // real img
            cv::rectangle(image_ocv, center_rect.tl(), center_rect.br(), red, 2); // real img
            cv::rectangle(img_binary, templ_rect.tl(), templ_rect.br(), blue, 2); // binary img 
            cv::rectangle(img_binary, center_rect.tl(), center_rect.br(), red, 2); // binary img

            is_matched = check_rect_matched(templ_rect, center_rect);
            manuver(image_ocv, is_space, is_matched, templ_rect, center_rect); // guidance on real img
            manuver(img_binary, is_space, is_matched, templ_rect, center_rect); // guidance on binary img
            
            //show img
            cv::imshow("Real", image_ocv);
            // cv::imshow("Depth", img_binary);

            //save img
            string img_number = std::to_string(img_counter);
            final_real_img = "/home/nguyen/Desktop/img/final_real_img_" + img_number + ".jpg";
            final_binary_img = "/home/nguyen/Desktop/img/binary_real_img_" + img_number + ".jpg";
            if(frame_counter == 0) {
                cv::imwrite(final_real_img, image_ocv);
                cv::imwrite(final_binary_img, img_binary);
                img_counter++;
            }

            // FPS counter:
            fps = fps_counter(frame_counter, final_time, initial_time);
        }
    key = cv::waitKey(1);
    if (key == 'q') {break;}
    }

    // sl::Mat GPU memory needs to be free before the zed
    depth_image_zed_gpu.free();
    zed.close();
    // Py_Finalize();
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


bool finding_best_space(cv::Mat &img_binary, cv::Rect &templ_rect, cv::Rect &center_Rect) {
    cv::Mat img_bianry_cropped; // cropped img to only detect horizontal regions;
    int frame_w = img_binary.cols;
    int frame_h = img_binary.rows;
    int templ_w = frame_w / 4;
    int templ_h = frame_h / 4;
    cv::Mat img_result_templ_matching;
    // double min_val, max_val;
    // cv:: Point min_loc, max_loc;

    cv::Mat img_binary_formatted;
    cv::Mat templ(templ_h, templ_w, CV_8U, cv::Scalar(0, 0, 0));
    cv::Point center_frame_tf;
    center_frame_tf.x = (frame_w / 2) - (templ_w / 2);
    center_frame_tf.y = (frame_h / 2) - (templ_h / 2);
    center_Rect = cv::Rect(center_frame_tf.x, center_frame_tf.y, templ_w, templ_h);
    cv::Point cropped_frame_tf;
    vector<cv::Point> space_loc_tf;

    cv::cvtColor(img_binary, img_binary_formatted, cv::COLOR_BGR2GRAY);
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


int fps_counter(int &frame_counter, int &final_time, int &initial_time) {
    frame_counter++;
    int fps;
    final_time = time(NULL);
    if(final_time - initial_time > 0) {
        fps = frame_counter / (final_time - initial_time);
        cout << "FPS: " << fps <<endl;
        cout << endl;
        frame_counter = 0;
        initial_time = final_time;
    }
    return fps;
}

void manuver(cv::Mat img, bool is_space, bool is_matched, cv::Rect &templ_rect, cv::Rect &center_rect) {
    cv::Scalar red = cv::Scalar(0, 0, 256);
    cv::Scalar blue = cv::Scalar(256, 0, 0);
    if(!is_space) {
        cv::putText(img, "STOP", cv::Point(200, 50), cv::FONT_HERSHEY_PLAIN, 4, red, 3);
    }
    else if(is_matched) {
        cv::putText(img, "go a head", cv::Point(200, 50), cv::FONT_HERSHEY_PLAIN, 4, blue, 3);
    } else if(!is_matched) {
        if(templ_rect.x - center_rect.x > 0) {
            cv::putText(img, " slide right", cv::Point(200, 50), cv::FONT_HERSHEY_PLAIN, 4, blue, 3);
        } else {
            cv::putText(img, " slide left", cv::Point(200, 50), cv::FONT_HERSHEY_PLAIN, 4, blue, 3);
        }
    } else {
        cv::putText(img, "STOP, cannot figure what to do", cv::Point(200, 50), cv::FONT_HERSHEY_PLAIN, 4, red, 3);
    }
}

bool check_rect_matched(cv::Rect templ_rect, cv::Rect center_rect) {
    int error = 5;
    return (templ_rect.tl().x < center_rect.tl().x + error && templ_rect.tl().x > center_rect.tl().x - error);
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