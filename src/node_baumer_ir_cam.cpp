#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "neoapi/neoapi.hpp"


using namespace std::chrono_literals;

class NodeBaumerIrCam : public rclcpp::Node
{
private:
    NeoAPI::Cam camera_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Camera parameters
    std::string topicOutRawImage_;
    float FrameRate_;
    std::string ExposureAuto_;
    std::string WhiteBalance_;
    int WhiteBalanceRed_;
    int WhiteBalanceBlue_;
    std::string TriggerMode_;
    std::string TriggerSource_;
    int ExposureTime_;
    float Gain_;
    int Width_;
    int Height_;
    int OffsetX_;
    int OffsetY_;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

public:
    NodeBaumerIrCam() : Node("node_baumer_ir_cam")
    {
        // Get parameters from config file
        getParams();

        // Initialize camera and other resources here
        camera_.Connect(); // Connects to the first available camera

        if (camera_.IsConnected()) 
        {
            auto fs = NeoAPI::FeatureStack();
            fs.Add("ExposureAuto", "Continuous");       
            // fs.Add("TriggerMode", "On");
            // fs.Add("TriggerSource", "Software");
            fs.Add("ExposureTime", 20000);      
            fs.Add("Gain", 1.2);
            fs.Add("Width", 300);
            fs.Add("Height", 300);
            fs.Add("OffsetX", 800);
            fs.Add("OffsetY", 100);
            camera_.WriteFeatureStack(fs);       // write all features in one go to the camera
        }
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "Camera not connected!");
            return;
        }


        // Initialize the publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topicOutRawImage_, 10);


        // Create a timer
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / FrameRate_), std::bind(&NodeBaumerIrCam::timerCallback, this));
    }


    ~NodeBaumerIrCam()
    {
        // Cleanup camera and other resources here
        camera_.Disconnect();
    }


    void getParams()
    {
        topicOutRawImage_        = this->declare_parameter<std::string>("topic_out_raw_image", "/baumer_ir/raw_image");
        FrameRate_               = this->declare_parameter<float>("FrameRate", 30.0);
        ExposureAuto_            = this->declare_parameter<std::string>("ExposureAuto", "Continuous");
        WhiteBalance_            = this->declare_parameter<std::string>("WhiteBalance", "Off");
        WhiteBalanceRed_         = this->declare_parameter<int>("WhiteBalanceRed", 2000);
        WhiteBalanceBlue_        = this->declare_parameter<int>("WhiteBalanceBlue", 2000);
        TriggerMode_             = this->declare_parameter<std::string>("TriggerMode", "Off");
        TriggerSource_           = this->declare_parameter<std::string>("TriggerSource", "Software");
        ExposureTime_            = this->declare_parameter<int>("ExposureTime", 20000);
        Gain_                    = this->declare_parameter<float>("Gain", 1.0);
        Width_                   = this->declare_parameter<int>("Width", 640);
        Height_                  = this->declare_parameter<int>("Height", 480);
        OffsetX_                 = this->declare_parameter<int>("OffsetX", 0);
        OffsetY_                 = this->declare_parameter<int>("OffsetY", 0);
    }


    void timerCallback()
    {
        try 
        {
            // Get an image from the camera
            NeoAPI::Image image = camera_.GetImage();

            if (!image.IsEmpty()) 
            {
                // 1. Convert neoAPI Image to OpenCV Mat
                cv::Mat frame(
                    (int)image.GetHeight(),
                    (int)image.GetWidth(),
                    CV_8UC1, // Use CV_8UC3 for Color cameras
                    image.GetImageData()
                );

                // 2. Wrap the Mat in a cv_bridge object
                // "mono8" is the ROS equivalent for CV_8UC1
                std_msgs::msg::Header header;
                header.stamp = this->get_clock()->now();
                header.frame_id = "camera_optical_frame";

                cv_bridge::CvImage cv_img(header, "mono8", frame);

                // 3. Convert to ROS Image Message and Publish
                // .toImageMsg() performs a deep copy of the data, 
                // ensuring the data survives after the 'image' object goes out of scope.
                publisher_->publish(*cv_img.toImageMsg());
            }
        } 
        // catch (NeoAPI::NeoException& e) 
        // {
        //     RCLCPP_ERROR(this->get_logger(), "neoAPI Exception: %s", e.GetDescription().c_str());
        // } 
        catch (std::exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Standard Exception: %s", e.what());
        }
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NodeBaumerIrCam>());
    rclcpp::shutdown();
    return 0;
};




// int main() {
//     try {
//         // 1. Initialize the camera
//         NeoAPI::Cam camera;
//         camera.Connect(); // Connects to the first available camera

//         // 2. Setup Camera Parameters (Optional)
//         if (camera.HasFeature("ExposureAuto")) {
//             camera.SetFeature("ExposureAuto", "Continuous");
//         }

//         // if (camera.IsConnected()) {
//         //     auto fs = NeoAPI::FeatureStack();
//         //     fs.Add("ExposureAuto", "Off");       
//         //     fs.Add("TriggerMode", "On");
//         //     fs.Add("TriggerSource", "Software");
//         //     fs.Add("ExposureTime", 20000);      
//         //     fs.Add("Gain", 1.2);
//         //     fs.Add("Width", 300);
//         //     fs.Add("Height", 300);
//         //     fs.Add("OffsetX", 800);
//         //     fs.Add("OffsetY", 100);
//         //     camera.WriteFeatureStack(fs);       // write all features in one go to the camera
//         // }

//         // 3. Start Acquisition
//         // std::cout << "Starting camera: " << camera.GetFeature("DeviceModelName") << std::endl;
        
//         while (true) {
//             // 4. Get an image from the camera
//             NeoAPI::Image image = camera.GetImage();

//             if (!image.IsEmpty()) {
//                 // 5. Convert neoAPI Image to OpenCV Mat
//                 // neoAPI provides a direct pointer to the image data
//                 cv::Mat frame(
//                     (int)image.GetHeight(),
//                     (int)image.GetWidth(),
//                     CV_8UC1, // Use CV_8UC3 for Color cameras
//                     image.GetImageData()
//                 );

//                 // 6. Display the image
//                 cv::imshow("Baumer USB3 Camera - Press ESC to Exit", frame);
//             }

//             // 7. Break loop on ESC key
//             if (cv::waitKey(1) == 27) break;
//         }

//         camera.Disconnect();
//     } catch (NeoAPI::NeoException& e) {
//         std::cerr << "neoAPI Exception: " << e.GetDescription() << std::endl;
//     } catch (std::exception& e) {
//         std::cerr << "Standard Exception: " << e.what() << std::endl;
//     }

//     return 0;
// }