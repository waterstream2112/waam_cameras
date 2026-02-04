#include <iostream>
#include <opencv2/opencv.hpp>
#include "neoapi/neoapi.hpp"

int main() {
    try {
        // 1. Initialize the camera
        NeoAPI::Cam camera;
        camera.Connect(); // Connects to the first available camera

        // 2. Setup Camera Parameters (Optional)
        if (camera.HasFeature("ExposureAuto")) {
            camera.SetFeature("ExposureAuto", "Continuous");
        }

        // 3. Start Acquisition
        // std::cout << "Starting camera: " << camera.GetFeature("DeviceModelName") << std::endl;
        
        while (true) {
            // 4. Get an image from the camera
            NeoAPI::Image image = camera.GetImage();

            if (!image.IsEmpty()) {
                // 5. Convert neoAPI Image to OpenCV Mat
                // neoAPI provides a direct pointer to the image data
                cv::Mat frame(
                    (int)image.GetHeight(),
                    (int)image.GetWidth(),
                    CV_8UC1, // Use CV_8UC3 for Color cameras
                    image.GetImageData()
                );

                // 6. Display the image
                cv::imshow("Baumer USB3 Camera - Press ESC to Exit", frame);
            }

            // 7. Break loop on ESC key
            if (cv::waitKey(1) == 27) break;
        }

        camera.Disconnect();
    } catch (NeoAPI::NeoException& e) {
        std::cerr << "neoAPI Exception: " << e.GetDescription() << std::endl;
    } catch (std::exception& e) {
        std::cerr << "Standard Exception: " << e.what() << std::endl;
    }

    return 0;
}