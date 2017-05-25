#include "stdafx.h"

#include "ArduinoSerialCommunicator.hpp"
#define TARGET_FPS 30
#define BUF_SIZE 128


int main()
{

    cv::VideoCapture cap(0); //capture the video from webcam

    if (!cap.isOpened())  // if not success, exit program
    {
        std::cout << "Cannot access the camera with given index" << std::endl;
        return -1;
    }

    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE);
    //create a window called "Control"


    ////RED
    //int iLowH = 170;
    //int iHighH = 179;

    //int iLowS = 150;
    //int iHighS = 255;

    //int iLowV = 60;
    //int iHighV = 255;

    //BLUE
    int iLowH = 93;
    int iHighH = 120;

    int iLowS = 196;
    int iHighS = 255;

    int iLowV = 121;
    int iHighV = 255;


    //Create trackbars in "Control" window
    cv::createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cv::createTrackbar("HighH", "Control", &iHighH, 179);

    cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", "Control", &iHighS, 255);

    cv::createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
    cv::createTrackbar("HighV", "Control", &iHighV, 255);


    ////configure Serial Port communication

    ArduinoSerialCommunicator arduinoCOM6("/dev/ttyACM0");
    if (!arduinoCOM6.isOpened())
    {
        std::cout << "Could not open COM port" << std::endl;
        return(-5);
    }


    //Stabilize communication
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));


    //testing the communication with RS232 loop-back or arduino echo program
    for (int i = 0; i < 10; i++)
    {
        std::string tempStringToSend = "V(20,30)";
        arduinoCOM6.sendString(tempStringToSend);
        printf("Sent to Arduino: '%s'\n", tempStringToSend.c_str());

        std::this_thread::sleep_for(std::chrono::milliseconds(33));

        std::string temp = arduinoCOM6.pollSerialPortForData();
        std::cout << temp << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(33));

    }

    std::vector<std::vector<cv::Point>> contouredElementsOnScreen;

    bool dilate_erode = true, hough_circles = true, canny = true;

    while (true)
    {
        auto time_start = std::chrono::high_resolution_clock::now();
        cv::Mat imgOriginal;

        // read a new frame from video
        bool bSuccess = cap.read(imgOriginal);

        if (!bSuccess) //if not success, break loop
        {
            std::cout << "Could not read frame from the video stream" << std::endl;
            break;
        }


        //Scale down one step for faster processing
        cv::pyrDown(imgOriginal, imgOriginal);


        //Convert the captured frame from BGR to HSV
        cv::Mat imgHSV;
        cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV);

        //Convert the captured frame to greyscale
        cv::Mat imgGray;
        cv::cvtColor(imgOriginal, imgGray, cv::COLOR_BGR2GRAY);



        //Threshold the image
        cv::Mat imgThresholded;
        inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);

        if (dilate_erode)
        {
            ////morphological opening (removes small objects from the foreground)
            erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
            dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

            //morphological closing (removes small holes from the foreground)
            dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
            erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        }


        cv::Mat canny_out;
        if (canny)
        {
            cv::Canny(imgThresholded, canny_out, 100, 200);
        }


        //******************************************************************************************************************************************************
        //Finding circles - one way


        //Find the contours of the image
        contouredElementsOnScreen.clear();


        //cv::findContours(canny_out, conturedElementsOnScreen, cv::RetrievalModes::RETR_EXTERNAL, cv::ContourApproximationModes::CHAIN_APPROX_NONE);
        cv::findContours(imgThresholded, contouredElementsOnScreen, cv::RetrievalModes::RETR_EXTERNAL, cv::ContourApproximationModes::CHAIN_APPROX_NONE);



        //Discard all the elements from the list, for which the contour is smaller than {X} points
        unsigned int minPointsInContour = 50;

        //Discarding using C++ iterators
        //for (auto it = conturedElementsOnScreen.begin(); it < conturedElementsOnScreen.end(); )
        //{
        //	auto contour = *it;
        //	if (contour.size() < minPointsInContour)
        //	{
        //		it = conturedElementsOnScreen.erase(it);
        //	}
        //	else  ++it;
        //}

        //Discarding using C++11 std::remove_if and lambda expressions
        contouredElementsOnScreen.erase(std::remove_if(contouredElementsOnScreen.begin(),
            contouredElementsOnScreen.end(),
            [ = ](const std::vector<cv::Point>& contour)
                    {
                        if (contour.size() < minPointsInContour ) return true;
                        else { return false; }
                    }),
            contouredElementsOnScreen.end());

        //Iterate over what's left (only big contours) and try to find all the circles
        std::vector<cv::Vec3f> circlesFoundUsingContours;
        for (auto contour : contouredElementsOnScreen)
        {
            //if (contour.size() < 100) continue;
            cv::Moments tempMoments = cv::moments(contour);
            //double dM02 = tempMoments.m02;
            //double dM20 = tempMoments.m20;
            double dN02 = tempMoments.nu02;
            double dN20 = tempMoments.nu20;

            double area = tempMoments.m00;
            double radius = std::sqrt(area / 3.141592);

            double maxMoment = std::max(dN02, dN20);
            double minMoment = std::min(dN02, dN20);

            if ((maxMoment / minMoment) > 0.75 && (maxMoment / minMoment) < 1.25)
            {
                std::cout << "Found circle" << std::endl;
                int posY = tempMoments.m01 / tempMoments.m00;
                int posX = tempMoments.m10 / tempMoments.m00;

                ////TODO:
//                //Check if this is truly a circle - calculate how far should be each contour point from the center
//                std::vector<float> differenceInDistanceFromCenter;
//                float average = 0;
//                for(cv::Point point : contour)
//                {
//                    float theoreticalRadius = std::sqrt( std::pow((point.x - posX),2) + std::pow((point.y - posY),2 ) );
//                    float diff = radius - theoreticalRadius;
//                    average += diff;
//                    differenceInDistanceFromCenter.push_back(diff);
//                }
//                average = average / differenceInDistanceFromCenter.size();


                if (posX > 0 && posY > 0)
                {
                    circlesFoundUsingContours.push_back(cv::Vec3f(posX, posY, radius));
                    cv::circle(imgGray, cv::Point{ posX, posY }, radius, cv::Scalar{ 255,0,0 }, 5);
                }
            }

        }

        //Draw remaining contours on the screen
        cv::drawContours(imgGray, contouredElementsOnScreen, -1, cv::Scalar{ 255,0,0 }, 2);


        //******************************************************************************************************************************************************
        //FINDING Objects of interest - another way, using blobs of single color and (optionally) Hough circles algorithm
        std::vector<cv::Vec3f> circlesFoundUsingHoughes;


        //Calculate the moments of the thresholded image
        cv::Moments oMoments = cv::moments(imgThresholded);

        double dM01 = oMoments.m01;
        double dM10 = oMoments.m10;
        double dArea = oMoments.m00;

        //Create a vector for storing information about how much the camera has to move in each direction;
        cv::Vec2i cameraMovementVector(0, 0);

        // if the area of the found object is not big enough, then it's just noise
        if (dArea > 10000)
        {
            //calculate the position of the blob
            int posX = dM10 / dArea;
            int posY = dM01 / dArea;

            if (posX >= 0 && posY >= 0)
            {
                cv::Point screenCenter(imgOriginal.cols / 2, imgOriginal.rows / 2);
                std::cout << "Point coordinates: Middle: (" << imgOriginal.cols / 2 << "," << imgOriginal.rows / 2 << ",); Current Point: (" << posX << "," << posY << ")" << std::endl;
                cameraMovementVector[0] = posX - screenCenter.x;
                cameraMovementVector[1] = posY - screenCenter.y;
                std::cout << "Vector coords: (" << cameraMovementVector[0] << ", " << cameraMovementVector[1] << ") " << std::endl;

                //Scaling camera movement vector from image coordinates to range (-20,20) -> servo movement (degrees)
                //int NewValue = ( ( (OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin;
                cv::Vec2i scaledVector(0, 0);
                scaledVector[0] = (((cameraMovementVector[0] - (-imgOriginal.cols / 2)) * (20 - (-20))) / ((imgOriginal.cols / 2) - (-imgOriginal.cols / 2))) + (-20);
                scaledVector[1] = (((cameraMovementVector[1] - (-imgOriginal.rows / 2)) * (20 - (-20))) / ((imgOriginal.rows / 2) - (-imgOriginal.rows / 2))) + (-20);

                std::string stringToSend;
                stringToSend += std::string("V(") + std::to_string(scaledVector[0]) + std::string(",") + std::to_string(scaledVector[1]) + std::string(")");
                std::cout << "Sending scaled vector: " << stringToSend << std::endl;

				
                //Sending data to arduino and polling for response
                arduinoCOM6.sendString(stringToSend);
                std::cout << "Arduino response: " << arduinoCOM6.pollSerialPortForData() << std::endl;



                //Draw a red line from the middle to ball position
                cv::line(imgOriginal, cv::Point(imgOriginal.cols / 2, imgOriginal.rows / 2), cv::Point(posX, posY), cv::Scalar(0, 0, 255), 5);


                if (hough_circles)
                {
                    //Hough Transform - detecting circles
                    cv::GaussianBlur(imgThresholded, imgThresholded, cv::Size(9, 9), 2, 2);
                    cv::HoughCircles(imgThresholded, circlesFoundUsingHoughes, cv::HOUGH_GRADIENT, 2, imgThresholded.rows / 4, 200, 50);

                    if (circlesFoundUsingHoughes.size() > 0)
                    {
                        std::cout << "Hough circles on screen: " << circlesFoundUsingHoughes.size() << std::endl;

                        for (auto circle : circlesFoundUsingHoughes)
                        {
                            cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
                            int radius = cvRound(circle[2]);

                            std::cout << "circle: (" << center.x << "," << center.y << "); radius: " << radius << std::endl;
                            cv::circle(imgOriginal, center, radius, cv::Scalar(255, 0, 0), 4);

                        }
                    }

                }


            }

        }

        //Accessing the found circles
        for (auto circle : circlesFoundUsingContours)
        {
            //do something
        }

        for (auto circle : circlesFoundUsingHoughes)
        {
            //do something

        }


        //show the thresholded image
        cv::imshow("Thresholded Image", imgThresholded);
        //show B&W image
        cv::imshow("Black&White", imgGray);
		
        if (canny)
        {
            cv::imshow("Canny", canny_out);

        }


        //show the original image with annotations
        cv::imshow("Original", imgOriginal);

        //timing the code execution
        auto time_end = std::chrono::high_resolution_clock::now();
        auto time_passed = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
        auto willWaitFor = std::max(1, (1000 / TARGET_FPS) - (int)time_passed);


        std::cout << "Processing time was: "
            << (int)time_passed
            << "ms. Will wait for: " << willWaitFor << "ms" << std::endl;
		
		
        //Wait for key presses
        int key = cvWaitKey(willWaitFor);
        key = (key == 255) ? -1 : key;
		
        //handling key presses - enable/disable functionality
        switch (key)
        {
            case 27:
            {
                std::cout << "esc key is pressed by user" << std::endl;
                cv::destroyAllWindows();
                return 0;
            }
            case 'd':
            {
                dilate_erode = !dilate_erode;
                break;
            }
            case 'c':
            {
                canny = !canny;
                break;
            }
            case 'h':
            {
                hough_circles = !hough_circles;
            }
        }

    }




    return 0;

}
