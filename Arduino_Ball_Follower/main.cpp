// OpenCV_test1.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#define TARGET_FPS 30
#define BUF_SIZE 128


int main()
{

    cv::VideoCapture cap(1); //capture the video from webcam

    if (!cap.isOpened())  // if not success, exit program
    {
        std::cout << "Cannot open the web cam" << std::endl;
        return -1;
    }

    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"


    ////RED
    //int iLowH = 170;
    //int iHighH = 179;

    //int iLowS = 150;
    //int iHighS = 255;

    //int iLowV = 60;
    //int iHighV = 255;

    //BLUE
    int iLowH = 83;
    int iHighH = 103;

    int iLowS = 56;
    int iHighS = 252;

    int iLowV = 63;
    int iHighV = 255;


    //Create trackbars in "Control" window
    cv::createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cv::createTrackbar("HighH", "Control", &iHighH, 179);

    cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", "Control", &iHighS, 255);

    cv::createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
    cv::createTrackbar("HighV", "Control", &iHighV, 255);

    //int iLastX = -1;
    //int iLastY = -1;

    ////Capture a temporary image from the camera
    //cv::Mat imgTmp;
    //cap.read(imgTmp);

    ////Create a black image with the size as the camera output
    //cv::Mat imgLines = cv::Mat::zeros(imgTmp.size(), CV_8UC3);;

    //configure Serial Port communication

    int cport_nr = 24; /* /dev/ttyUSB0 */
    int bdrate = 9600; /* 9600 baud */

    char mode[] = { '8','N','1',0 }; // 8 data bits, no parity, 1 stop bit
    char str_send[2][BUF_SIZE]; // send data buffer
    unsigned char str_recv[BUF_SIZE]; // recv data buffer
    strcpy(str_send[0], "V(20,30)");
    strcpy(str_send[1], "V(-3,-6)");

    if (RS232_OpenComport(cport_nr, bdrate, mode))
    {
        printf("Can not open com port\n");
        return(-5);
    }
    //using namespace std::chrono_literals;
    //Stabilize communication
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));


    ////testing the communication with RS232 loop-back or arduino echo program
    //for(int i=0; i< 10; i++)
    //{
    //	RS232_cputs(cport_nr, str_send[i%2]); // sends string on serial
    //	printf("Sent to Arduino: '%s'\n", str_send[i%2]);
    //	std::this_thread::sleep_for(33ms);

    //	int n = RS232_PollComport(cport_nr, str_recv, (int)BUF_SIZE);
    //	if (n > 0) {
    //		str_recv[n] = 0;
    //		/* always put a "null" at the end of a string! */
    //		printf("Received %i bytes: '%s'\n", n, (char *)str_recv);
    //	}

    //	std::this_thread::sleep_for(33ms);

    //}



    while (true)
    {
        auto time_start = std::chrono::high_resolution_clock::now();
        cv::Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video
        //Scale down for faster processing
        cv::pyrDown(imgOriginal, imgOriginal);

        if (!bSuccess) //if not success, break loop
        {
            std::cout << "Cannot read a frame from video stream" << std::endl;
            break;
        }

        cv::Mat imgHSV;
        cv::Mat imgGray;

        cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        ////scale down - faster processing
        //cv::pyrDown(imgHSV, imgHSV);

        cv::cvtColor(imgOriginal, imgGray, cv::COLOR_BGR2GRAY);
        ////scale down - faster processing
        //cv::pyrDown(imgGray, imgGray);


        cv::Mat imgThresholded;

        inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

                                                                                                      //morphological opening (removes small objects from the foreground)
        erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

        //morphological closing (removes small holes from the foreground)
        dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

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
            //calculate the position of the ball
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
                std::cout << "Sending scaled vector: "<< stringToSend << std::endl;


                RS232_cputs(cport_nr, stringToSend.c_str());


                int n = RS232_PollComport(cport_nr, str_recv, (int)BUF_SIZE);
                if (n > 0) {
                    str_recv[n] = 0;
                    /* always put a "null" at the end of a string! */
                    printf("Received %i bytes: '%s'\n", n, (char *)str_recv);
                }



                //Draw a red line from the middle to ball position
                cv::line(imgOriginal, cv::Point(imgOriginal.cols / 2, imgOriginal.rows / 2), cv::Point(posX, posY), cv::Scalar(0, 0, 255), 5);

                //Hough Transform - detecting circles
                //This can be used to track obects that we are sure are of interest (another form of noise cancelation)
                cv::GaussianBlur(imgThresholded, imgThresholded, cv::Size(9, 9), 2, 2);
                std::vector<cv::Vec3f> circles;
                cv::HoughCircles(imgThresholded, circles, cv::HOUGH_GRADIENT, 2, imgThresholded.rows / 4, 200, 50);

                if (circles.size() > 0)
                {
                    std::cout << "Circles on screen: " << circles.size() << std::endl;


                    //for each (auto circle in circles)
                    for(auto it = circles.begin(); it< circles.end(); it++)
                    {
                        //cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
                        //int radius = cvRound(circle[2]);
                        cv::Point center (cvRound( (*it)[0] ), cvRound( (*it)[1]) );
                        int radius = cvRound((*it)[2]);

                        std::cout << "circle: (" << center.x << "," << center.y << "); radius: " << radius << std::endl;
                        cv::circle(imgOriginal, center, radius, cv::Scalar(255, 0, 0), 4);

                    }
                }

            }

        }



        cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
        cv::imshow("Black&White", imgGray);


        cv::imshow("Original", imgOriginal); //show the original image

        auto time_end = std::chrono::high_resolution_clock::now();
        auto time_passed = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
        auto willWaitFor = std::max(1, (1000 / TARGET_FPS) - (int)time_passed);
        //auto willWaitFor = std::max(1, (1000 / TARGET_FPS) - (int)time_passed);


        std::cout << "Processing time was: "
            << (int)time_passed
            << "ms. Will wait for: " << willWaitFor << "ms" << std::endl;
        int key = cvWaitKey(willWaitFor);
        key = (key == 255) ? -1 : key;
        if (key == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            std::cout << "esc key is pressed by user" << std::endl;
            cv::destroyAllWindows();
            break;
        }
    }




    return 0;

}

