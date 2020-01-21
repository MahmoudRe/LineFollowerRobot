    /**
     * @file CannyDetector_Demo.cpp
     * @brief Sample code showing how to detect edges using the Canny Detector
     * @author OpenCV team
     */

    #include "opencv2/imgproc.hpp"
    #include "opencv2/highgui.hpp"
    #include <iostream>

    using namespace cv;

    void turnLeft(int diff) {
        std::cout << "turn left by " << diff << std::endl;
    }

    void turnRight(int diff) {
        std::cout << "turn right by " << diff << std::endl;
    }

    //![variables]
    Mat src, src_gray;
    Mat dst, detected_edges;

    int lowThreshold = 50;
    const int max_lowThreshold = 100;
    const int ratio = 3;
    const int kernel_size = 3;
    const char *window_name = "Edge Map";


    //![variables]

    /**
     * @function CannyThreshold
     * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
     */
    static void CannyThreshold(int, void *) {


        //![reduce_noise]
        /// Reduce noise with a kernel 3x3
        blur(src_gray, detected_edges, Size(3, 3));

        //![reduce_noise]

        //![canny]
        /// Canny detector
        Canny(detected_edges, detected_edges, lowThreshold, lowThreshold * ratio, kernel_size);
        //![canny]

        /// Using Canny's output as a mask, we display our result
        //![fill]
        dst = Scalar::all(0);
        //![fill]

        //![copyto]
        src.copyTo(dst, detected_edges);
        //![copyto]

        //![display]
        //imshow( window_name, dst );
        //![display]

    }


    /**
     * @function main
     */
    void convert(Mat &srcIm) {

        //   src = imread("/home/themostwanted/CLionProjects/LineFollow/assets/pic2.jpeg");
    //    src = imread("/home/themostwanted/CLionProjects/LineFollow/assets/black.png");

        src = srcIm;


        if (src.empty()) {
            std::cout << "Could not open or find the image!\n" << std::endl;
            //std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
        }
        //![load]

        double angle = -90;

        // get rotation matrix for rotating the image around its center in pixel coordinates
        cv::Point2f center((src.cols-1)/2.0, (src.rows-1)/2.0);
        cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
        // determine bounding rectangle, center not relevant
        cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), src.size(), angle).boundingRect2f();
        // adjust transformation matrix
        rot.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
        rot.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;


        cv::Mat rotated;
        cv::warpAffine(src, rotated, rot, bbox.size());

        src = rotated;


        int offset_x = 0;
        int offset_y = src.rows * 0.75f;

        cv::Rect roi;

        roi.x = offset_x;
        roi.y = offset_y;

        roi.width = src.cols;
        roi.height = src.rows - offset_y;
        roi.height = src.rows /6 ;
        src = src(roi);



        //![create_mat]
        /// Create a matrix of the same type and size as src (for dst)
        dst.create(src.size(), src.type());
        //![create_mat]

        //![convert_to_gray]
        cvtColor(src, src_gray, COLOR_BGR2GRAY);
        //![convert_to_gray]

        //![create_window]
        namedWindow(window_name, WINDOW_AUTOSIZE);
        //![create_window]

        //![create_trackbar]
        /// Create a Trackbar for user to enter threshold
      //  createTrackbar("Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold);
        //![create_trackbar]

        /// Show the image
        CannyThreshold(0, 0);


        int numberOfValues = 0;
        int sum = 0;

        int mean = -1;

    //    cv::findContours(detected_edges , )

        for (int r = 0; r < detected_edges.rows; r++) {
            for (int c = 0; c < detected_edges.cols; c++) {


                //reduce the extreme edges
                if (detected_edges.row(r).col(c).at<int>(0) == 255 && (c >= detected_edges.cols / 4) &&
                    (c <= 3 * detected_edges.cols / 4)) {

                    std::cout << dst.col(c).row(r) << std::endl;

                    sum += c;
                    numberOfValues++;
                }
            }
        }

        if (numberOfValues != 0) {
            mean = sum / numberOfValues;
            std::cout << mean << std::endl;
        }

        if (mean != -1) {
            if (mean < detected_edges.cols / 2) {
                turnLeft(detected_edges.cols / 2 - mean);
            }
            if (mean > detected_edges.cols / 2) {
                turnRight(mean - detected_edges.cols / 2);
            }

        }


    //    for (int i = 0; i < detected_edges.rows; i++){
    //        detected_edges.at<int>(i , mean) = 100;
    //    }

        imshow(window_name, detected_edges);


        /// Wait until user exit program by pressing a key
       // waitKey(0);


    }
