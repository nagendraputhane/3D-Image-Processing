#ifndef FEATURE_DETECTORS_H
#define FEATURE_DETECTORS_H

#endif // FEATURE_DETECTORS_H

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

int getMaxAreaContourId(vector <vector<cv::Point>> contours) {
    double maxArea = 0;
    double a1 = 3000, a2 = 15000;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++) {
        cv::Rect boundRect = cv::boundingRect(contours[j]);
        double newArea = boundRect.area();
        if (newArea > maxArea) {
        //if(newArea > a1 && newArea < a2){
            maxArea = newArea;
            maxAreaContourId = j;
        } // End if
    } // End for
    return maxAreaContourId;
}

class FeatureDetectors
{
public:
    void harris(Mat src_gray, vector<Point2f> corners_global)
    {
        int blockSize = 5, apertureSize = 3, thresh = 130; //2, 75
        double k = 0.04;

        std::vector<cv::Point2f> corners;
        Mat dst = Mat::zeros( src_gray.size(), CV_32FC1 );

        cornerHarris( src_gray, dst, blockSize, apertureSize, k );

        /// Normalizing
        Mat dst_norm, dst_norm_scaled;
        normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
        convertScaleAbs( dst_norm, dst_norm_scaled );

        for(int i = 0; i < corners_global.size(); i++)
            circle( dst_norm_scaled, corners_global[i], 5,  Scalar(255), 1, LINE_AA, 0 );

        namedWindow( "Corners detected" );
        imshow( "Corners detected", src_gray);
        cvtColor( dst_norm_scaled, dst_norm_scaled, cv::COLOR_GRAY2BGR );
        //video << dst_norm_scaled;
    }

    void hough(Mat src_gray)
    {
        std::vector<Vec2f> lines;
        cv::Mat src_gray_c;
        cv::HoughLines(src_gray, lines, 1, CV_PI/180, 108); //150

        cvtColor(src_gray, src_gray_c, COLOR_GRAY2BGR);
        for( size_t i = 0; i < lines.size(); i++ )
        {
            float rho = lines[i][0], theta = lines[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            line( src_gray_c, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);

            namedWindow( "Edges detected" );
            imshow( "Edges detected", src_gray_c );


        }
    }


    Mat rect(Mat frame)
    {
        Mat src_gray = frame, canny_output;
        Scalar color = Scalar(0, 255, 0);
        vector<vector<Point> > contours;

        //cvtColor( frame, src_gray, COLOR_BGR2GRAY );

        //blur( src_gray, src_gray, Size(4,4) );
        //cv::threshold(src_gray, src_gray, 85, 255, 0);

        cv::Mat src_gray_new = cv::Mat::ones(src_gray.size(), src_gray.type()) * 255; //create blank frame with white pixels
        cv::Mat src_gray_new_ = cv::Mat::ones(src_gray.size(), src_gray.type()) * 255;
        //src_gray.colRange(307, 379).copyTo(src_gray_new.colRange(307, 379)); //phone X - 480
        //src_gray_new.rowRange(65, 480).copyTo(src_gray_new_.rowRange(65, 480));
        src_gray.rowRange(230, 260).copyTo(src_gray_new.rowRange(230,260)); //phone Y - 480
        //src_gray_new.colRange(69, 640).copyTo(src_gray_new_.colRange(69, 640));
        //src_gray.rowRange(295, 443).copyTo(src_gray_new.rowRange(295, 443)); //phone Y - 720
        //src_gray.colRange(600, 740).copyTo(src_gray_new.colRange(600, 740)); //phone X - 720
/*        Canny( src_gray_new_, canny_output, 120, 200, 3 ); //find edges

        findContours( canny_output, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE ); //find contours

        Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 ); //used to draw rectangles

        Rect boundRect = cv::boundingRect(contours.at(getMaxAreaContourId(contours))); //find the largest rectangle

        //cout << boundRect << " Area : " << boundRect.area() << " TopLeft : " << boundRect.tl() << " BottomRight : " << boundRect.br() <<endl;

        Mat blackRect(720, 1280, CV_8UC1);

        blackRect.setTo(255); //set all pixels to white

        //blackRect(boundRect).setTo(0); //set the largest rectangle as black

        frame(boundRect).copyTo(blackRect(boundRect));

        rectangle( drawing, boundRect.tl(), boundRect.br(), color, 1 ); //draw rectangle boundry
*/
        namedWindow( "Contours" );
        imshow( "Contours", src_gray_new);
        //namedWindow( "Input" );
        //imshow( "Input", src_gray_new);
        char c=(char)waitKey(0);
        if(c==27)
            exit(0);

        return src_gray_new_;

    }
};
