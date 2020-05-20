#include "feature_detectors.h"
#include  "point_detectors.h"
#include "capture_frame.h"

int main()
{
    FeatureDetectors fd;
    PointDetectors pd;
    CaptureFrames cf;

    std::vector<cv::Point2f> corners_global; //Corner points selected from goodFeaturesToTrack()
    int frame_width, frame_height;
    int first_black_row_mid, first_black_col_mid; //Middle-point along width
    int second_black_row_mid, second_black_col_mid; //Middle-point along height

    //VideoCapture cap("/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/ios/3_T_X_P_Y_480/Frames.m4v");
    /*frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    VideoWriter video("/home/iq9/nagendra/delay/dataset/output_corners.avi", CV_FOURCC('M','J','P','G'),
                      30, Size(frame_width,frame_height));*/

    //std::ofstream corner_csv;
    //corner_csv.open ("/home/iq9/nagendra/delay/corners.csv");

    std::ofstream points_to_track; //File that contains edge points
    points_to_track.open ("/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/android/Y/9_T_X_P_Y/points_to_track.csv");//top left, bottom left, top right, bottom right
    std::ifstream stream ("/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/android/Y/9_T_X_P_Y/data_image.csv"); //read file that contain frame names

    Mat src_gray, frame;

    string line, frame_name;

    //for images
    while(stream >> line) //read line by line
    {
        frame = cf.fromFrame(line);

    //for video
    /*while(1)
    {
        frame = cf.fromVideo(cap);*/

        if (frame.empty())
            break;

        cvtColor( frame, src_gray, COLOR_BGR2GRAY ); //Covert to Gray scale

        //cv::threshold(src_gray, src_gray, 160, 255, ThresholdTypes::THRESH_BINARY); //Apply thresholding on grayscale image 80 255

        //cv::goodFeaturesToTrack(src_gray, corners_global, 4, 0.01, 2, noArray(), 5,true, 0.04); //extract four good feature points
        //fd.harris(src_gray, corners_global); //get harris corner points
        //fd.hough(src_gray); //draw hough lines
        src_gray = fd.rect(src_gray); //extract only the target from the frame

        //pd.corners(corners_global);
        pd.edge_points(src_gray, &first_black_row_mid, &first_black_col_mid, &second_black_row_mid, &second_black_col_mid); //extract edge points

        /*corner_csv << corners_global[0].x << "," << corners_global[0].y << "," <<
                      corners_global[1].x << "," << corners_global[1].y << "," <<
                      corners_global[2].x << "," << corners_global[2].y << "," <<
                      corners_global[3].x << "," << corners_global[3].y << "\n";*/
        points_to_track << first_black_col_mid << "," << first_black_row_mid << "," <<
                               second_black_col_mid << "," << second_black_row_mid << "\n"; //save edge points

        first_black_row_mid = first_black_col_mid = second_black_row_mid = second_black_col_mid = 0;

        /*cv::imshow("Image", src_gray);
        char c=(char)waitKey(1);
        if(c==27)
            break;*/

    }
    //corner_csv.close();
    points_to_track.close();
    //cap.release();
}
