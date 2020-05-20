#ifndef POINT_DETECTORS_H
#define POINT_DETECTORS_H

#endif // POINT_DETECTORS_H

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <string>

#define width 640
#define height 480

using namespace cv;
using namespace std;

bool sorting (const cv::Point p1, const cv::Point p2) {
    return ((p1.x ) < (p2.x));
}

class PointDetectors
{
public:
    void edge_points(Mat src_gray, int *first_black_row_mid, int *first_black_col_mid,
                                   int *second_black_row_mid, int *second_black_col_mid)
    {
        int end_loop = 0;
        for (int i = 0; i < height && end_loop != 1; i++) //row indices to inspect pixels
        {
            for (int j = 0; j < width && end_loop != 1; j++) //columns indices to inspect pixels
            {
                if (static_cast<int>(cv::norm(src_gray.row(i).col(j))) == 0) //if the pixel is black, its the first pixel of the target
                {
                    int first_black_r = i; //get the row number of first pixel
                    int first_black_c = j; //get the column number of first pixel

                    int row_to_track = first_black_r + 10; //go 10 pixels down from the first corner point, to find the opposite point
                    int column_to_track;
                    for ( int coli = first_black_c; coli < width; coli++) //traverse along the row from the column of first black pixel to 640th pixel
                    {
                        if (static_cast<int>(cv::norm(src_gray.row(row_to_track).col(coli))) == 255) //find the first white pixel
                        {
                            int last_black_c = coli - 1; //subtract 1 to get the last black pixel of the target
                            column_to_track = (first_black_c + last_black_c)/2; //mid point of the target
                            break;
                        }
                    }
                    for ( int k = 0; k < height; k++) //traverse along the mid column
                    {
                        if (static_cast<int>(cv::norm(src_gray.row(k).col(column_to_track))) == 0) //find the first black pixel along the mid column
                        {
                            *first_black_row_mid = k;                //Y for 1st point
                            *second_black_row_mid = row_to_track;    //Y for 2nd point
                            *first_black_col_mid = column_to_track;  //X for 1st point
                            *second_black_col_mid = first_black_c;   //X for 2nd point
                            cout << "Point to track : " << *first_black_col_mid << ", " << *first_black_row_mid << endl;
                            //cout << "Point to track : " << *second_black_col_mid << ", " << *second_black_row_mid << endl;
                            cout << "------------------------" << endl;
                            end_loop = 1;
                            //exit(0);
                            break;
                        }
                    }
                }
            }
        }
    }

    void corners(vector<Point2f> corners_global)
    {
        std::sort(corners_global.begin(), corners_global.end(), sorting);

        if(corners_global[0].y < corners_global[1].y)
        {
            float x0 = corners_global[0].x;
            float y0 = corners_global[0].y;
            float x1 = corners_global[1].x;
            float y1 = corners_global[1].y;
            corners_global[0].x = x1;
            corners_global[0].y = y1;
            corners_global[1].x = x0;
            corners_global[1].y = y0;
        }
        if(corners_global[2].y < corners_global[3].y)
        {
            float x0 = corners_global[2].x;
            float y0 = corners_global[2].y;
            float x1 = corners_global[3].x;
            float y1 = corners_global[3].y;
            corners_global[2].x = x1;
            corners_global[2].y = y1;
            corners_global[3].x = x0;
            corners_global[3].y = y0;
        }
        cout << "Corners are : " << corners_global << endl;

    }
};
