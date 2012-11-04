#ifndef imagehandler_H
#define imagehandler_H

#include <iostream>

#include <fstream> //file operations
#include <sys/stat.h>
#include <vector>

#include <cv.h>
#include <highgui.h>
#include "constants.h"

#define BOARD_W 5
#define BOARD_H 8
#define BOARDTOTAL 40
#define SQUARESIZE 20

static float Hintrinsic_data[9] =
      {1646.52,      4.96473,      629.935,
      0.00000,      1645.35,      473.293,
      0.00000,      0.00000,      1.00000};

static float Kdistortion_data[4] =
            {-0.211152,     0.202376,      0.00000, 0.00000};
            
class ImageHandler {
    private:
        /// grabbed camera ID (i.e. 0)
        int camera;
/** Camera querying capture*/
        CvCapture* capture;
        /// buffer image
        IplImage *workImage;
        /// grey image pointer
        IplImage *gray;
        /// Is streaming working?
        bool streaming;
        
                /// Intrinsic camera parameters matrix
        CvMat Hintrinsic;

        /// distortion coeficients array
        CvMat Kdistortion;

        /// undistortion map x
        IplImage* mapx;
        /// undistortion map y
        IplImage* mapy;
       
        /** pattern chasing stuff*/
         CvMat* image_points;
         CvMat* object_points;
         CvMat* intrinsic_matrix;
         CvMat* distortion_coeffs;
        void printMat(CvMat *A);
        void logpos(double x, double y, double z);
    public:
        ImageHandler();

        int initStream();
        void closeStream();
        /** save the image
         * saves the image to images/image%d.png
         * @param image the image to save
         * @param take the image number
         */
        void saveImage(int take);
        void findPattern(std::vector<double> *patternpose);
        void work(std::vector<double> *patternpose);

};
#endif
