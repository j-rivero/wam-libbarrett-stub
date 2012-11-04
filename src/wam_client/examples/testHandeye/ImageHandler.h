#ifndef imagehandler_H
#define imagehandler_H

#include <iostream>

#include <fstream> //file operations
#include <sys/stat.h>
#include <vector>

#include <cv.h>
#include <highgui.h>
#include "constants.h"

#define BOARD_W 7
#define BOARD_H 8
#define BOARDTOTAL 56

static float Hintrinsic_data[9] =
      {1646.52,      4.96473,      629.935,
      0.00000,      1645.35,      473.293,
      0.00000,      0.00000,      1.00000};

static float Htcp2patternC[16] =
     {0.99989,   -0.0066958,     0.013335,     0.013695,
    -0.013895,    -0.092119,      0.99565,    -0.042249,
   -0.0054383,     -0.99573,    -0.092202,     0.035122,
            0,            0,            0,            1};
static float Hcam2worldC[16] =
   {-0.024706,     -0.99601,     0.085712,      0.61838, 
     -0.99969,     0.024721,   -0.0008759,    -0.019742,
   -0.0012465,    -0.085708,     -0.99632,      0.90426,
            0,            0,            0,            1};

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
        
        //Homography from camera frame to world frame
        std::vector<double> Hcam2world;
        //Homography from tcp frame to pattern frame
        std::vector<double> Htcp2pattern;
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
    public:
        ImageHandler();

        void mat4x4Mul(std::vector<double> matA, std::vector<double> matB, std::vector<double> *result);
        int initStream();
        void closeStream();
        /** save the image
         * saves the image to images/image%d.png
         * @param image the image to save
         * @param take the image number
         */
        void saveImage(int take);
        bool findPattern(std::vector<double> *patternpose);
        void refreshImage();
        IplImage* loadImage(int imagenum);
        void colorSegmentation(IplImage *image);
        void work(std::vector<double> *patternpose);
        void handeyetest(std::vector<double> *wampose);
        void transform(std::vector<double> *wampose,std::vector<double> patternpose);

};
#endif
