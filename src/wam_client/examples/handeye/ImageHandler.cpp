#include "ImageHandler.h"

using namespace std;

ImageHandler::ImageHandler(){

    camera = 0;
    workImage = NULL;
    gray = NULL;

    Hintrinsic = cvMat(3,3,CV_32FC1,Hintrinsic_data);
    Kdistortion = cvMat(4,1,CV_32FC1,Kdistortion_data);

    image_points      = cvCreateMat(BOARDTOTAL,2,CV_32FC1);
    object_points     = cvCreateMat(BOARDTOTAL,3,CV_32FC1);
    intrinsic_matrix  = cvCreateMat(3,3,CV_32FC1);
    distortion_coeffs = cvCreateMat(4,1,CV_32FC1);

    for( int i=0; i<BOARDTOTAL; i++ ) {
        CV_MAT_ELEM(*object_points,float,i,0) = (float) 20*i/BOARD_W;
        CV_MAT_ELEM(*object_points,float,i,1) = (float) 20*(i%BOARD_W);
        CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
    }
}

int ImageHandler::initStream(){
    //set capture properies and all
    cout << "Camera "<<camera<<" chosen."<<endl;

    capture = cvCaptureFromCAM(camera);

    cvSetCaptureProperty(capture,CV_CAP_PROP_FPS,1.875);
//    cvSetCaptureProperty(capture,CV_CAP_PROP_FPS,7.5);
    cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,800);
    cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT,600);

    if(capture){
        DEBUG("Capturing from camera %d\n",camera);
        cvStartWindowThread();
        //cvNamedWindow("Object", CV_WINDOW_AUTOSIZE);
        cvNamedWindow("Object", NULL);
        cvResizeWindow("Object", 400, 300);
        DEBUG("Querying image\n");
        workImage = cvQueryFrame(capture);
        DEBUG("Showing image\n");
        cvShowImage("Object", workImage);

        /** image dependent data members init*/
        gray = cvCreateImage( cvGetSize(workImage), IPL_DEPTH_8U, 1 ); // allocate a 1 channel byte image
        /*************************************/
        return 0;
    }else{
        cout<< "Error capturing camera!!"<<endl;
        return -1;
    }
}

void ImageHandler::closeStream(){
    //set capture properies and all
    DEBUG("Stopping streaming.\n");
    cvReleaseCapture(&capture);
    DEBUG("Closing window\n");
    cvDestroyWindow("Object");
    cvWaitKey(10);
}

void ImageHandler::saveImage(int take){
    char filename[40];
    struct stat st;
    if(stat("images",&st) == 0)
        snprintf(filename,40*sizeof(char),"images/image%d.png",take);
    else
        snprintf(filename,40*sizeof(char),"image%d.png",take);
    cvSaveImage(filename, workImage);
}
  
void ImageHandler::printMat(CvMat *A){
    int i, j;
    for (i = 0; i < A->rows; i++)
    {
        fprintf(stderr,"\n");
        switch (CV_MAT_DEPTH(A->type))
        {
            case CV_32F:
            case CV_64F:
                for (j = 0; j < A->cols; j++)
                    fprintf (stderr, "%8.3f ", (float)cvGetReal2D(A, i, j));
                break;
            case CV_8U:
            case CV_16U:
                for(j = 0; j < A->cols; j++)
                    fprintf (stderr, "%6d",(int)cvGetReal2D(A, i, j));
                break;  
            default:
                break;
        }
    }
    fprintf(stderr,"\n");
}

void ImageHandler::findPattern(std::vector<double>* ppose){
  CvPoint2D32f corners[48];
  int numCornersFound;
  int found;
  CvPoint onecorner;   
  double patternpose[16];
  CvMat* rotation_vector = cvCreateMat(3,1,CV_32FC1);
  CvMat* translation_vector = cvCreateMat(3,1,CV_32FC1);
  CvMat* rotation_matrix = cvCreateMat(3,3,CV_32FC1);
           
  IplImage* img0 = 0;   
  CvMemStorage* storage = 0;   

  // convert color image img to gray image gray:   
  cvCvtColor(workImage, gray, CV_RGB2GRAY);   
  found = cvFindChessboardCorners(gray,cvSize(6,8),corners,&numCornersFound);
  if(found){
    DEBUG("Found %d\n",numCornersFound);
    cvFindCornerSubPix(gray, corners, numCornersFound, cvSize(5,5), cvSize(-1, -1), cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.1)); 
    cvDrawChessboardCorners(workImage,cvSize(6,8),corners,numCornersFound,found);
    DEBUG("image_points\n");
    //set camera matrix, 2d array of points, ...
    for( int i=0; i<BOARDTOTAL; i++ ) {
        CV_MAT_ELEM(*image_points, float,i,0) = corners[i].x;
        CV_MAT_ELEM(*image_points, float,i,1) = corners[i].y;
    }
    DEBUG("extrinsic params2\n");
    cvFindExtrinsicCameraParams2(object_points,image_points,&Hintrinsic,&Kdistortion,rotation_vector,translation_vector);
    //transform the rotation vector to rotation matrix
    cvRodrigues2(rotation_vector,rotation_matrix);
    DEBUG("copying results\n");
    //get results
    for(int i=0;i<3;i++){
        for(int j=0;j<4;j++){
            if(j<3){
                if(i<3){
                  patternpose[i*4+j] = CV_MAT_ELEM(*rotation_matrix, float, i,j);
                }
            }else{
                if(i<3){
                  patternpose[i*4+j] = CV_MAT_ELEM(*translation_vector, float, i,0);
                  patternpose[i*4+j] = patternpose[i*4+j]/1000.0;
                }
            }
        }
    }
    patternpose[12] = 0;
    patternpose[13] = 0;
    patternpose[14] = 0;
    patternpose[15] = 1;
    ppose->clear();
    for(int i;i<16;i++)
        ppose->push_back(patternpose[i]);
    DEBUG("\npattern:\n");
    DEBUG("%f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n", 
                            patternpose[0],patternpose[1],patternpose[2],patternpose[3],
                            patternpose[4],patternpose[5],patternpose[6],patternpose[7],
                            patternpose[8],patternpose[9],patternpose[10],patternpose[11],
                            patternpose[12],patternpose[13],patternpose[14], patternpose[15]);
  }else{
      DEBUG("Not found");
  }
}

void ImageHandler::refreshImage(){
    workImage = cvQueryFrame(capture);
    if(workImage != (IplImage*) NULL){
        cvShowImage("Object", workImage);
    }else{
        DEBUGE(" query frame failed!");
    }
}
void ImageHandler::work(std::vector<double> *patternpose){
    workImage = cvQueryFrame(capture);
    if(workImage != (IplImage*) NULL){
        findPattern(patternpose);
        cvShowImage("Object", workImage);
    }else{
        DEBUGE(" query frame failed!");
    }
}
