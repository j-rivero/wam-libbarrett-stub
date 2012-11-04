#include "ImageHandler.h"

using namespace std;

ImageHandler::ImageHandler(){

    camera = 0;
    workImage = NULL;
    gray = NULL;

    Htcp2pattern = std::vector<double>(Htcp2patternC, Htcp2patternC + sizeof(Htcp2patternC) / sizeof(double) );
    Hcam2world = std::vector<double>(Hcam2worldC, Hcam2worldC + sizeof(Hcam2worldC) / sizeof(double) );

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

void ImageHandler::mat4x4Mul(std::vector<double> matA, std::vector<double> matB, std::vector<double> *result){
    double resultaux = 0;
    result->clear();
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            resultaux = 0;
            for(int k=0;k<4;k++){
                resultaux += matA.at(i*4+k)*matB.at(k*4+j);
            }
            result->push_back(resultaux);
        }
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

bool ImageHandler::findPattern(std::vector<double>* ppose){
  CvPoint2D32f corners[BOARDTOTAL];
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
  found = cvFindChessboardCorners(gray,cvSize(BOARD_W,BOARD_H),corners,&numCornersFound);
  if(found){
    DEBUG("Found %d\n",numCornersFound);
    cvFindCornerSubPix(gray, corners, numCornersFound, cvSize(5,5), cvSize(-1, -1), cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.1)); 
    cvDrawChessboardCorners(workImage,cvSize(BOARD_W,BOARD_H),corners,numCornersFound,found);
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
    return true;
  }else{
      DEBUG("Not found\n");
      ppose->clear();
      return false;
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

IplImage* ImageHandler::loadImage(int imagenum){

    // load an image  
    char filename[40];
    struct stat st;
    if(stat("images",&st) == 0)
        snprintf(filename,40*sizeof(char),"images/image%d.png",imagenum);
    else
        snprintf(filename,40*sizeof(char),"image%d.png",imagenum);

    workImage = cvLoadImage(filename);
    if(!workImage){
      fprintf(stderr,"Could not load image file: %s\n",filename);
      exit(0);
    }
    return workImage;
}

int ImageHandler::colorSegmentation(IplImage *image){
  int height, width, step, channels;
  uchar *data;
  int i, j, k;
  int threshold1 = 111,threshold2 = 114;
  int detected = 0;
  double percent = 0.0;
      int heightmono, widthmono, stepmono, channelsmono;
      int heighthsv, widthhsv, stephsv, channelshsv;
      uchar *datahsv, *datamono;

      if(image == (IplImage *)NULL)
          image = workImage;
      // get the image data
      height = image->height;
      width = image->width;
      step = image->widthStep;
      channels = image->nChannels;
      data = (uchar *) image->imageData;


      IplImage *colimg = cvCreateImage (cvGetSize (image), 8, 3);
      heighthsv = colimg->height;
      widthhsv = colimg->width;
      stephsv = colimg->widthStep;
      channelshsv = colimg->nChannels;
      datahsv = (uchar *) colimg->imageData;

  IplImage *monoimg = cvCreateImage (cvGetSize (image), 8, 1);
  heightmono = monoimg->height;
  widthmono = monoimg->width;
  stepmono = monoimg->widthStep;
  channelsmono = monoimg->nChannels;
  datamono = (uchar *) monoimg->imageData;

  //HSV segmentation
  cvCvtColor(image,colimg,CV_RGB2HSV);
  
//    cvCreateTrackbar("Threshold1", "resultat", &threshold1, 180, NULL);
//      cvCreateTrackbar("Threshold2", "resultat", &threshold2, 180, NULL);
  for(i=0;i<height;i++){
    for(j=0;j<width;j++){
      if ((datahsv[(i)*stephsv+j*channelshsv]>=threshold1) &&
         (datahsv[(i)*stephsv+j*channelshsv]<=threshold2)) {
       //((uchar *)monoimg->imageData)[i*monoimg->widthStep+j]=255;
         datamono[i*stepmono+j*channelsmono]=255;
         detected++;
       } else {
        datamono[i*stepmono+j*channelsmono]=0;
       }  
    }
  }
  cvShowImage( "original", image );
  cvShowImage( "resultat", monoimg );
  percent = detected*100/(double)(height*width);
  DEBUG(" %d pixels detected (%f)\n",detected,percent);
  // Release the capture device housekeeping
  cvReleaseImage(&monoimg);
  return (int)percent;
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

void ImageHandler::handeyetest(std::vector<double> *wampose){
    std::vector<double> patternpose(16,0);
    workImage = cvQueryFrame(capture);
    if(workImage != (IplImage*) NULL){
        if(findPattern(&patternpose)){
            transform(wampose, patternpose);
        }
        cvShowImage("Object", workImage);
    }else{
        DEBUGE(" query frame failed!");
    }
}

void ImageHandler::transform(std::vector<double> *wampose, std::vector<double> patternpose){

    std::vector<double> poseaux;
    mat4x4Mul(Hcam2world,patternpose,&poseaux);
    mat4x4Mul(poseaux,Htcp2pattern,wampose);

}
