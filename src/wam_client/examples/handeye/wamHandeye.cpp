#include <unistd.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <assert.h>
#include "CWamDriver.h"
#include "ImageHandler.h"

#define FILENAME "posesList.txt"

#define USEWAM 1
#define IMGCAM 1

enum KEYS{
    END,
    SAVE,
    UNKNOWN
};

double startPoseC[6] = {0.6, 0.10, 0.4, -0.5, 1.57, 0};
double myPoseC[16] = {1.0, 0.0, 0.0, 0.5, 
                     0.0, 1.0, 0.0, 0.0,
                     0.0, 0.0, 1.0, 0.0,
                     0.0, 0.0, 0.0, 1.0};

double nextPoseC[16] = {1.0, 0.0, 0.0, 0.6, 
                     0.0, 1.0, 0.0, 0.3,
                     0.0, 0.0, 1.0, 0.5,
                     0.0, 0.0, 0.0, 1.0};
double myJointsC[7] = {0.6, 0.10, 0.4, -0.5, 1.57, 0, 0.4};

double aPoseC[16] = {0.0, 0.0, 1.0, 0.65, 
                           1.0, 0.0, 0.0, 0.0,
                           0.0, 1.0, 0.0, 0.3,
                           0.0, 0.0, 0.0, 1.0};
//not thread safe...
double desiredPoseC[16] = {0.0, 0.0, 1.0, 0.65, 
                           1.0, 0.0, 0.0, 0.0,
                           0.0, 1.0, 0.0, 0.3,
                           0.0, 0.0, 0.0, 1.0};

using namespace std;

void readNextPose(fstream *fin, std::vector<double> *myPose){

  double value;
  int size = 0;

  myPose->clear();
  //readline
  while(myPose->size() < 16 && *fin >> value){
      //fill myPose vector
      myPose->push_back(value); 
  }

  printf("%f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n",
                        myPose->at(0), myPose->at(1), myPose->at(2), myPose->at(3),
                        myPose->at(4), myPose->at(5), myPose->at(6), myPose->at(7),
                        myPose->at(8), myPose->at(9), myPose->at(10), myPose->at(11),
                        myPose->at(12),myPose->at(13),myPose->at(14), myPose->at(15));

}

void printData(double anglex, double angley, double anglez, double *nominalPose,std::vector<double> myJoints,std::vector<double> aPose,std::vector<double> patternPose){

    printf(" %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f",
                                  nominalPose[0], nominalPose[1], nominalPose[2], nominalPose[3],
                                  nominalPose[4], nominalPose[5], nominalPose[6], nominalPose[7],
                                  nominalPose[8], nominalPose[9], nominalPose[10], nominalPose[11],
                                  nominalPose[12], nominalPose[13], nominalPose[14], nominalPose[15]);
    printf(" %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f",
                          aPose[0], aPose[1], aPose[2], aPose[3],
                          aPose[4], aPose[5], aPose[6], aPose[7],
                          aPose[8], aPose[9], aPose[10], aPose[11],
                          aPose[12], aPose[13], aPose[14], aPose[15]);
    printf("%f %f %f %f %f %f %f",
                          myJoints[0], myJoints[1], myJoints[2], myJoints[3],
                          myJoints[4], myJoints[5], myJoints[6]);

    printf(" %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f",
                          patternPose[0], patternPose[1], patternPose[2], patternPose[3],
                          patternPose[4], patternPose[5], patternPose[6], patternPose[7],
                          patternPose[8], patternPose[9], patternPose[10], patternPose[11],
                          patternPose[12], patternPose[13], patternPose[14], patternPose[15]);
    printf("Rot(x): %f Rot(y): %f Rot(z): %f ", anglex, angley, anglez);
}

int saveData(std::vector<double> aPose, int take){
    FILE *fd;
    char filename[256];

    snprintf(filename,40*sizeof(char),"images/image%d.coords",take);
    fprintf(stdout,"Take number %d, opening %s\n",take,filename);
    if((fd=fopen(filename,"w"))==NULL){
        perror("Error opening position register file");
        return -1;
    }

    fprintf(fd,"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
                              aPose[0],aPose[1],aPose[2],aPose[3],
                              aPose[4],aPose[5],aPose[6],aPose[7],
                              aPose[8],aPose[9],aPose[10],aPose[11],
                              aPose[12],aPose[13],aPose[14],aPose[15]);
    fclose(fd);

    if(take == 0){
         if((fd=fopen("coords_list.txt","w"))==(FILE *)NULL){
            perror("Error opening position register file");
            return -1;
        }
    }else{
        if((fd=fopen("coords_list.txt","a"))==(FILE *)NULL){
            perror("Error opening position register file");
            return -1;
        }
    }
    fprintf(fd,"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                              aPose[0],aPose[1],aPose[2],aPose[3],
                              aPose[4],aPose[5],aPose[6],aPose[7],
                              aPose[8],aPose[9],aPose[10],aPose[11],
                              aPose[12],aPose[13],aPose[14],aPose[15]);

    fclose(fd);
    return 0;
}

int getKey(){

    char key;
    key = getchar();

    switch(key){
        case 'q':
            return END;
       case 's':
            return SAVE;
        default:
            return UNKNOWN;
    }
}

int main(int argc, char* argv[])
{
    uint16_t errormask = 0x00;
    int take=0;
    int i = 0;

    vector<double> myPose(myPoseC, myPoseC + sizeof(myPoseC) / sizeof(double) );
    vector<double> startPosition(startPoseC, startPoseC + sizeof(startPoseC) / sizeof(double) );
    vector<double> nextPose(myPoseC, myPoseC + sizeof(myPoseC) / sizeof(double) );

    fstream posesList(FILENAME,fstream::in);
    assert(!posesList.fail());     
    ios_base::iostate oldIOState = posesList.exceptions();
    posesList.exceptions(~ios::goodbit);  // turn on exceptions

    try{

#ifdef USEWAM
        CWamDriver *arm = new CWamDriver("147.83.76.250",4321,100);
    
        arm->open();
        cout << "[APP] Wam opened. press shift+idle and press enter." << endl;
        getchar();
    
        arm->create();
        cout << "[APP] Wam created. press shift+activate and press enter." << endl;
        getchar();
        arm->activate();
        //you have to compensate gravity!
        arm->setGravityCompensation(1.2);
        cout << "[APP] Going to default position." << endl;
        arm->goToDefaultPosition();
        cout << "[APP] Waiting movement termination." << endl;
        arm->waitTillMotionDone();
#endif
#ifdef IMGCAM
        ImageHandler *imgHandler = new ImageHandler();
        if(imgHandler->initStream()<0){
            cerr << "Error opening camera." << endl;
            imgHandler->closeStream();
            exit(EXIT_FAILURE);
        }
        imgHandler->refreshImage();
#endif
        try{
           while( true ){
               //set of calibration moves (read from file?)
               cout << "Pose "<< i++ << endl;
               readNextPose(&posesList, &nextPose);
               //move
#ifdef USEWAM
               arm->moveInCartesian(&nextPose);
               cout << "waiting move." << endl;
               arm->waitTillMotionDone();
               //save image and data
               sleep(4);
               arm->getCartesianPose(&myPose);
#endif
#ifdef IMGCAM
               imgHandler->refreshImage();
               imgHandler->refreshImage();
               imgHandler->saveImage(take);
               printf("reached:\n%f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n",
                        myPose.at(0), myPose.at(1), myPose.at(2), myPose.at(3),
                        myPose.at(4), myPose.at(5), myPose.at(6), myPose.at(7),
                        myPose.at(8), myPose.at(9), myPose.at(10), myPose.at(11),
                        myPose.at(12),myPose.at(13),myPose.at(14), myPose.at(15));
               saveData(myPose, take);
               sleep(4);
#endif
               take++;
           }
        } catch(ios_base::failure failure) {
            if (posesList.bad()) {
              cout << "posesList bad" << endl;
            }else if (posesList.eof()) {
              cout << "posesList hit eof" << endl;
            }else if (posesList.fail()) {
              cout << "posesList failed" << endl;
            }
        }
        posesList.exceptions(oldIOState);

        posesList.close();
#ifdef USEWAM
        cout << "[APP] Closing arm" << endl;
        arm->close();
        cout << "[APP] Press enter when done" << endl;
        getchar();
#endif
#ifdef IMGCAM
        imgHandler->closeStream();
#endif
    }catch(CException& e){
        cout << e.what() << endl;
    }
}
