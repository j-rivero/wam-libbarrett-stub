#include <unistd.h>
#include <stdio.h>
#include <ncurses.h>
#include <math.h>
#include <vector>
#include "WamDriver.h"
#include "ImageHandler.h"

#define USEWAM 1
#define IMGCAM 1

enum MOVECODE{
lX,
uX,
lY,
uY,
lZ,
uZ,
lRX,
uRX,
lRY,
uRY,
lRZ,
uRZ,
END,
lSTEP,
uSTEP,
SAVE,
HOLDSWITCH,
UNKNOWN,
CONTINUE
};

double startPoseC[7] = {0.0, 0.0, 0.0, 2.2, 0.0, 0.0, 0.0};
double myPoseC[16] = {1.0, 0.0, 0.0, 0.6, 
                     0.0, 1.0, 0.0, 0.3,
                     0.0, 0.0, 1.0, 0.5,
                     0.0, 0.0, 0.0, 1.0};

double aPoseC[16] = {0.0, 0.0, 1.0, 0.65, 
                           1.0, 0.0, 0.0, 0.0,
                           0.0, 1.0, 0.0, 0.3,
                           0.0, 0.0, 0.0, 1.0};
//not thread safe...
double desiredPoseC[16] = {0.0, 0.0, 1.0, 0.65, 
                           1.0, 0.0, 0.0, 0.0,
                           0.0, 1.0, 0.0, 0.3,
                           0.0, 0.0, 0.0, 1.0};

bool newdata = false;
int movement = UNKNOWN;
double increment = 0.05;
CMutex *myLock;

using namespace std;

void matrixMultiplication(std::vector<double> mat1, std::vector<double> mat2){

}

void rotate(double *apose, double anglex, double angley, double anglez){

    apose[0] = cos(angley);
    apose[1] = -cos(angley)*sin(anglez); 
    apose[2] = sin(angley);
    apose[4] = cos(anglez)*sin(anglex)*sin(angley)+cos(anglex)*sin(anglez);
    apose[5] = -sin(anglez)*sin(anglex)*sin(angley)+cos(anglex)*cos(anglez);
    apose[6] = -sin(anglex)*cos(angley);
    apose[8] = -cos(anglez)*cos(anglex)*sin(angley)+sin(anglex)*sin(anglez); 
    apose[9] = cos(anglex)*sin(angley)*sin(anglez)+sin(anglex)*cos(anglez);
    apose[10] = cos(angley)*cos(anglex); 

}


//obtain rotation angles from pose's rotation matrix
void rotToAngles(std::vector<double> somepose, double anglex, double angley, double anglez){
//TODO


}

void printData(double anglex, double angley, double anglez, double *nominalPose,std::vector<double> myJoints,std::vector<double> aPose,std::vector<double> patternPose){

    mvprintw(5,0," %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f",
                                  nominalPose[0], nominalPose[1], nominalPose[2], nominalPose[3],
                                  nominalPose[4], nominalPose[5], nominalPose[6], nominalPose[7],
                                  nominalPose[8], nominalPose[9], nominalPose[10], nominalPose[11],
                                  nominalPose[12], nominalPose[13], nominalPose[14], nominalPose[15]);
    mvprintw(10,0," %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f",
                                  aPose[0], aPose[1], aPose[2], aPose[3],
                                  aPose[4], aPose[5], aPose[6], aPose[7],
                                  aPose[8], aPose[9], aPose[10], aPose[11],
                                  aPose[12], aPose[13], aPose[14], aPose[15]);
    mvprintw(15,0,"%f %f %f %f %f %f %f",
                                  myJoints[0], myJoints[1], myJoints[2], myJoints[3],
                                  myJoints[4], myJoints[5], myJoints[6]);

    mvprintw(17,0," %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f",
                                  patternPose[0], patternPose[1], patternPose[2], patternPose[3],
                                  patternPose[4], patternPose[5], patternPose[6], patternPose[7],
                                  patternPose[8], patternPose[9], patternPose[10], patternPose[11],
                                  patternPose[12], patternPose[13], patternPose[14], patternPose[15]);
    mvprintw(22,0,"Rot(x): %f Rot(y): %f Rot(z): %f ", anglex, angley, anglez);
}

int saveData(std::vector<double> myAngles, std::vector<double> aPose, std::vector<double> patternPose, int take){
    FILE *fd;
    double Rrot[9];

    memset(Rrot,0,9*sizeof(double));
/*
    snprintf(filename,40*sizeof(char),"images/image%d.coords",takeNumber);

    fprintf(stdout,"Take number %d, opening %s\n",takeNumber,filename);
    if((fd=fopen(filename,"w"))==NULL){
        perror("Error opening position register file");
        return -1;
    }
*/

    if(take == 0){
         if((fd=fopen("coords_list.txt","w"))==NULL){
            perror("Error opening position register file");
            return -1;
        }
        fprintf(fd,"take, J[0], J[1], J[2], J[3], J[4], J[5], J[6], C[0], C[1], C[2], C[3], C[4], C[5], C[6] C[7], C[8], C[9], C[10], C[11], C[12], C[13], C[14], C[15], P[0], P[1], P[2], P[3], P[4], P[5], P[6], P[7], P[8], P[9], P[10], P[11], P[12], P[13, P[14], P[15]\n");
    }else{
        if((fd=fopen("coords_list.txt","a"))==NULL){
            perror("Error opening position register file");
            return -1;
        }
    }
    fprintf(fd,"%d, %f, %f, %f, %f, %f, %f, %f, ",take,myAngles[0],myAngles[1],
                                myAngles[2],myAngles[3],myAngles[4],myAngles[5],myAngles[6]);

    fprintf(fd,"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f,",
                              aPose[0],aPose[1],aPose[2],aPose[3],
                              aPose[4],aPose[5],aPose[6],aPose[7],
                              aPose[8],aPose[9],aPose[10],aPose[11],
                              aPose[12],aPose[13],aPose[14],aPose[15]);

    fprintf(fd,"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                              patternPose[0],patternPose[1],patternPose[2],patternPose[3],
                              patternPose[4],patternPose[5],patternPose[6],patternPose[7],
                              patternPose[8],patternPose[9],patternPose[10],patternPose[11],
                              patternPose[12],patternPose[13],patternPose[14],patternPose[15]);
    fclose(fd);
    return 0;
}

int getMove(){

    char key;
    timeout(1);
    key = getch();
    if(key == ERR){
        return CONTINUE;
    } 
//    mvprintw(3,0,"Read %c\n",key);

    switch(key){
        case 'q':
            return END;
        case 'e':
            return lSTEP;
        case 'r':
            return uSTEP;
        case 's':
            return lX;
        case 'w':
            return uX;
        case 'a':
            return lY;
        case 'd':
            return uY;
        case 'z':
            return lZ;
        case 'x':
            return uZ;
        case 'f':
            return lRX;
        case 'h':
            return uRX;
        case 'g':
            return lRY;
        case 't':
            return uRY;
        case 'v':
            return lRZ;
        case 'b':
            return uRZ;
        case 'p':
            return SAVE;
        case 'o':
            return HOLDSWITCH;
        default:
            return UNKNOWN;
    }
}

void *testgui(void *param){
    int i=0, take=0;
    bool hold = true;
    double radincrement = 0.2;
    double anglex = 0, angley = 0, anglez = 0;
    WamDriver *mywam = (WamDriver *)param;
    vector<double> aPose(aPoseC, aPoseC + sizeof(aPoseC)/sizeof(double) );
    vector<double> patternpose(aPoseC, aPoseC + sizeof(aPoseC)/sizeof(double) );
    vector<double> angles(7,0);

    //video capture
    #ifdef IMGCAM
    ImageHandler *imgHandler = new ImageHandler();
    if(imgHandler->initStream()<0){
        movement = END;
        cerr << "Error opening camera." << endl;
        imgHandler->closeStream();
        pthread_exit(NULL);
    }
    while(1){
        imgHandler->work(&patternpose);
    }
    imgHandler->closeStream();
    #endif
    pthread_exit(NULL);
}
void *gui(void *param){

    int i=0, take=0;
    bool hold = true;
    double radincrement = 0.2;
    double anglex = 1.571, angley = 1.571, anglez = 0;
    WamDriver *mywam = (WamDriver *)param;
    vector<double> aPose(16,0);
    vector<double> patternpose(16,0);
    vector<double> angles(7,0);

    //video capture
    #ifdef IMGCAM
    ImageHandler *imgHandler = new ImageHandler();
    if(imgHandler->initStream()<0){
        movement = END;
        cerr << "Error opening camera." << endl;
        imgHandler->closeStream();
        pthread_exit(NULL);
    }
    #endif
    initscr();

    //raw(); //disables ctrl+c? using cbreak() instead
    cbreak();

    noecho();

    while((movement = getMove()) != END){
       clear();
       mvprintw(0,0,"Move pressing the keys. position keys: ad,sw,zx rotation: fh,gt,vb, step: -e,+r,[o]hold on/off [p]save, [q]uit.");
       mvprintw(3,0,"%f", increment);

       switch(movement){
           case lX:
               mvprintw(2,0,"less X ");
               desiredPoseC[3]-=increment;
               break;
           case uX:
               mvprintw(2,0,"more X ");
               desiredPoseC[3]+=increment;
               break;
           case lY:
               mvprintw(2,0,"less Y ");
               desiredPoseC[7]-=increment;
               break;
           case uY:
               mvprintw(2,0,"more Y ");
               desiredPoseC[7]+=increment;
               break;
           case lZ:
               mvprintw(2,0,"less Z ");
               desiredPoseC[11]-=increment;
               break;
           case uZ:
               mvprintw(2,0,"more Z ");
               desiredPoseC[11]+=increment;
               break;
           case lRX:
               mvprintw(2,0,"less RX");
               anglex -= radincrement;
               rotate(desiredPoseC,anglex,angley,anglez);
               break;
           case uRX:
               mvprintw(2,0,"more RX");
               anglex += radincrement;
               rotate(desiredPoseC,anglex,angley,anglez);
               break;
           case lRY:
               mvprintw(2,0,"less RY");
               angley -= radincrement;
               rotate(desiredPoseC,anglex,angley,anglez);
               break;
           case uRY:
               mvprintw(2,0,"more RY");
               angley += radincrement;
               rotate(desiredPoseC,anglex,angley,anglez);
               break;
           case lRZ:
               mvprintw(2,0,"less RZ");
               anglez -= radincrement;
               rotate(desiredPoseC,anglex,angley,anglez);
               break;
           case uRZ:
               mvprintw(2,0,"more RZ");
               anglez += radincrement;
               rotate(desiredPoseC,anglex,angley,anglez);
               break;
           case uSTEP:
               movement = CONTINUE;
               mvprintw(2,0,"+step  ");
               increment += 0.02;
               continue;
           case lSTEP:
               movement = CONTINUE;
               mvprintw(2,0,"-step  ");
               increment -= 0.02;
               continue;
           case HOLDSWITCH:
               movement = CONTINUE;
                hold = !hold;
                if(hold){
                    mvprintw(2,0,"hold on");
                    memcpy(desiredPoseC,&aPose[0],16*sizeof(double));
                }else{
                    mvprintw(2,0,"holdoff");
                }

               #ifdef USEWAM
                mywam->holdCurrentPosition(hold);
               #endif
                continue;
           case SAVE:
               movement = CONTINUE;
               mvprintw(2,0,"save   ");
               //saveDATA
               saveData(angles, aPose, patternpose,take);
                take++;
               continue;
           case CONTINUE:
               movement = CONTINUE;
               break;
           default:
               mvprintw(2,0,"unknown");
               movement = CONTINUE;
               break;
       }
        if(movement != CONTINUE){
            myLock->enter();
            mvprintw(4,0,"new data");
            newdata = true;
            myLock->exit();
        }else{
            mvprintw(4,0,"skip     ");
        }
#ifdef USEWAM
        mywam->getCartesianPose(&aPose);
        mywam->getJointAngles(&angles);
#endif
        #ifdef IMGCAM
        imgHandler->work(&patternpose);
        #endif
        printData(anglex, angley, anglez, desiredPoseC, angles, aPose, patternpose);
        refresh();
    }

    #ifdef IMGCAM
    imgHandler->closeStream();
    #endif
    endwin();
    system("/bin/sh");
    pthread_exit(NULL);
}


int main(int argc, char* argv[])
{
    uint16_t errormask = 0x00;
    CThread *displayThread = NULL;
    myLock = new CMutex();
    
    newdata = false;

    vector<double> myPose(myPoseC, myPoseC + sizeof(myPoseC) / sizeof(double) );
    vector<double> startPosition(startPoseC, startPoseC + sizeof(startPoseC) / sizeof(double) );
    vector<double> gotPosition(7,0);
    vector<double> posrot(6,0);

    try{


#ifdef USEWAM
        WamDriver *arm = new WamDriver("147.83.76.250",4321,100);
    
        arm->open();
        cout << "[APP] Wam opened. press shift+idle and press enter." << endl;
        getchar();
    
        arm->create();
        cout << "[APP] Wam created. press shift+activate and press enter." << endl;
        getchar();
        arm->activate();
        //you have to compensate gravity!
        arm->setGravityCompensation(1.03);
        cout << "[APP] Going to default position. " << startPosition.size() << endl;
        arm->goToDefaultPosition();
        cout << "[APP] Waiting movement termination." << endl;
        arm->waitTillMotionDone();
        /** application start */
        displayThread = new CThread("socket service thread");
        displayThread->attach(gui,arm); 
        displayThread->start();

        while(movement != END){
                if(newdata){
                    myLock->enter();
                    newdata = false;
                    myLock->exit();
                    myPose.clear();
                    //TODO properly protect desiredPoseC by mutex
                    for(int i=0;i<16;i++)
                        myPose.push_back(desiredPoseC[i]);
                    arm->moveInCartesian(&myPose);
                }else{
                    usleep(100000);
                }
        }
        cout << "[APP] Closing arm" << endl;
        arm->close();
#else

        displayThread = new CThread("socket service thread");
        displayThread->attach(gui,NULL); 
        displayThread->start();

        while(movement != END){
                if(newdata){
                    myLock->enter();
                    newdata = false;
                    myLock->exit();
                }else{
                    usleep(500000);
                }
        }
        cout << "[APP] Closing arm" << endl;
#endif
        if(displayThread != (CThread *)NULL){
            displayThread->end();
        }
    }catch(CException& e){
        cout << e.what() << endl;
    }
}

//quich video test
int main2(int argc, char **argv){

    vector<double> patternpose(aPoseC, aPoseC + sizeof(aPoseC)/sizeof(double) );
    ImageHandler *imgHandler = new ImageHandler();
    if(imgHandler->initStream()<0){
        movement = END;
        cerr << "Error opening camera." << endl;
        imgHandler->closeStream();
        pthread_exit(NULL);
    }
    while(1){
        imgHandler->work(&patternpose);
    }
}

