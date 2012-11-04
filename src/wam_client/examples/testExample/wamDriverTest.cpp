#include <unistd.h>
#include "CWamDriver.h"
#include <sys/time.h>
#include <time.h>

//double startPositionC[6] = {0.6, 0.10, 0.4, -0.5, 1.57, 0};
double startPoseC[6] = {0.6, 0.10, 0.4, -0.5, 1.57, 0};
//double myPositionC[6] = {0.6, 0.10, 0.4, -0.5, 1.57, 0};
double myPoseC[16] = {0.0, 1.0, 0.0, 0.45, 
                     1.0, 0.0, 0.0, 0.3,
                     0.0, 0.0, 1.0, 0.1,
                     0.0, 0.0, 0.0, 1.0};

double myJointsC[7] = {0.1, 0.0, 0.1, 2.2, 0.1, 0.0, 0.2};
double myVelsC[7] = {0.2, 0.010, 0.24, -0.25, 1.17, 0.1, -0.2};
double myAccsC[7] = {0.4, 0.11, 0.2, -0.1, 1.5, 0.2, 0.3};

//double apos[6] = {0.7, 0.20, 0.3, -0.5, 1.57, 0};
double aPoseC[16] = {0.0, 1.0, 0.0, 0.45, 
                     1.0, 0.0, 0.0, 0.3,
                     0.0, 0.0, 1.0, 0.1,
                     0.0, 0.0, 0.0, 1.0};

using namespace std;

int main(int argc, char* argv[])
{
    int i;
  
    vector<double> startPosition(startPoseC, startPoseC + sizeof(startPoseC) / sizeof(double) );
//    vector<double> myPosition(myPositionC, myPositionC + sizeof(myPositionC) / sizeof(double) );
    vector<double> myPose(myPoseC, myPoseC + sizeof(myPoseC) / sizeof(double) );
    vector<double> myJoints(myJointsC, myJointsC + sizeof(myJointsC) / sizeof(double) );
    vector<double> myVels(myVelsC, myVelsC + sizeof(myVelsC) / sizeof(double) );
    vector<double> myAccs(myAccsC, myAccsC + sizeof(myAccsC) / sizeof(double) );
    vector<double> aPose(aPoseC, aPoseC + sizeof(aPoseC)/sizeof(double) );
    vector<double> gotPosition(7,0);
    vector<double> angles(7,0);
    vector<double> posrot(6,0);

    try{

        cout << "[APP] Creating Wam..." << endl;
        
//        CWamDriver *arm = new CWamDriver("192.168.100.49",4321,100);
        CWamDriver *arm = new CWamDriver("192.168.100.50",4321,100);
    
        cout << "[APP] Opening Wam..." << endl;
        arm->open();
        cout << "[APP] Wam opened. press shift+idle and press enter." << endl;
        getchar();
    
        arm->create();
        cout << "[APP] Wam created. press shift+activate and press enter." << endl;
        getchar();
        arm->activate();
        
        //you have to compensate gravity!
        arm->setGravityCompensation(1.1);
        cout << "[APP] Going to default position." << endl;
        arm->goToDefaultPosition();
//        gettimeofday(&foo, NULL);
//        printf("%d\n", foo.tv_usec);
//        delta = foo.tv_usec;
        cout << "[APP] Waiting movement termination." << endl;
        arm->waitTillMotionDone();

//        gettimeofday(&foo, NULL);
//        printf("%d\n", foo.tv_usec);
//        delta = foo.tv_usec - delta;
//        cout << "[APP] move done in " << delta << endl;
        cout << "[APP] move done " << endl;
        sleep(2);
        cout << "[APP] arbitrary wait done." << endl;

        for(i=0;i<1;i++){
//            cout << endl << " GetPosition " << endl;
//            arm->getCartesianPose(&posrot);
//            vector<double>::const_iterator cii;
//            for(cii=posrot.begin(); cii!=posrot.end(); cii++){
//                  cout << *cii << " ";
//            }
//            cout << "[APP] Move in cartesian " << i << endl;
//            myPose[11] += 0.05;
//            for(cii=myPose.begin(); cii!=myPose.end(); cii++){
//                  cout << *cii << " ";
//            }
//            arm->moveInCartesian(&myPose);
//            arm->waitTillMotionDone();
//
//            cout << endl << "[APP] Move in Joints " << i << endl;
//            arm->moveInJoints(&errormask, &myJoints,&myVels,&myAccs);
//            cout << endl << "[APP] Checking errors of # " << i << endl;
//            if(errormask > 0x00){
//                arm->errorToString(errormask);
//                //arm->resetmask()?
//                errormask = 0x00;
//                continue;
//            }
            cout << endl << "[APP] Wait trajectory finish " << i << endl;
            arm->waitTillMotionDone();
            cout << endl << " GetAxes " << i << endl;
            arm->getAxes();

            cout << endl << "[APP] Freeing " << i << endl;
            arm->holdCurrentPosition(false);
            sleep(5);
            cout << endl << "[APP] holding " << i << endl;
            arm->holdCurrentPosition(true);
            sleep(5);
            cout << endl << "[APP] Moving " << i << endl;
        }

        
        cout << endl << " GetPosition " << endl;
        arm->getCartesianPose(&posrot);
        vector<double>::const_iterator cii;
        for(cii=posrot.begin(); cii!=posrot.end(); cii++){
              cout << *cii << " ";
        }
        cout << endl << " GetAngles " << endl;
        arm->getJointAngles(&angles);
        for(cii=angles.begin(); cii!=angles.end(); cii++){
              cout << *cii << " ";
        }

        cout << endl << " GetVels " << endl;
        arm->getSpeeds(&angles);
        for(cii=angles.begin(); cii!=angles.end(); cii++){
              cout << *cii << " ";
        }
        cout << endl;
        cout << endl << " GetAccs " << endl;
        arm->getAccelerations(&angles);
        for(cii=angles.begin(); cii!=angles.end(); cii++){
              cout << *cii << " ";
        }
        cout << endl;
       
        cout << "[APP] teach and play session " << endl;
        arm->startMovementsRecording();
        arm->stopMovementsRecording();
        arm->playMovementsRecording();
        
        cout << "[APP] arbitrary wait." << endl;
        sleep(5);
        cout << "[APP] Closing arm" << endl;
        arm->close();
    }catch(CException &e){
        cout << "error:" << e.what() << endl;
    }
    cout << "[APP] Wait until wam gets home, idle the wam and press enter." << endl;
    getchar();
}

