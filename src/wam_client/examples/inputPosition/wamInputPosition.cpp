#include <unistd.h>
#include <vector>
#include <iostream>

#include "CWamDriver.h"
#include <sys/time.h>
#include <time.h>
#include <cstdlib> //string conversions
using namespace std;

//double startPositionC[6] = {0.6, 0.10, 0.4, -0.5, 1.57, 0};
double startPoseC[6] = {0.6, 0.10, 0.4, -0.5, 1.57, 0};
double myJointsC[7] = {-0.014865, 0.594804, -0.0918629, 1.80294, 0.124191, 0.235841, 1.30887};
double myVelsC[7] = {0.2, 0.010, 0.24, -0.25, 1.17, 0.1, -0.2};
double myAccsC[7] = {0.4, 0.11, 0.2, -0.1, 1.5, 0.2, 0.3};

/*double myPoseC[16] = {0.0, 1.0, 0.0, 0.45, 
                     1.0, 0.0, 0.0, 0.3,
                     0.0, 0.0, 1.0, 0.1,
                     0.0, 0.0, 0.0, 1.0};
*/
double myPoseC[16] = { -0.029469, 0.790274, 0.612045, 0.656292,
 0.998061, 0.056845, -0.025342, -0.050075,
 -0.054819, 0.610111, -0.790417, 0.104247,
 0.0, 0.0, 0.0, 1.0};

   using namespace std;

int main(int argc, char* argv[])
{
  uint16_t errormask = 0x00;
  
  cin.exceptions(std::ios_base::badbit   |
  std::ios_base::failbit);  
  //   uint16_t errormask = 0x00;
  vector<double>::const_iterator cii;//iterator for the current pose
  bool moveDone;
  int i;
  int delta;
  struct timeval foo;
  bool program_end = false;
  vector<double> startPosition(startPoseC, startPoseC + sizeof(startPoseC) / sizeof(double) );
  vector<double> myJoints(myJointsC, myJointsC + sizeof(myJointsC) / sizeof(double) );  vector<double> myPose(myPoseC, myPoseC + sizeof(myPoseC) / sizeof(double) );
  vector<double> myVels(myVelsC, myVelsC + sizeof(myVelsC) / sizeof(double) );
  vector<double> myAccs(myAccsC, myAccsC + sizeof(myAccsC) / sizeof(double) );
  vector<double> defaultPose=myPose;
  vector<double> gotPosition(7,0);
  vector<double> angles(7,0);
  vector<double> posrot(6,0);
  
  try{
    
    cout << "[APP] Creating Wam..." << endl;
    CWamDriver *arm = new CWamDriver("147.83.76.250",4321,100);
    
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
    gettimeofday(&foo, NULL);
    printf("%d\n", foo.tv_usec);
    delta = foo.tv_usec;
    cout << "[APP] Waiting movement termination." << endl;
    arm->waitTillMotionDone();
    gettimeofday(&foo, NULL);
    printf("%d\n", foo.tv_usec);
    delta = foo.tv_usec - delta;
    cout << "[APP] move done in " << delta << endl;
    sleep(5);
    cout << "[APP] arbitrary wait done." << endl;
//move to OUR initial position
    cout << "[APP] Move in cartesian\n";            
    arm->moveInCartesian(&myPose);
    cout << endl << "[APP] Wait trajectory finish\n";
    arm->waitTillMotionDone();
    
    cout << endl << "Current position (GetPosition) " << endl;
    arm->getCartesianPose(&posrot);
    
    int pos=0;
    for(cii=posrot.begin(); cii!=posrot.end(); cii++){
      cout << *cii << " ";
      if (++pos==4) {
        cout<<endl;
        pos=0;
      }
    }
    
    char command;
    
    //wait for a command
    float fl;
    char temp[100];

    while (!program_end) {
 //     try {
	cout<<"Waiting:\n";
        cout<<"m:introduce new pose;\n";
        cout<<"e:execute pose;\n";
        cout<<"p:get current position;\n"; 
        cout<<"d:go to default pose;\n";
        cout<<"q:quit\n";
	cin>>command;
	switch (command) {
	  case 'm':cout<<"[APP] New motion";
	  cout<<"Enter 4x4 new pose matrix row by row (press enter at each row)\n";
	  cout<<"BE CAREFUL: not a very strict control on float conversion is done...\n";
	  cin>>myPose[0]>>myPose[1]>>myPose[2]>>myPose[3];cin.clear();
	  cout<<myPose[0]<<" "<<myPose[1]<<" "<<myPose[2]<<" "<<myPose[3]<<endl;
	  cin>>myPose[4]>>myPose[5]>>myPose[6]>>myPose[7];cin.clear();
	  cout<<myPose[4]<<" "<<myPose[5]<<" "<<myPose[6]<<" "<<myPose[7]<<endl;
	  cin>>myPose[8]>>myPose[9]>>myPose[10]>>myPose[11];cin.clear();
	  cout<<myPose[8]<<" "<<myPose[9]<<" "<<myPose[10]<<" "<<myPose[11]<<endl;
	  cin>>myPose[12]>>myPose[13]>>myPose[14]>>myPose[15];cin.clear();
	  cout<<myPose[12]<<" "<<myPose[13]<<" "<<myPose[14]<<" "<<myPose[15]<<endl;
	  break;
	  case 'e':cout<<"[APP] Go to next pose? (Y,n)\n";
	  cout<<myPose[0]<<" "<<myPose[1]<<" "<<myPose[2]<<" "<<myPose[3]<<endl;
	  cout<<myPose[4]<<" "<<myPose[5]<<" "<<myPose[6]<<" "<<myPose[7]<<endl;
	  cout<<myPose[8]<<" "<<myPose[9]<<" "<<myPose[10]<<" "<<myPose[11]<<endl;
	  cout<<myPose[12]<<" "<<myPose[13]<<" "<<myPose[14]<<" "<<myPose[15]<<endl;
	  cin.clear();
	  cin>>command;
	  if (command='Y') {
	    cout << "[APP] Move in cartesian\n";            
	    arm->moveInCartesian(&myPose);
	    cout << endl << "[APP] Wait trajectory finish\n";
	    arm->waitTillMotionDone();
	  } else
	    cout <<"[APP] Aborted\n";
	  break;
	  case 'p':cout<<"[APP] Current position\n";
 	  arm->getCartesianPose(&posrot);
          
          pos=0;
	  for(cii=posrot.begin(); cii!=posrot.end(); cii++){
	    cout << *cii << " ";
            if (++pos==4) {
              cout<<endl;
              pos=0;
            }
          }        
          
          cout << endl << " GetAngles " << endl;
          arm->getJointAngles(&angles);
          for(cii=angles.begin(); cii!=angles.end(); cii++){
            cout << *cii << " ";
          }
	  break;
          case 'd':cout<<"Going to default position\n";
          //arm->moveInCartesian(&defaultPose);
          //cout << endl << "[APP] Wait trajectory finish\n";
          //arm->waitTillMotionDone();
          cout << endl << "[APP] Move in Joints " << i << endl;
          arm->moveInJoints(&errormask, &myJoints,&myVels,&myAccs);
          cout << endl << "[APP] Checking errors of # " << i << endl;
          if(errormask > 0x00){
            arm->errorToString(errormask);
                //arm->resetmask()?
            errormask = 0x00;
            continue;
          }
          cout << endl << "[APP] Wait trajectory finish " << i << endl;
          arm->waitTillMotionDone();
          cout << endl << " GetAxes " << i << endl;
          arm->getAxes();
          
          break;
	  case 'q':cout<<"Quit\n";
	  program_end=true;
	  break;
	}
	cin.clear();
   //   } catch (ios_base::failure& e) {
//	cout << "error:"<<e.what()  << endl;
  //    }
      //	  cout<<fl<<"\n";
    }        
    cout << "[APP] Closing arm" << endl;
    arm->close();
  }catch(CException &e){
    cout << "error:" << e.what() << endl;
  } 
}

