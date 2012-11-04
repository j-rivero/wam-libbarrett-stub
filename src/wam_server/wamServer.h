#ifndef WAMSERVER_H
#define WAMSERVER_H
/*
 * wamServer.h
 */

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>
#include <errno.h>
#include <sys/wait.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <signal.h>
#include <sys/types.h>

#include <algorithm> //per poder copiar containers
#include <vector> 
#include <string>
#include <string.h>
#include <time.h>

#include "socket.h"
#include "socketserver.h"
//#include "socketclient.h"
#include "commexceptions.h"
#include "wamServerExceptions.h"

#include "constants.h"
#include "protocol.h"

#include "mutex.h"
#include "thread.h"

//WAM native PC
//#include "waminterface/wamif.h"

//libbarrett includes
#include <barrett/exception.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems/wam.h>

#ifndef BARRETT_SMF_WAM_CONFIG_PATH
#  define BARRETT_SMF_WAM_CONFIG_PATH NULL
#endif
//libbarrett end

/** the port that will be used for users' connections
 * @todo get port number from config file and use this one as default
 */
#define DEFAULTPORT 4321

#define DEFAULT_SERVER_IP "127.0.0.1"

#define DEFAULTSTATERATE 100000
//#define DEFAULTSTATERATE 500000
#define MOVEDONERATE 10000

//ADDRESS: the address that will be used for users' connections
//#define ADDRESS "147.83.76.42"

/// number of pending connections on the listen socket
#define MAXCONN 5

///socket buffer size
#define BUFFERSIZE 512
#define DATASIZE 512

//expected length of pose vector (depends on whether we're using 4x4 matrices) or position + rotation angles. 
#define POSESIZE 16

//WamInterface *wamif = NULL;

enum EVENTNUMBERING
{
    NEWCONNECTION,
    DISCONNECTION,
    RXDATA
};

/// Configuration structure
struct WAMconfig
{
    /// Listening port for the WamServer
    char listenPort[10];
};

struct WAMPacketHeader
{
    ///opcionalment
    int packetid;
    //int senderid;

    int mainkind;
    int subkind;
    int datalength;
};

/**
 * Per a escriure/llegir length denota la mida de les dades
 * en bytes
 */
struct WAMPacket
{    
    WAMPacketHeader header;
    //unsigned char *data; //puc especificar una mida variable d'alguna altra manera?
    std::vector<double> data;
};

BARRETT_UNITS_TYPEDEFS(7);

/**
 * \brief Driver class for the Whole-Arm Manipulator.
 *
 * provides basic set of networked functions for the WAM.
 */
class WamServer {

    private:

        //libbarret objects
        barrett::ProductManager pm_;
        barrett::systems::Wam<7>* local_wam_;

        unsigned int port_;
        int lastasyncid_;
        int stateRefreshRate_;
        std::string server_ip_;
        int statemachine_;
        bool connected_;
        bool shutdown_;
        bool withStateThread_;
        bool withTorqueThread_;

        std::string laststrcmd_;
        TClient_info *name_client_;

        CSocketServer *ssocket_;

        /// list of internal events
        std::list<std::string> events_;
        CEventServer *eventserver_;

        CMutex mutex_connected_;
        CMutex mutex_socket_;
        CThread *stateThread_; 
        CThread *torqueNotifierThread_; 
        CThread *moveDoneThread_;
       
        void printPacket(WAMPacket *packet);

        void startMoveDoneThread();
        void startStateThread();
        void startTorqueThread();

        /**
         *  sets gravity to specified value
         *  @param gravity scale-up factor. 1 for calibrated value
         *  0 to disable gravity compensation. (Processed as a different
         *  packet)
         */
        void setGravityCompensation(double gravity);

        /**
         * gets cartesian coordinates
         */
        void getCartesian(double pos[3],double rot[3]);

        /**
         * Given an string, builds the appropiate command packet.
         * Syntax:
         * @TODO Syntax of the command packet string 
         */
        int parse_input(char *buffer);

         /** \brief Performs a move in cartesian coordinates.
         *   With the cartesian coordinates and the orientation angles
         *   provided it will send the data to the wam and a move
         *   will be performed. It asks for command acknowledgment but
         *   it's a non-blocking function.
         *   
         *   @param double pos coordinates x,y,z in double format
         *   @param double rot 3 euler angles double vector
         *   @see waitUntilTrajDone If you'd want to wait for the move to
         *   finish and thus blocking.
         *   @see check_traj_done If you want to know if the move has been 
         *   done but you don't want to block if not
         */
        int cartesianMove(double pos[3], double rot[3]);

    public:
        /**
         *  Constructor
         *  @param port defines the port the driver will listen to.
         *  @param stateRefreshRate sets the refresh rate of the wam's state
         *  in microseconds.
         */ 
        WamServer(const std::string serverip = "127.0.0.1", unsigned int port = DEFAULTPORT, int stateRefreshRate = DEFAULTSTATERATE, bool stateThread = true, bool torqueThread = false);

        /// Default destructor
        ~WamServer();

        void disconnect();

        bool isConnected();
        bool isClosing();
        void statePublisher();
        void torqueFaultsNotifier();

        void getState(WAMPacket *packet);

        void moveDone();

        /**
        * \brief Initialization function for wam. 
        *
        * It opens a socket and waits for wam connection.
        * Calls config method (which is void so far)
        * Finally, it turns gravity on.
        *
        * @todo Use config strings.
        * @param config Sets the listening port number. 
        */
        void open();

        void recovery();
        void waitConnection();
        void create();
        void destroy();
    
        /**
        * Sends the wam home, shutdown's it and closes the connection.
        */
        void close();   

        void readPacket(WAMPacket *packet);

        void packetInterpreter(WAMPacket *packet);

         ///////////////////////////////
         // Position Control  methods //
         ///////////////////////////////
       
        /**
         * \brief returns the number of Axes present, usually seven.
         * In cartesian, that's actually 3 for position and 3 for
         * rotation in euler angles. 
         * @return the number of axes
         */
        unsigned int getAxes();

        // Operational modes
        /**
         * \brief switch between joints positions and cartesian positions
         * mode
         */
//        void setJointSpace();

        /**
         * \brief switch between joints positions and cartesian positions
         * mode
         */
//        void setCartesianSpace();

        /**
         * Constraints/unconstraints the WAM to the current position.
         * The WAM is left to gravity compensation if turned off.
         */
        void holdCurrentPosition(bool on);

        /**
         * \brief moves the selected joint to angle ref
         *
         * @param j number of the joint to move
         * @param ref angle to go to in radians
         */
        void jointMove(int j, double ref);
   
        /**
         * \brief moves in joints
         * @param refs pointer to a vector with the numAxes joint angles
         */
        void jointMove(const double *refs);

        /**
        * \brief Fetches the position and the rotation matrix from ref
        * and performs a cartesian move
        *
        * The movement data has to be properly stored on the refs pointer,
        * otherwise operation will end up on segmentation fault. The format
        * is 6 double values, 3 for position and 3 for rotation matrix.
        *
        * @param posrot - pointer to the array of coordinates+rotations
        */
        void cartesianMove(const double *posrot);

        /**
         * \brief Performs an incremental move to the selected joint
         *
         * @param j Selected joint to move
         * @param delta increment angle in radians to move
         */
        void relativeMove(int j, double delta);

        /**
         * \brief Performs an incremental move to all joints, specifying
         * the value of each joint increment in a 7-element array.
         *
         * @param deltas Array of increments, one for each joint, in radians
         */
        void relativeMove(const double *deltas);

        /**
         * \brief Performs an incremental move  in cartesian coordinates
         * Retrieves the current position, adds the specified values and
         * sends the move.
         *
         * @param deltas Array of increments, xyz and euler angles
         */
        void cartesianRelativeMove(const double *deltas);

       /**
        * \brief Checks whether wam has finished its move. 
        * Blocking method that returns when the wam 
        * is idle waiting for commands or waits if it's moving
        * @deprecated per event
        */
        //void waitTillMotionDone();

       /**
        * \brief Checks whether wam has finished its move. 
        * Non-Blocking method that returns true when the wam 
        * is idle waiting for commands.
        * @return true if wam's idle, false otherwise
        * @deprecated per event
        */
        //bool checkMotionDone();

        /**
         * \brief Sets the desired speed for joint j
         */
        void setRefSpeed(int j, const double speed);

        /**
         * \brief Sets the desired speed of each joint
         */
        void setRefSpeeds(const double *spds);

        /**
         * \brief Sets the desired speed of each joint
         */
        //void setRefSpeeds(double speed);

        /** 
         * \brief Sets the desired acceleration for joint j
         * @param joint target joint
         * @param acceleration desired acceleration in m/s^2
         */
        void setRefAcceleration(int j, double acceleration);

        /** 
         * \brief Sets the desired acceleration for each joint 
         * @param acceleration numAxes vector desired acceleration
         * in m/s^2
         */
        void setRefAccelerations(const double *accs);

        /** 
         * \brief Sets the desired acceleration for all joints 
         * @param acceleration Acceleration value for all joints
         */
        //void setRefAccelerations(const double acceleration);


        /** 
         * \brief 
         * @todo not yet implemented
         */
        void stop(int j);

        /** 
         * \brief 
         * @todo not yet implemented
         */
        void stop();
      
         ////////////////////////////////
         //      Encoder  methods      //
         ////////////////////////////////
       
        /** 
         * \brief Returns the value of the angle for the specified joint
         *
         */
        double getJointAngle(int j);
       
        /**
        * \brief Returns the joint angles on a 7-element vector
        *
        *  @param encs array with the number of axes (tipically 7) where
        *  data will be stored.
        *  @see src/test/wamDriverTest.cpp
        */
        void getJointsAngles(double *encs);

        /**
         * \brief Stores the position and rotation in cartesian coordinates
         * on a 6-element vector
         *
         * The format will be (x,y,z,Rot(z),Rot(y),Rot(x))
         *
         * @param posrot vector where data will be stored
         *
         */
        void getCartesianPosition(double *posrot);

        /** 
         * \brief gets the speed of the joint j 
         */
        double getSpeed(int j);

        /** 
         * \brief gets the speed of each joint
         * @param spds Speeds vector in m/s
         */
        void getSpeeds(double *spds);

        /** 
         * \brief gets the current acceleration of joint j 
         */
        double getAcceleration(int j);

        /** 
         * \brief gets the accelerations for each joint
         * @param accs Acceleration vector for each joint 
         */
        void getAccelerations(double *accs);

        /**************************************/
        /*      WAM specific functions        */
        /**************************************/

        double setTorqueLimit();


        /**
         * \brief Gets the torque force on the specified joint
         * @todo not yet implemented
         * @param torque Double where the torque force will be stored.
         * @param j the selected joint to evaluate
         */
        double getTorque(int j);

        /**
         * \brief Gets the torque force on each joint
         * @param torques An array of doubles where the torque of each
         * joint will be stored.
         */
        void getTorques(double *torques);

        /************************************/
        /*  Convinient operational methods  */
        /************************************/
        
        /**
         * Moves WAM to the home position
         */
        void moveHome();

        /**
         * generic move, in joints
         */
        void move(/**mask,**/ double angles[7], double vec[7],double acc[7]);

        /**
         * velocity control move, in joints
         */
        void move(double vec[7], double accs[7]);

        /**
         * generic move, in cartesian
         */
        void move(double pos[3], double vel, double acc);

        /**
         * velocity control, in cartesian
         */
        void move(double vel[3], double acc);
        
        /**
         * \brief Performs a blocking move
         *
         * Sends the move command and waits for movement
         * termination.
         */
        void moveAndWait(const double *posrot);

        /**
         * Records a set of moves ina teach&play session
         */
        void startMovementsRecording();

        /**
         * Stops teach&play session
         */
        void stopMovementsRecording();


        /**
        * Deprecated
        */
        void getEncoders(double *encs);
        void waitTillMotionDone();
};

#endif

