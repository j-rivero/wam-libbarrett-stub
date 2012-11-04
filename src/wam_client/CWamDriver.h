#ifndef wamdriver_H
#define wamdriver_H
/*
 * wamdriver.h
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
#include <sys/socket.h>

#include <algorithm> //per poder copiar containers
#include <vector>
#include <queue>

//errors uint
#include <stdint.h>

#include "socket.h"
//#include "socketserver.h"
#include "socketclient.h"

#include "constants.h"
#include "protocol.h"

#include "mutex.h"
#include "thread.h"

#include "CWamDriver_exceptions.h"
#include "eventexceptions.h"

/** the port that will be used for users' connections
 * @todo get port number from config file and use this one as default
 */
#define DEFAULTPORT 4321

#define DEFAULTSTATERATE 20

#define DATARX "datarx"
//app events
#define TRAJDONE "trajDone"
#define TORQUEFAULT "torqueFault"

#define ERROR_POS 0
#define RX_ID_POS 1
//ADDRESS: the address that will be used for users' connections
//#define ADDRESS "147.83.76.42"

/// number of pending connections on the listen socket
#define MAXCONN 5

///socket buffer size
#define BUFFERSIZE 512
#define DATASIZE 512

/// Configuration structure
struct WAMconfig {
    /// Listening port for the CWamDriver
    char listenPort[10];
};

/**
* \brief Header structure of the WAMdriver - WAM link
*/
struct WAMPacketHeader {
    ///id of the packet refreshed by any write operation
    int packetid;

    ///id of the appropiate branch of tyes of message
    int mainkind;
    ///high-level string description of the message
    int subkind;
    ///number of bytes present in the data
    int datalength;
};

/**
 * Per a escriure/llegir length denota la mida de les dades
 * en bytes
 */
struct WAMPacket {    
    /// header of the packet
    WAMPacketHeader header;
    ///Data of the packet, the data vector memory is written as unsigned bytesto the socket
    std::vector<double> data; //puc especificar una mida variable d'alguna altra manera?
};

/**
* Structure holding the most recent WAM state.
* The refreshed parameters can be re-configured in 
* realtime.
*/
struct WAMState {
    /// moviment status of the WAM arm
    //bool idle;

    /// Timestamp for logging purposes
    //struct timeval timestamp;

//remember:
//gettimeofday(&detail_time,NULL);
//printf("%d %d",
//detail_time.tv_usec /1000,  /* milliseconds */
//detail_time.tv_usec); /* microseconds */
    bool moving;

    ///joint angles
    std::vector<double> angles;
    ///number of coordinates of the pose 3 xyz and 3 rotation
    static const int numcoordinates = 16;
    ///position of the wam in world coordinates and meters
    std::vector<double> pos;

    ///rotation of the WAM as a 3x3 matrix of rotation (or euler angles?)
    std::vector<double> rot;
    ///WAM's joint velocity
    std::vector<double> vels;
    ///WAM's joint acceleration
    std::vector<double> accs;
};

/**
 * \brief Driver class for the Whole-Arm Manipulator.
 *
 * provides basic set of networked functions for the WAM.
 */
class CWamDriver {

    private:
        std::string serverip;
        int port;
        ///mutex protecting writes on the socket
        CMutex socketmutex;

        ///mutex protecting writes on the socket
        CMutex statemutex;

        ///mutex protecting writes on the socket
        CMutex servingMoveMutex;

        ///State structure published by the WAM periodically
        WAMState state;

        ///Socket server that accepts the WAM connection only
        CSocketClient *csocket;
        
        ///Eventserver (singleton) 
        CEventServer *eventserver;

        ///Thread reading and enqueuing received packets
        CThread *servicethread;
       
        ///client name string identifier on the eventserver
        //std::string name_client;

        ///Auxiliary packet (not thread safe)
        WAMPacket appPacket;

        ///last packet sent id used for refreshing
        int lastid;
 
        /**Received packets (non-state mainkind) queue,
        * Only FIFO accesses allowed
        */
        std::queue<WAMPacket> packetqueue;

        /// list of internal events
        std::list<std::string> events;

        /// Auxiliary list of events for application requests
        std::list<std::string> appevents;

        /** Boolean which denotes if the WAM is connected to
            the driver or not */
        bool connected;

        /** boolean denoting whether a movement command has been sent to the WAM or not
         The variable is NOT thread-safe*/
        bool servingMove;

        /// Default position for the WAM in joint angles
        std::vector<double> defaultPosition;

        ///number of joints
        unsigned int numangles;

        //Protocol stuff
        /// Sends a command to the WAM
        void sendPacketHeader(int mainkind, int subkind);
        /// sends a query packet of specified subkind
        void sendQuery(int subkind);
        /// sends an async packet of specified subkind
        void sendAsync(int subkind);
        /// sends command packet of specified subkind without data
        void sendCommand(int subkind);
        /// send a raw command
        void sendCommand(std::string cmd);
        /// sends command packet of specified subkind and data
        void sendCommand(int subkind, std::vector<double> data);
        
        /// waits until bytesToRead bytes are ready to be read or a disconnection event happens
//        bool waitData(unsigned char *buffer, int bytesToRead);
        bool waitData(unsigned int bytesToRead);

        /// Reads the header of a packet from the socket
        void readPacket(WAMPacket *packet);        
        /// Prints the header in a human-readable way to stdout
        void printPacketHeader(WAMPacket *packet); 
        /// Prints packet data understood as list of doubles
        void printPacketData(WAMPacket *packet); 
        /// Prints the header and data (if non-empty) of a packet
        void printPacket(WAMPacket *packet); 

        ///processes de WAMPacket and refreshes WAMState local struct
        void refreshState(WAMPacket *packet);

        ///waits for the next packet arrival and 
        //copies the last packet's data vector to vect
        void waitvector(std::vector<double> *vect);


        /**
         * Given an string, builds the appropiate command packet.
         * Syntax:
         * TODO to be implemented
         */
        int parse_input(std::string buffer);

    public:
        /**
         *  Constructor
         *  @param port defines the port the driver will listen to.
         *  @param stateRefreshRate sets the refresh rate of the wam's state
         *  in milliseconds.
         */ 
        CWamDriver(std::string serverip, int port = DEFAULTPORT, int stateRefreshRate = DEFAULTSTATERATE);

        /** Method ran by the Service thread which reads the socket
         * and processes the data.
        */
        void serviceThread();

        /** Returns the string event that signals the movement is done
         * which can be waited in the eventserver singleton */
        std::string getTrajDoneEvent();

        /** \brief Translate error from bit strip to output
         *
         * prints the errors present in a human-readable text
         * to standard output
         */
        std::string errorToString(uint16_t errormask);

        /**
        * \brief Initialization function for wam. 
        *
        * It opens a socket and waits for wam connection.
        * Calls config method (which is void so far)
        * Finally, it turns gravity on.
        *
        * @todo Use config strings for listening port number, ...?. 
        */
        void open();
    
        void create();
        void activate();
        void deactivate();

        /**
        * Sends the wam home, shutdown's it and closes the connection.
        */
        void close();   

         ///////////////////////////////
         // Position Control  methods //
         ///////////////////////////////
       
        /**
         *  sets gravity to specified value
         *  @param gravity scale-up factor. 1 for calibrated value
         *  0 to disable gravity compensation. (Processed as a different
         *  packet)
         */
        void setGravityCompensation(double gravity);

        /**
         * \brief returns the number of Axes present, usually seven.
         * In cartesian, that's actually 3 for position and 3 for
         * rotation in euler angles. 
         * @return the number of axes
         */
        int getAxes();
        //returns the stored number of axes
        int getNumAngles();

        /**
         * Constraints/unconstraints the WAM to the current position.
         * The WAM is left to gravity compensation if turned off.
         */
        void holdCurrentPosition(bool on);

       /**
        * \brief Checks whether wam has finished its move. 
        * Blocking method that returns when the wam 
        * is idle waiting for commands or waits if it's moving
        * @deprecated per event
        */
        void waitTillMotionDone();

        /** 
         * \brief Stops a specific joint 
         * @todo not yet implemented
         */
        void stop(int j);

        /** 
         * \brief Stops current movement
         * @todo not yet implemented
         */
        void stop();
      
         ////////////////////////////////
         //         Get methods        //
         ////////////////////////////////
       
        /** 
         * \brief Returns the value of the angle for the specified joint
         *
         */
        double getJointAngle(int j);
       
        /**
        * \brief Returns the joint angles on a 7-element vector
        *
        *  @param angles array with the number of axes (tipically 7) where
        *  data will be stored.
        *  @see src/test/wamDriverTest.cpp
        */
        void getJointAngles(std::vector<double> *angles);

        /**
         * \brief Stores the position and rotation in cartesian coordinates
         * on a 6-element vector
         *
         * The format will be a a 4x4 matrix :(Rot3x3 | translation)
         *
         * @param posrot vector where data will be stored (size numcoordinates : 16)
         *
         */
        void getCartesianPose(std::vector<double> *posrot);

        /** 
         * \brief gets the speed of the joint j 
         */
        double getSpeed(int j);

        /** 
         * \brief gets the speed of each joint
         * @param spds Speeds vector in m/s
         */
        void getSpeeds(std::vector<double> *spds);

        /** 
         * \brief gets the current acceleration of joint j 
         */
        double getAcceleration(int j);

        /** 
         * \brief gets the accelerations for each joint
         * @param accs Acceleration vector for each joint 
         */
        void getAccelerations(std::vector<double> *accs);

        /**************************************/
        /*      WAM specific functions        */
        /**************************************/

        /**
        * Sets the torque threshold for joint j
        */
        void setTorqueLimits(const std::vector<double> torquesth);

        /**
        * Fetches the torque joint thresholds
        */
        void getTorqueLimits(std::vector<double> *torquesth);

        /**
        * Sets the torque limit of joint joint
        * @param joint joint id to apply the torque threshold to
        * @param threshold limit of torque upon which the WAM is stopped and exception issued
        */
        void setTorqueLimit(int joint,double threshold);

        /**
         * \brief Gets the torque force on the specified joint
         * @todo not yet implemented
         * @param j the selected joint to evaluate
         * @return torque Double where the torque force will be stored.
         */
        double getTorque(int j);

        /**
         * \brief Gets the torque force on each joint
         * @param torques An array of doubles where the torque of each
         * joint will be stored.
         */
        void getTorques(std::vector<double> *torques);

        /************************************/
        /*  Convinient operational methods  */
        /************************************/
        
        /**
         * Moves WAM to the home position
         */
        void home();

        /**
         * \brief Sets the position casted by the defaultPosition() method
         * The default position format is in joint angles
         * @param angles 7-dimensional stl vector
         */
        void setDefaultPosition(std::vector<double> angles);

        /**
         * \brief Moves the WAM to the default position
         * Moves the wam to the default position specified by the
         * setDefaultPosition method
         */
        void goToDefaultPosition();

        /**
         * \brief Performs an incremental move to all joints, specifying
         * the value of each joint increment in a 7-element array.
         *
         * @param deltas Array of increments, one for each joint, in radians
         */
        void incrementalJointMove(uint16_t *errormask, double angle, double vels = 0, double accs = 0);

        /**
         * \brief Performs an incremental move to the selected joint
         *
         * @param j Selected joint to move
         * @param delta increment angle in radians to move
         */
        void incrementalMoveInJoints(uint16_t *errormask, std::vector<double>* angles, std::vector<double>* vels = NULL, std::vector<double>* accs = NULL);

        /**
         * generic move, in joints
         * @param angles vector of angles for each joint
         * @param vels vector of velocities for each joint (default 0.5 m/s for all)
         * @param accs vector of accelerations for each joint (default ?)
         */
        void moveInJoints(uint16_t *errormask, std::vector<double>* angles, std::vector<double>* vels = NULL, std::vector<double>* accs = NULL);

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
        void moveInCartesian(std::vector<double>* pose, double vel = 0, double acc = 0);

        /**
         * velocity control, in cartesian
         * @param vels tridimensional vector of movement
         * @todo TODO Shouldn't it be 6 dimensional to contain rotation?
         * @param acc acceleration of the tool center point frame
         */
        void moveInSpeeds(std::vector<double> *vels, double acc);
        
        ///convenient methods

        /**
         * \brief Performs a blocking move in Cartesian coordinates
         *
         * Sends the move command and waits for movement
         * termination.
         * @param posrot position and rotation to get to
         */
        void moveAndWait(std::vector<double> *posrot);
        
        /**
         * Records a set of moves in a teach&play session
         */
        void startMovementsRecording();

        /**
         * Stops teach&play session
         */
        void stopMovementsRecording();

       /**
        * Plays the session
        */
        void playMovementsRecording();
};

#endif

