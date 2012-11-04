#ifndef _PROTOCOL_H
#define _PROTOCOL_H

enum STATEMACHINE {
    OFF,
    LISTENING,
    IDLE,
    CREATED,
    //how to know that buttons have been pressed?
    ACTIVATED,
    DEACTIVATED
};

enum MAINMSGKIND {
    COMMAND,
    QUERY,
    REPLY,
    STATE,
    ASYNC
};

enum SUBMSGKIND {
    /** Data field specs within comments */

    /**Configuration messages, COMMANDS **/
    INITSEQ,
    GRAVITY,
        /**
         * 0 = disable, else normalized scale-up factor
         */
    HOME,
    SHUTDOWN,
    HOLDON,
    HOLDOFF,
    /** Operation messages, COMMANDS **/
    MOVEJOINT,
        /**
         * int joint 
         * double angle value
         * double speed value
         * double acceleration value
         */

    MOVEJOINTS,
        /**
         * double angles[7] 
         * double speeds[7]
         * double accelerations[7]
         */
    INCMOVEJOINTS,
        /**
         * double delta angles[7]
        */

    MOVECARTESIAN,
        /**
         * double pos[3]
         * double rot[9] rotation matrix
         * double speed (optional)
         * double acceleration (optional)
         */
    MOVEINSPEEDS,
    GETAXES,
        /**
         * preguntar a en sergi com funciona el control per velocitat
         * cartesianes o joints?
         * double vels[6]
         * double acceleration 
         */
    SETTORQUELIMITS,

    /** Reply messages, QUERY/REPLY **/
        /* empty */
    /** @deprecated by publishing state */
//        /**All messages return the values for all joints*/
//    GETPOSITION,
//    GETSPEEDS,
//    GETACCELERATIONS,
//    GETJOINTANGLES,
        /* empty */
    GETTORQUES,
    GETTORQUELIMITS,

    /** Operation messages, ASYNC **/
    MOVEDONE,
    TORQUEEXCEDDED,

    /** Other **/
    RAWMESSAGE
};
#endif
