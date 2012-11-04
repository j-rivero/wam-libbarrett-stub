#include "wamServer.h"

using namespace std;

WamServer::WamServer(const string serverip, unsigned int port, int stateRefreshRate, bool withStateThread, bool withTorqueThread){

        statemachine_ = OFF;
        //create Server Socket
        ssocket_ = new CSocketServer("Connection server for WAM");
        //create Event Socket
        eventserver_ = CEventServer::instance();
    
        port_ = port;
        server_ip_ = serverip;
        stateRefreshRate_ = stateRefreshRate;
        if (withStateThread_ == false)
            DEBUG("Warning, non-state thread is not supported yet\n");
        withStateThread_ = withStateThread;
        withTorqueThread_ = withTorqueThread;
        connected_ = false;
        shutdown_ = false;
        lastasyncid_ = 0;
    
    }
    
    void WamServer::open(){

        //Socket configuration
        int listen_queue = 5;
        TSocket_info serverconfig;
        serverconfig.address = server_ip_ ;
        serverconfig.port = port_;
    
        //starts listening
        ssocket_->open(&serverconfig);
        ssocket_->config(&listen_queue);
        ssocket_->set_max_clients(10);
        events_.push_back(ssocket_->get_new_connection_event_id());
        DEBUG("Start server\n");
        ssocket_->start_server();
        DEBUG("[WD] Awaiting connection event: %s\n", events_.back().c_str());
    
        this->waitConnection();
    }

    void WamServer::recovery(){

        switch(statemachine_){
            case ACTIVATED:
                this->moveHome(); 
                break;
            default:
                break;

        }
        if(events_.size() != 1){
            DEBUGE("events list size > 1, assuming reconnection\n");
            ssocket_->free_client(name_client_);

            events_.clear();
            events_.push_back(ssocket_->get_new_connection_event_id());
            DEBUGE("Restarting...\n");
        }
        this->waitConnection();
    }

    void WamServer::waitConnection(){
        
        int eventpos;
    
        eventpos = eventserver_->wait_first(events_);
        DEBUG("[WD] received connection %d\n", eventpos);
        name_client_ = ssocket_->get_new_client();
        DEBUG("[WD] received connection %d\n", eventpos);
        DEBUG("[WD] New client: %s\n", name_client_->client_id.c_str());
        DEBUG("  IP address: %s at port %d\n", name_client_->IP.c_str(), name_client_->port);
     
        events_.push_back(name_client_->disconnect_event_id);
        DEBUG("[WD] Disconnect client event: %s\n", events_.back().c_str());
    
        events_.push_back(name_client_->rx_event_id);
        DEBUG("[WD] Rx event id: %s\n", events_.back().c_str());

        mutex_connected_.enter(); 
        connected_ = true;
        mutex_connected_.exit();

        DEBUG("Connected\n");

    }

    void WamServer::create(){
        //TODO react only to IDLE
        if(statemachine_ == IDLE || statemachine_ == OFF || statemachine_ == LISTENING){
//            wamif_init();
//        	  wamif = wamif_create(0);
//          	wamif_activate(wamif);

          // INICIALIZACION_WAM 
          pm_.waitForWam(false);
          pm_.wakeAllPucks();
          bool wait_for_active = true;
          local_wam_ = pm_.getWam7(wait_for_active, BARRETT_SMF_WAM_CONFIG_PATH);
          // FIN INICIALIZACION 

        	  DEBUG("[Main thread] Robot ready for Activation\n");
            startMoveDoneThread();
            if(withStateThread_){
                startStateThread();
            }
            if(withTorqueThread_){
                startTorqueThread();
            }
        }else{
            DEBUGE("[Main thread] Wrong departure robot state: %d\n", statemachine_);
        }
    }

    void WamServer::destroy(){

        //kill helper threads
        //statemachine_ = DEACTIVATED; 
        shutdown_ = true;
        moveDoneThread_->end(); 
        if(withStateThread_){
           stateThread_->end();
        }
        if(withTorqueThread_){
           torqueNotifierThread_->end();
        }

        DEBUG("[Main thread] shutting down, idle the robot\n");
        pm_.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);
        DEBUG(" Idle event detected. Destroying wam...\n");
        local_wam_->idle();
    }

    bool WamServer::isClosing(){

        return shutdown_;
    }

bool WamServer::isConnected()
{
    bool connected;
    mutex_connected_.enter();
    connected = connected_;
    mutex_connected_.exit();
    return connected;
}

/*
    void WamServer::readPacket(WAMPacket *packet){
        double buffer[64];
        char strbuffer[256];
        int numbytes;
        
        DEBUG(endl << "Awaiting message.");
        while((numbytes = ssocket_->get_num_data_from(name_client_->client_id)) < 1){
            sleep(1);
        }
        
        DEBUG("Message received: " << numbytes << " Bytes");
        ssocket_->read_from(name_client_->client_id,(unsigned char *) &packet->header, sizeof(struct WAMPacketHeader));

        packet->data.clear();
        if(packet->header.subkind == RAWMESSAGE){
            
            numbytes = ssocket_->read_from(name_client_->client_id,(unsigned char *) &strbuffer[0], packet->header.datalength);
            laststrcmd_ = strbuffer;
        }else{

            if(packet->header.datalength > 0){
                numbytes = ssocket_->read_from(name_client_->client_id,(unsigned char *) &buffer[0], packet->header.datalength);
                for(int i=0; i*sizeof(double) < packet->header.datalength; i++){
                    packet->data.push_back(buffer[i]);
                }
            }else{
                numbytes = 0;
            }
            //if((ssocket_->read_from(name_client_->client_id,(unsigned char*)buffer,sizeof(buffer)))>1){
            //    printf("%s \n",buffer);
            //}
            DEBUG("Message read: " << numbytes+sizeof(struct WAMPacketHeader)<< " Bytes"<< endl;
        }
    }
*/
void WamServer::readPacket(WAMPacket *packet){

    unsigned char buffer[512];
    double dbuffer[64];
    unsigned int bytesRead = 0, num = 0, eventpos;

    packet->data.clear();

        while(bytesRead < sizeof(WAMPacketHeader)){
            DEBUG("Waiting data %d Bytes read\n", bytesRead);
            
            if ( ssocket_->get_num_data_from(name_client_->client_id) == 0 ) {
                eventpos = eventserver_->wait_first(events_);// es pot afegir un timeout al wait per evitar que quedi bloquejat (llença una excepció)
                switch(eventpos){
                    case RXDATA: //rx_id
                        DEBUG("[ServiceTh] New data on socket (%d Bytes)\n", ssocket_->get_num_data_from(name_client_->client_id));
                        // Read the piece of the cake after the switch
                        break;
                    case DISCONNECTION: //disconnection
                        DEBUGE("[ServiceTh] Socket Disconnection. \n");
                        mutex_connected_.enter();
                        connected_ = false;
                        mutex_connected_.exit();
                        DEBUGE("[ServiceTh] Throwing Disconnetion Exception. \n");
                        throw CDisconnectedException(_HERE_, "[ServiceTh][ERROR] Socket disconnected while waiting remaining bytes. \n");
                    case NEWCONNECTION:
                        throw CNewConnectionException(_HERE_,"[ServiceTh][ERROR] New Socket connection not allowed.\n");
                    default:
                        throw CException(_HERE_,"[ServiceTh][ERROR] Unkown event position \n");
                }
//                num = ssocket_->get_num_data_from(name_client_->client_id);
            }
            num = ssocket_->get_num_data_from(name_client_->client_id);
            if((bytesRead + num) > sizeof(WAMPacketHeader))
                num = sizeof(WAMPacketHeader) - bytesRead;
            ssocket_->read_from(name_client_->client_id, &buffer[bytesRead], num);
            bytesRead += num;
        }
        memcpy(&packet->header, buffer, sizeof(WAMPacketHeader));
        
        // process header -> reads the data
        if(packet->header.datalength > 0){
            if((num = ssocket_->get_num_data_from(name_client_->client_id)) > 0){ // maybe data is there already
                if(num > packet->header.datalength)
                    num = packet->header.datalength;
                ssocket_->read_from(name_client_->client_id, buffer, num);
                bytesRead = num;
            }else{
               bytesRead = 0;
            }
           //reads data 
           while(bytesRead < packet->header.datalength){
                eventserver_->wait_first(events_);
                switch(eventpos){
                    case DISCONNECTION: //disconnection
                        mutex_connected_.enter();
                        connected_ = false;
                        mutex_connected_.exit();
                        throw CDisconnectedException(_HERE_, "[ServiceTh][ERROR] Socket disconnected while waiting remaining bytes. \n");
                    case RXDATA: //rx_id
                        DEBUG("[ServiceTh] New data on socket (%d Bytes)\n", ssocket_->get_num_data_from(name_client_->client_id));
                        // continuing to read the piece of the cake
                        break;
                    case NEWCONNECTION:
                        throw CNewConnectionException(_HERE_,"[ServiceTh][ERROR] Socket disconnected while waiting remaining bytes. \n");
                    default:
                        throw CException(_HERE_,"[ServiceTh][ERROR] Unkown event position \n");
                }
                num = ssocket_->get_num_data_from(name_client_->client_id);
                if((bytesRead+num) > packet->header.datalength)
                    num = packet->header.datalength - bytesRead;
                ssocket_->read_from(name_client_->client_id, &buffer[bytesRead], num);
                bytesRead += num;
            }
    
            memcpy(dbuffer, buffer, packet->header.datalength); 

            if(packet->header.subkind == RAWMESSAGE){
                //data is a string
                laststrcmd_ = (char *)buffer;
            }else{
                //data is an array of doubles
                for(unsigned int ii=0; ii*sizeof(double) < packet->header.datalength; ii++){
                    packet->data.push_back(dbuffer[ii]);
                }
            }
        }
        DEBUG("[ServiceTh] Packet read\n");
    }

void WamServer::moveHome()
{
    local_wam_->moveHome(false);    
}

void WamServer::printPacket(WAMPacket *packet){
    DEBUG("Packet ID: %d\n", packet->header.packetid);
    DEBUG("\tHeader:\n");
    DEBUG("\t\tID: %d\n", packet->header.packetid);
    DEBUG("\t\tMain Kind: %d\n", packet->header.mainkind);
    DEBUG("\t\tSub Kind: %d\n", packet->header.subkind);
    DEBUG("\t\tdata lenght: %d\n", packet->header.datalength);
    DEBUG("\tData: \n");
    if(packet->header.datalength > 0){
        for(unsigned int i=0; i < packet->data.size(); i++){
            DEBUG(" %f ", packet->data[i]);
        }
        DEBUG("\n");
    }else{
        DEBUG("[WD] \t\t None \n ");
    }
}

void WamServer::packetInterpreter(WAMPacket *packet){

   int step;
   vector<double>::const_iterator cii;

   printPacket(packet);

   switch(packet->header.mainkind){
        case COMMAND:
            DEBUG("Command received\n"); 
            switch(packet->header.subkind){
                case INITSEQ:
                    step = (int) packet->data[0];
                    DEBUG("Requested State: %d, at present in %d\n", step, statemachine_);
		            switch(step){
                        case CREATED:
                            create();
                            statemachine_ = CREATED;
                            break;
                        case ACTIVATED:
                            statemachine_ = ACTIVATED; 
                            break;
                        case DEACTIVATED:
                            //destroy(); //should not enter here yet
                            DEBUGE("Deactivating not implemented yet %d\n", step);
                            statemachine_ = DEACTIVATED; 
                            break;
                        default:
                            DEBUGE("Unrecognized state %d\n", step);
                            break;
                    }
                    break;
                case GRAVITY:
                    if(packet->data[0] == 0){
                        DEBUG("Gravity disabled\n");
                    }else{
                        DEBUG("Set gravity to %f\n", packet->data[0]);
                    }
                      //wamif_set_gcomp(wamif, packet->data[0]);
                    local_wam_->gravityCompensate();
                    break;
                case HOME:
                    DEBUGE("Going home\n");
                    this->moveHome();
                    DEBUGE("Going home done\n");
                    break;
                case HOLDON:
                    //wamif_hold_position(wamif, 1);
                    local_wam_->moveTo(local_wam_->getJointPositions(), false);
                    break;
                case HOLDOFF:
                    //wamif_hold_position(wamif, 0);
                    local_wam_->idle();
                    break;
                case SHUTDOWN:
                    DEBUG("Activating 'Not connected' flag\n");
                    mutex_connected_.enter();
                    connected_ = false; //the other threads will end
                    mutex_connected_.exit();
                    this->close();
                    break;
                case MOVECARTESIAN:
                    DEBUG("Move cartesian\n"); 
                    DEBUG("Move cartesian is Disabled");
                    {
                        double vel = 0.4, acc = 0.3;
                        for(int i=0;i<POSESIZE;i++){
                            DEBUG(" %f ", packet->data[i]);
                        }
                        DEBUG("\n");
                        if(packet->data.size() >= POSESIZE+1){
                            DEBUG("vel: %f ",packet->data[POSESIZE+1]);
                            vel = packet->data[POSESIZE+1];
                        }
                        if(packet->data.size() >= POSESIZE+2){
                            DEBUG(" acc: %f ", packet->data[POSESIZE+2]);
                            acc = packet->data[POSESIZE+2];
                        }

                        //call moveInCartesian method
                        //wamif_move_cartesian(wamif,&packet->data[0], vel, acc);
                    }
                    DEBUG(" Move sent. \n");
                    break;
                case MOVEJOINTS:
                    {
                        //double angles[7];
                        jp_type angles;
                        DEBUG("Move Joints.\n"); 
                        if(packet->data.size() < this->getAxes()){
                            DEBUGE("Wrong number of axes: %d\n",packet->data.size());
                            break;
                        }
                        for(unsigned int i=0;i<7 && i<packet->data.size();i++){
                            DEBUG(" %f ",packet->data[i]);
                            //angles[i] = packet->data[i];
                            angles(i) = packet->data[i];
                        }
//                        DEBUG(endl << "[ignored] vels: ";
//                        for(int i=7;i<14 && i<packet->data.size();i++){
//                            DEBUG(" " << packet->data[i];
//                        }
//                        DEBUG(endl << "[ignored] accs: ";
//                        for(int i=14;i<21 && i<packet->data.size();i++){
//                            DEBUG(" " << packet->data[i];
//                        }
//                        DEBUG(endl;
                        DEBUG(" Move loaded. \n");
                        //wamif_move_joint(wamif, angles);
                        local_wam_->moveTo(angles, false);
                        DEBUG(" Move sent. \n");
                    }
                    break;
                case RAWMESSAGE:
                    DEBUG(" %s \n",laststrcmd_.c_str());
                    //switch (str)
                    break;
                default:
                    DEBUG("Unrecognized subkind received. Using string parser. \n"); 
                    break;
            }
            break;
        case REPLY:
            DEBUG("Query-reply message received\n"); 
            switch(packet->header.subkind){
                case GETAXES:
                    packet->header.datalength = sizeof(double);
                    packet->data.clear(); 
                    packet->data.push_back((double)getAxes()); 
                    DEBUG("Returning num axes: %f\n", packet->data[0]); 
                    printPacket(packet);
                    mutex_socket_.enter();
                    try{
                        ssocket_->write_to(name_client_->client_id, (unsigned char *)&packet->header, sizeof(WAMPacketHeader));
                        ssocket_->write_to(name_client_->client_id, (unsigned char *)&packet->data[0], packet->header.datalength);
                    }catch(CException& e){
                        DEBUGE("Error: ");
                        DEBUGE("Exception message: %s\n", e.what().c_str());
                        mutex_connected_.enter();
                        connected_ = false;
                        mutex_connected_.exit();
                    }
                    mutex_socket_.exit();
                    DEBUG("Done.\n"); 
                    break;
                //case GETJOINTANGLES: @deprecated
                case GETTORQUES:
                    packet->data.clear(); 
                    packet->data.push_back(7.0); 
                    packet->data.push_back(6.0); 
                    packet->data.push_back(5.0); 
                    packet->data.push_back(4.0); 
                    packet->data.push_back(3.0); 
                    packet->data.push_back(2.0); 
                    packet->data.push_back(1.0); 
                    packet->header.datalength = packet->data.size()*sizeof(double);
                    DEBUG("Returning angles: "); 
                    for(cii = packet->data.begin(); cii != packet->data.end(); cii++){
                          DEBUG(" %f ", *cii);
                    }
                    DEBUG("\n");

                    mutex_socket_.enter();
                    ssocket_->write_to(name_client_->client_id, (unsigned char *)&packet->header, sizeof(WAMPacketHeader));
                    ssocket_->write_to(name_client_->client_id, (unsigned char *)&packet->data[0], packet->header.datalength);
                    mutex_socket_.exit();
                    DEBUG("Done.\n"); 
                    break;
                default:
                    DEBUG("other subkind received \n"); 
                    break;
            }
            break;
        case ASYNC:
            case MOVEDONE:
                DEBUGE("Move done async disabled, use state: Don't send MOVE DONE async messages, they are sent automatically!\n");
                //startMoveDoneThread();
                break;
        default:
            DEBUG("other received\n"); 
    }
}

unsigned int WamServer::getAxes(){
    return 7;
}

    void WamServer::getState(WAMPacket *packet){
        double myPoseC[16] = {1.0, 0.0, 0.0, 0.0,
                              0.0, 1.0, 0.0, 0.0,
                              0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 1.0};

        double myDefaultAngles[7] = {0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0};
        /* velocity feed back not provided by wam??*/
        double myVels[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
        double myAccs[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
        packet->header.packetid = 0;
        packet->header.mainkind = STATE;
        packet->header.subkind = 0;

        packet->data.clear();

        //double moving = !wamif_check_traj_done(wamif);
        double moving = !local_wam_->moveIsDone();
        cout << "moving: " << moving << endl;

        packet->data.push_back(moving);

        double error = 0;
        packet->data.push_back(error);

        // pose_type is a tuple with position and the orientation in a quaternion
        pose_type current_pose = local_wam_->getToolPose();
        cp_type current_xyz = current_pose.get<0>();
        Eigen::Quaterniond current_xyzw = current_pose.get<1>();
        Eigen::Quaterniond::Matrix3 current_rot = current_xyzw.toRotationMatrix();
        // Traslation
        myPoseC[3] = current_xyz(0);
        myPoseC[7] = current_xyz(1);
        myPoseC[11] = current_xyz(2);
        // Rotation
        myPoseC[0] = current_rot(0);
        myPoseC[1] = current_rot(1);
        myPoseC[2] = current_rot(2);
        myPoseC[4] = current_rot(3);
        myPoseC[5] = current_rot(4);
        myPoseC[6] = current_rot(5);
        myPoseC[8] = current_rot(6);
        myPoseC[9] = current_rot(7);
        myPoseC[10] = current_rot(8);
        // Last Matrix file
        myPoseC[12] = current_rot(0);
        myPoseC[13] = current_rot(0);
        myPoseC[14] = current_rot(0);
        myPoseC[15] = current_rot(1);

        for(int ii=0; ii<POSESIZE; ii++)
             packet->data.push_back(myPoseC[ii]);

        //Assuming 7 degrees of freedom!!
        jp_type current_joint_position = local_wam_->getJointPositions();
    	  //wamif_get_joint(wamif, myAngles);
        if(current_joint_position.size() != 7){
          DEBUGE("Joint dimension mismatch! returning default values");
          for(int i=0;i<7;i++)
              packet->data.push_back(myDefaultAngles[i]);
        }

        for(int i=0;i<7;i++)
            packet->data.push_back(current_joint_position(i));
//            packet->data.push_back(myDefaultAngles[i]);

        /*TODO velocity feed back not provided by wam??*/
        for(int i=0;i<7;i++)
            packet->data.push_back(myVels[i]);
        for(int i=0;i<7;i++)
            packet->data.push_back(myAccs[i]);

        packet->header.datalength = packet->data.size()*sizeof(double);
    }

    void *state_publisher_function(void *param){

        WamServer *mywam = (WamServer *)param;
        mywam->statePublisher();

        pthread_exit(NULL);
    }

void WamServer::statePublisher(){

    WAMPacket packet;

    DEBUG("State Publisher active\n");
    while(!this->isClosing()){
        if(this->isConnected()){ 
            this->getState(&packet);
            DEBUGE("[State Publisher] Writing state\n");
            try{
                mutex_socket_.enter();
                ssocket_->write_to(name_client_->client_id, (unsigned char*)&packet.header, sizeof(struct WAMPacketHeader));
                DEBUGE("[State Publisher] Writing state data\n");
                //printPacket(&packet);
                //DEBUGE("[State Publisher] Writing state data2\n");
                ssocket_->write_to(name_client_->client_id, (unsigned char*)&packet.data[0], packet.header.datalength);
                mutex_socket_.exit();
                DEBUGE("[State Publisher] state written 1\n");
            }catch(CException& e){
                DEBUGE("Error: ");
                DEBUGE("Exception message: %s\n", e.what().c_str());
                mutex_connected_.enter();
                connected_ = false;
                mutex_connected_.exit();
            }
            DEBUGE("[State Publisher] state written\n");
            usleep(stateRefreshRate_);
        }else{
            DEBUG("[State Publisher] Not connected...\n");
            sleep(3);
        }
    }

//        DEBUG("State Publisher closing...");
//        if(connected_){
//            this->getState(&packet);
//            DEBUG("Writing state");
//            ssocket_->write_to(((unsigned char*)&packet, sizeof(struct WAMPacket));
//        }
        DEBUG("State Publisher exiting...\n");
        pthread_exit(NULL);
    }

    void WamServer::startStateThread(){
       stateThread_ = new CThread("wamstate publisher");
       DEBUG("Attaching publisher...\n");
       stateThread_->attach(state_publisher_function,this);
       DEBUG("Starting publisher...\n");
       stateThread_->start();     
    }

    void *torque_faults_notifier(void *param){

        WamServer *mywam = (WamServer *)param;
        mywam->torqueFaultsNotifier();
        pthread_exit(NULL);
    }

    void WamServer::torqueFaultsNotifier(){
        WAMPacket packet;

        packet.header.packetid = 0;
        packet.header.mainkind = ASYNC;
        packet.header.subkind = TORQUEEXCEDDED;

        packet.data.clear();
        packet.header.datalength = sizeof(double);

        srand(time(NULL));

        DEBUG("Torque Notifier active\n");
        while(!this->isClosing()){
            // Look for torque faults with timeout @TODO real torque fault
            if(isConnected()){ 
                if(rand() % 10 < 9){
                    sleep(3);
                }else{
                    mutex_socket_.enter();
                    DEBUG("Writing Torque Fault\n");
                    packet.data.push_back(rand()%7);
                    try{
                        ssocket_->write_to(name_client_->client_id, (unsigned char*)&packet.header, sizeof(struct WAMPacketHeader));
                        ssocket_->write_to(name_client_->client_id, (unsigned char*)&packet.data[0], packet.header.datalength);
                    }catch(CException& e){
                        DEBUGE("Error: ");
                        DEBUGE("Exception message: %s\n", e.what().c_str());
                        mutex_connected_.enter();
                        connected_ = false;
                        mutex_connected_.exit();
                    }
                    DEBUG("Torque Fault Written\n");
                    mutex_socket_.exit();
                }
            }else{
                DEBUG("[Torque notifier] Not connected...\n");
                sleep(3);
            }
        }

        DEBUG("Torque notifier exiting...\n");
        pthread_exit(NULL);
    }

    void WamServer::startTorqueThread(){
       torqueNotifierThread_ = new CThread("wam torque notifier");
       DEBUG("Attaching publisher...\n");
       torqueNotifierThread_->attach(torque_faults_notifier,this);
       DEBUG("Starting publisher...\n");
       torqueNotifierThread_->start();     
    }

    void *async_move_done(void *param){

        WamServer *mywam = (WamServer *)param;

        mywam->moveDone();

        pthread_exit(NULL);
    }

    void WamServer::moveDone(){
        int numbytes = 0;
        int lastState = true; //not moving
        int lastStateaux = true;
        WAMPacket async;
        async.header.packetid = ++lastasyncid_;
        async.header.mainkind = ASYNC;
        async.header.subkind = MOVEDONE;
        async.header.datalength = 0;  
        async.data.clear();

        async.header.datalength = sizeof(double);


        DEBUG("[moveDone] Move Done Notifier active\n");
        while(!this->isClosing()){
            if(isConnected()){
                //call to move done
                //if((lastStateaux = wamif_check_traj_done(wamif)) != lastState){
                if((lastStateaux = local_wam_->moveIsDone()) != lastState){
                    lastState = lastStateaux; 
                    DEBUG("[moveDone] %d change\n",lastState );
                    if(lastStateaux == true){
                        async.data.clear();
                        async.data.push_back((double) lastState);
                        //report back asynchronously and exit
                        DEBUG("[moveDone] Report back %d\n",lastState);
                        printPacket(&async);
                        mutex_socket_.enter();
                        try{
                            DEBUG("[moveDone] write\n");
                            numbytes = ssocket_->write_to(name_client_->client_id, (unsigned char *)&async.header, sizeof(WAMPacketHeader));
                            numbytes += ssocket_->write_to(name_client_->client_id, (unsigned char *)&async.data[0], async.header.datalength);
                            DEBUG("[moveDone] written\n");
                        }catch(CException& e){
                            DEBUGE("Error: ");
                            DEBUGE("Exception message: %s\n", e.what().c_str());
                            mutex_connected_.enter();
                            connected_ = false;
                            mutex_connected_.exit();
                        }
                        mutex_socket_.exit();
                        DEBUG("[moveDone] %d Bytes written. Loop.\n",numbytes);
                    }
                }
                usleep(MOVEDONERATE);
            }else{
                DEBUG("[moveDone] Not connected...\n");
                sleep(3);
            }
        }
        DEBUG("[moveDone] Move Done Notifier ending\n");
        pthread_exit(NULL);

    }

void WamServer::startMoveDoneThread(){
    moveDoneThread_ = new CThread("wam move done checker");
    DEBUG("Attaching moveDone...\n");
    moveDoneThread_->attach(async_move_done,this);
    DEBUG("Starting moveDone...\n");
    moveDoneThread_->start();     
    DEBUG("Returning to main thread...\n");
}

void WamServer::close(){
    DEBUG("Going HOME ...\n"); 
    this->moveHome();
    DEBUG("HOME pose reached.\n"); 

    DEBUG("Destroying wam structure and other threads...\n"); 
    this->destroy();

    DEBUG("Freeing client...\n"); 
    mutex_socket_.enter();
    ssocket_->free_client(name_client_);
    mutex_socket_.exit();

    DEBUG("Terminating.\n"); 
}

/***************************************/
/********* MAIN ************************/
/***************************************/

int main(int argc, char *argv[]){

    WAMPacket packet;

    DEBUG("Starting WAM virtual\n");

    //log_open("WAMtest.log"); //LOG_INFO macros segfault in wamif if log's not open

    WamServer *wam = new WamServer(DEFAULT_SERVER_IP);

    //libarrett init
    //BARRETT_UNITS_TYPEDEFS(7);
  
    barrett::installExceptionHandler();
    
    //libbarrett end

    DEBUG("Opening WAM\n");
    try{
    
        wam->open();

        //start wam state publisher thread
        DEBUG("Starting proxy loop.\n");

        while(!wam->isClosing()){
            //blocking read
            try{
               wam->readPacket(&packet); 
            }catch(CDisconnectedException& e){
                DEBUGE("Connection error: ");
                DEBUGE("Exception message: %s\n", e.what().c_str());
                DEBUG("Attempting reconnection: ");
            }catch(CNewConnectionException& e){
                DEBUGE("NEW Connection error: ");
                DEBUGE("Exception message: %s\n", e.what().c_str());
                continue;
            }

            if(wam->isClosing()){
                DEBUG("Wam terminating main loop.\n");
            } else if(!wam->isConnected()) {
                DEBUGE("Connection error catched.\n");
                DEBUG("Attempting reconnection. Listening. \n");
                wam->recovery();
                DEBUG("ReStarting proxy loop.\n");
            } else {
                wam->packetInterpreter(&packet);
            }
        }

    //sembla que open de clientsocket captura les seves excepcions
    }catch(CException &e){
        DEBUGE("Error: ");
        DEBUGE("Exception message: %s\n", e.what().c_str());
    }
    DEBUG("Exiting.\n");

    return 0;

}
