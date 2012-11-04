#include "CWamDriver.h"

using namespace std;

CWamDriver::CWamDriver(std::string serverip, int port, int stateRefreshRate){

    double anglesC[7] = {0.0, 0.0, 0.0, 2.2, 0.0, 0.0, 0.0};
    vector<double> startAngles(anglesC, anglesC + sizeof(anglesC) / sizeof(double) );
    this->setDefaultPosition(startAngles);

    this->serverip = serverip;
    this->port = port;
    //create Server Socket
    //this->csocket = new CSocketClient("Client socket");
    //create Event Socket
    this->eventserver = CEventServer::instance();

    connected = false;
    lastid = 0;

    memset(&appPacket, 0, sizeof(WAMPacket));

    this->servingMove = false;

}

void *socket_service_thread_function(void *param){

    CWamDriver *mywam = (CWamDriver *)param;
    mywam->serviceThread();

    pthread_exit(NULL);
}

/*
bool CWamDriver::waitData(unsigned char *buffer, int bytesToRead){

    int numbytes, bytesread=0, eventpos;

    cout << "[ServiceTH] Waiting for " << bytesToRead << " Bytes" << endl;
    while(bytesread < bytesToRead){
        // Proof that the event doesn't stay up while there's data to be read
        cout << "[ServiceTh] Bytes left " << csocket->get_num_data_from(name_client->client_id) << " Bytes "<< endl;
        eventpos = this->eventserver->wait_first(events);
        cout << "[ServiceTh] 2 Bytes left " << csocket->get_num_data_from(name_client->client_id) << " Bytes "<< endl;
        eventpos = this->eventserver->wait_first(events);
        switch(eventpos){
            case CONN_POS: //connection event
                cerr << "[ServiceTh][ERROR] Incoming connection while no more connections allowed. " << endl;
                break;
                //return false;
            case RX_ID_POS: //rx_id
                numbytes = min(bytesToRead, csocket->get_num_data_from(name_client->client_id));
                cout << "[ServiceTh] [wait] New data on socket (" << numbytes << " Bytes / " \ 
                     << csocket->get_num_data_from(name_client->client_id) << " Bytes)"<< endl;
                // read the piece of the cake here
                bytesread += this->csocket->read_from(name_client->client_id,buffer,numbytes);
                cout << "[ServiceTH] "<< bytesread << " Bytes read" << endl;
                break;
            case DISCONN_POS: //disconnection
                cout << "[ServiceTh][ERROR] Socket disconnected while waiting remaining bytes. " << endl;
                this->connected = false;
                return false;
            default:
                return false;
        }
    }
    cout << "[ServiceTh] Total bytes read " << bytesread << " Bytes "<< endl;
    cout << "[ServiceTh] Total bytes left " << csocket->get_num_data_from(name_client->client_id) << " Bytes "<< endl;
    return true;
}

void CWamDriver::readPacket(WAMPacket *packet){
    int offset, offset2, eventpos; 
    double buffer[64];

    cout << "[ServiceTH] Message received." << endl;

    if(!waitData((unsigned char *) &packet->header, sizeof(struct WAMPacketHeader))){
        cerr << "Data reception interrupted" << endl;
        return;
    }

    printPacketHeader(packet);

    if(packet->header.datalength > 0){
        if(!waitData((unsigned char *) &buffer[0], packet->header.datalength)){
            cerr << "Data reception interrupted" << endl;
            return;
        }
        packet->data.clear();
        for(int i=0; i<packet->header.datalength/sizeof(double); i++){
            packet->data.push_back(buffer[i]);
        }
    }
    printPacketData(packet);


    //process data and set the appropiate event
    //i.e. state on rx event => set staterx event on appevents
   switch(packet->header.mainkind){
        case STATE:
            //copy state to local replica
            statemutex.enter();

            if(packet->data[0] != 0.0)
                state.moving = true;
            else
                state.moving = false;
            state.pos.clear();
            offset = state.numcoordinates + 1;
            state.pos.assign(packet->data.begin() + 1, packet->data.begin() + offset);
            
            state.angles.clear();
            offset2 = offset + this->numangles;
            state.angles.assign(packet->data.begin() + offset, packet->data.begin() + offset2);

            state.vels.clear();
            offset = offset2 + this->numangles;
            state.vels.assign(packet->data.begin() + offset2, packet->data.begin() + offset);
            
            state.accs.clear();
            offset2 = offset + this->numangles;
            state.accs.assign(packet->data.begin() + offset, packet->data.begin() + offset2);

            statemutex.exit();
            packet->data.clear();
            break;
        case ASYNC:
            switch(packet->header.subkind){
                case MOVEDONE:
                    this->eventserver->set_event(TRAJDONE);
                    break;
                default:
                    cerr << "[ServiceTH] Error: unrecognized event:"<<packet->header.subkind <<endl; 
            } 
            break;
        case REPLY: 
            cout << "[ServiceTH] Query-reply message received" << endl; 
        default:
            cout << "[ServiceTH] other received" << endl; 
            packetqueue.push(*packet);            

            //packet-specific
            eventserver->set_event(DATARX); 
    }

    cout << "[ServiceTH] Message read." << endl;

}
*/

bool CWamDriver::waitData(unsigned int bytesToRead){

    int eventpos;

    while(csocket->get_num_data() < bytesToRead){
        eventpos = this->eventserver->wait_first(events);
        DEBUG("[ServiceTH] Waiting for %d Bytes of data\n", bytesToRead);
        switch(eventpos){
            case ERROR_POS: //disconnection
                DEBUG("[ServiceTh][ERROR] Socket disconnected while waiting remaining bytes. \n");
                this->connected = false;
                return false;
            case RX_ID_POS: //rx_id
                DEBUG("[ServiceTh] New data on socket (%d Bytes)\n",csocket->get_num_data());
                // read the piece of the cake here
                break;
            default:
                return false;
        }
    }
    DEBUG("[ServiceTh] Total data available (%d Bytes)\n",csocket->get_num_data());
    return true;
}

void CWamDriver::readPacket(WAMPacket *packet){
    int numbytes=0, offset, offset2; 
    double buffer[64];

    DEBUG("[ServiceTH] Message received.\n");

    if(!waitData(sizeof(struct WAMPacketHeader))){
        DEBUGE("Data reception interrupted\n");
        return;
    }
    numbytes = this->csocket->read((unsigned char *) &packet->header, sizeof(struct WAMPacketHeader));

    DEBUG("[ServiceTH] %d Bytes of header read\n", numbytes);

    printPacketHeader(packet);

    if(packet->header.datalength > 0){
        if(!waitData(packet->header.datalength)){
            DEBUGE("Data reception interrupted\n");
            return;
        }
        numbytes = this->csocket->read((unsigned char *) &buffer[0], packet->header.datalength);
        DEBUG("[ServiceTH] %d Bytes of state data read\n", numbytes);
        packet->data.clear();
        for(unsigned int i=0; i<packet->header.datalength/sizeof(double); i++){
            packet->data.push_back(buffer[i]);
        }
    }
    printPacketData(packet);

    //process data and set the appropiate event
    //i.e. state on rx event => set staterx event on appevents
   switch(packet->header.mainkind){
        case STATE:
            //copy state to local replica
            statemutex.enter();

            //unused field
            if(packet->data[0] != 0.0)
                state.moving = true;
            else
                state.moving = false;
            //unused at the moment
//            if(packet->data[1] != 0.0)
//                state.error = packet->data[1];
//            else
//                state.moving = packet->data[1];
            

            state.pos.clear();
            offset = state.numcoordinates + 2;
            state.pos.assign(packet->data.begin() + 2, packet->data.begin() + offset);
            
            state.angles.clear();
            offset2 = offset + this->numangles;
            state.angles.assign(packet->data.begin() + offset, packet->data.begin() + offset2);
            //DEBUGE("copied angles %f %f %f\n", state.angles.at(0),state.angles.at(1),state.angles.at(2));

            state.vels.clear();
            offset = offset2 + this->numangles;
            state.vels.assign(packet->data.begin() + offset2, packet->data.begin() + offset);
            
            state.accs.clear();
            offset2 = offset + this->numangles;
            state.accs.assign(packet->data.begin() + offset, packet->data.begin() + offset2);

            statemutex.exit();
            packet->data.clear();
            break;
        case ASYNC:
            switch(packet->header.subkind){
                case MOVEDONE:
                  //shall we filter if positive or negative data?
                    DEBUG("Entering to TRAJDONE SET");
                    this->servingMoveMutex.enter();
                    this->servingMove = false;
                    this->servingMoveMutex.exit();
                    this->eventserver->set_event(TRAJDONE);
                    break;
                case TORQUEEXCEDDED:
                    DEBUGE("[ServiceTH] Torque excedded detected on joint %f\n", packet->data[0]);
                    this->eventserver->set_event(TORQUEFAULT);
                    break;
                default:
                    DEBUGE("[ServiceTH] Error: unrecognized event: %d\n", packet->header.subkind); 
            } 
            break;
        case REPLY: 
            DEBUG("[ServiceTH] Query-reply message received\n"); 
        default:
            DEBUG("[ServiceTH] other received\n"); 
            packetqueue.push(*packet);            

            //packet-specific
            eventserver->set_event(DATARX); 
    }

    DEBUG("[ServiceTH] Message read.\n");

}

void CWamDriver::printPacketHeader(WAMPacket *packet){
    DEBUG("Packet ID: %d\n", packet->header.packetid);
    DEBUG("\tHeader:\n");
    DEBUG("\t\tID: %d\n", packet->header.packetid);
    DEBUG("\t\tMain Kind: %d\n", packet->header.mainkind);
    DEBUG("\t\tSub Kind: %d\n", packet->header.subkind);
    DEBUG("\t\tdata lenght: %d\n", packet->header.datalength);
}

void CWamDriver::printPacketData(WAMPacket *packet){
    DEBUG("[WD] \tData:\n\t");
    if(packet->header.datalength > 0){
        for(unsigned int i=0;i<packet->data.size();i++){
            DEBUG(" %f ",packet->data[i]);;
        }
        DEBUG("\n");
    }else{
        DEBUG("[WD] \t\t None \n");
    }
}

void CWamDriver::printPacket(WAMPacket *packet){
    printPacketHeader(packet);
    printPacketData(packet);
}

void CWamDriver::serviceThread(){
    WAMPacket apacket;
    list<string>::iterator eviterator;

    while(connected){
        DEBUG("[ServiceTh] Waiting event\n");
        try{
            int eventpos = this->eventserver->wait_first(events,5000);
            switch(eventpos){
                case ERROR_POS: //disconnection
                    DEBUG("[ServiceTh] Socket error. Assuming disconnected. \n");
                    this->connected = false;
                    break;
                case RX_ID_POS: //rx_id
                    DEBUG("[ServiceTh] New data on socket (%d Bytes)\n",csocket->get_num_data());
                    readPacket(&apacket);
                    break;
                default:
                    break;
            }
        }catch(CEventTimeoutException &e){
        }
    }
    DEBUG("[ServiceTh] Disconnected. Quiting...\n");
}

void CWamDriver::waitvector(std::vector<double> *vect){

    WAMPacket packetaux;
    list<string>::iterator eviterator;

    //theres only datarx at the moment
    int eventpos = this->eventserver->wait_first(appevents);
    DEBUG("[WD] Reply event received, processing... \n");
    switch(eventpos){
        case 0: //reply received event
            {
                eviterator = appevents.begin();
                //advance(eviterator,eventpos); //not necessary since its the only one at the moment
                packetaux = packetqueue.front();
                packetqueue.pop();
                vect->assign(packetaux.data.begin(),packetaux.data.end());
            }
        default:
            break;
    }
}

string CWamDriver::getTrajDoneEvent(){

    return string(TRAJDONE);

}

string CWamDriver::errorToString(uint16_t errormask){

    string error_msg;
    if(errormask&0x01){
        DEBUGE("[WD] Format error\n");
        error_msg = "[WD] Format error";
    }

    if(errormask&~0x01){
        DEBUGE("Unknown error or malformed errormask\n");
        error_msg = "Unknown error or malformed errormask"; 
    } 
    return error_msg;
}

void CWamDriver::open(){
    DEBUG("[WD] Connecting to WAM\n");

    TSocket_info config;
    config.address = this->serverip.c_str();
    config.port = this->port;

    this->csocket = new CSocketClient("pc2wam");

    this->csocket->open(&config);
    this->csocket->config();
    this->connected = true;

    events.push_back(csocket->get_error_event_id());
    DEBUG("[WD] Error event id: %s\n", events.back().c_str());


    events.push_back(csocket->get_rx_event_id());
    DEBUG("[WD] Rx event id: %s\n", events.back().c_str());
    
    servicethread = new CThread("socket service thread");
    servicethread->attach(socket_service_thread_function,this); 
    eventserver->create_event(DATARX);
    appevents.push_back(DATARX);
    
    //async events
    eventserver->create_event(TRAJDONE);
    eventserver->create_event(TORQUEFAULT);

    DEBUG("[WD] Starting service thread \n");
    servicethread->start();

    //fetch the number of axes active for joint readings
    this->numangles = getAxes();
    DEBUG("Connected, %d axes reported.\n", this->numangles);

}

/**
void CWamDriver::open(){

    //Socket configuration
    int listen_queue=5;
    TSocket_info serverconfig;
    serverconfig.address = "127.0.0.1" ;
    serverconfig.port = DEFAULTPORT;

    //starts listening
    csocket->open(&serverconfig);
    csocket->config(&listen_queue);
    csocket->set_max_clients(10);
    events.push_back(csocket->get_new_connection_event_id());
    csocket->start_server();
    cout << "[WD] Awaiting connection event: " << events.back());

    int eventpos = eventserver->wait_first(events);
    if(eventpos != 0){
        cerr << "[WD] Received an event instead of connection event: "<< eventpos << endl;
    } //check thats a connection_event

    cout << "[WD] received connection "<< eventpos << endl;
    this->name_client = csocket->get_new_client();
    cout << "[WD] New client: " << name_client->client_id << endl;
    cout << "  IP address: " << name_client->IP << " at port " << name_client->port << endl;
 
    events.push_back(name_client->disconnect_event_id);
    cout << "[WD] Disconnect client event: " << events.back() << endl;

    events.push_back(name_client->rx_event_id);
    cout << "[WD] Rx event id: " << events.back()  << endl;
    
    connected = true;

    servicethread = new CThread("socket service thread");
    servicethread->attach(socket_service_thread_function,this); 
    eventserver->create_event(DATARX);
    appevents.push_back(DATARX);
    
    //async events
    eventserver->create_event(TRAJDONE);
    eventserver->create_event(TORQUEFAULT);

    cout << "[WD] Starting service thread " << endl;
    servicethread->start();

    //fetch the number of axes active for joint readings
    this->numangles = getAxes();

}
*/

void CWamDriver::close(){
    //Bring the WAM home
    home();
    //warn the other end (WAM)
    WAMPacket packet;
    packet.header.packetid = ++lastid;
    packet.header.mainkind = COMMAND; 
    packet.header.subkind = SHUTDOWN; 
    packet.header.datalength = 0;
    socketmutex.enter();
    DEBUG("[WD] Termination inform %d\n", packet.header.packetid);
    csocket->write((unsigned char *)&packet.header, sizeof(struct WAMPacketHeader));
    socketmutex.exit();
     
    //destroy everything
    connected = false;
    DEBUG("[WD] Waiting service thread termination \n");
    servicethread->end();
    //servicethread->kill();
    //sleep(10);
    csocket->close();
    //return control to the application
    DEBUG("[WD] Returning control to app \n");

}

void CWamDriver::sendPacketHeader(int mainkind, int subkind){
    appPacket.header.packetid = ++lastid;
    appPacket.header.mainkind = mainkind;
    appPacket.header.subkind = subkind;
    appPacket.header.datalength = 0;

    socketmutex.enter();
    csocket->write((unsigned char *)&appPacket.header, sizeof(struct WAMPacketHeader));
    socketmutex.exit();
}

void CWamDriver::create(){

    std::vector<double> statevec;
    statevec.push_back(CREATED);
    sendCommand(INITSEQ,statevec);
}

void CWamDriver::activate(){

    std::vector<double> statevec;
    statevec.push_back(ACTIVATED);
    sendCommand(INITSEQ,statevec);
}

void CWamDriver::deactivate(){

    std::vector<double> statevec;
    statevec.push_back(DEACTIVATED);
    sendCommand(INITSEQ,statevec);
}

void CWamDriver::sendQuery(int subkind){

    sendPacketHeader(REPLY,subkind);
}

void CWamDriver::sendAsync(int subkind){

    sendPacketHeader(ASYNC,subkind);
}

void CWamDriver::sendCommand(int subkind){

    sendPacketHeader(COMMAND,subkind);
}

void CWamDriver::sendCommand(string data){

    appPacket.header.packetid = ++lastid;
    appPacket.header.mainkind = COMMAND;
    appPacket.header.subkind = RAWMESSAGE;

    appPacket.header.datalength = (data.length()+1)*sizeof(char);
    
//    for(int i=0;i<appPacket.data.size();i++){
//        cout << appPacket.data[i] << "  ";
//    }

    socketmutex.enter();
    DEBUG("[BHD] Writing packet %d\n", appPacket.header.packetid);
    csocket->write((unsigned char *)&appPacket.header, sizeof(struct WAMPacketHeader));
    if(appPacket.header.datalength > 0){
    //    DEBUG("[BHD] Writing appPacket's " << appPacket.header.datalength << " Bytes of data");
        csocket->write((unsigned char *)data.c_str(), appPacket.header.datalength);
    }
    DEBUG("[BHD] Written \n");
    socketmutex.exit();

}

void CWamDriver::sendCommand(int subkind, std::vector<double> data){

    appPacket.header.packetid = ++lastid;
    appPacket.header.mainkind = COMMAND;
    appPacket.header.subkind = subkind;
    appPacket.header.datalength = 0;
    appPacket.data.clear();

    appPacket.data.assign(data.begin(),data.end());

    appPacket.header.datalength = appPacket.data.size()*sizeof(double);
    
//    for(int i=0;i<appPacket.data.size();i++){
//        cout << appPacket.data[i] << "  ";
//    }

    socketmutex.enter();
    DEBUG("[WD] Writing packet %d\n", appPacket.header.packetid);
    csocket->write((unsigned char *)&appPacket.header, sizeof(struct WAMPacketHeader));
    if(appPacket.header.datalength > 0){
    //    cout << "[WD] Writing appPacket's " << appPacket.header.datalength << " Bytes of data" << endl;
        csocket->write((unsigned char *)&appPacket.data[0], appPacket.header.datalength);
    }
    DEBUG("[WD] Written \n");
    socketmutex.exit();

}

void CWamDriver::setDefaultPosition(std::vector<double> angles){
    
    defaultPosition.assign(angles.begin(), angles.end());

}

void CWamDriver::goToDefaultPosition(){
    
    uint16_t errormask = 0;
    moveInJoints(&errormask, &defaultPosition);    
    //TODO do something with errors

}

void CWamDriver::setGravityCompensation(double gravity){

    std::vector<double> gravityvec;
    gravityvec.push_back(gravity);
    sendCommand(GRAVITY, gravityvec);

}

void CWamDriver::holdCurrentPosition(bool on){

    if(on)
        sendCommand(HOLDON);
    else
        sendCommand(HOLDOFF);

}

void CWamDriver::incrementalJointMove(uint16_t *errormask, double angle, double vel, double accs){

}

void CWamDriver::incrementalMoveInJoints(uint16_t *errormask, std::vector<double>* angles, std::vector<double>* vels, std::vector<double>* accs){

    std::vector<double> data;
    data.assign(angles->begin(), angles->end());
    if(vels != NULL)
        data.insert(data.end(), vels->begin(), vels->end());
    if(accs != NULL)
        data.insert(data.end(), accs->begin(), accs->end());

    sendCommand(INCMOVEJOINTS,data);
}

void CWamDriver::moveInJoints(uint16_t *errormask, std::vector<double>* angles, std::vector<double>* vels, std::vector<double>* accs){
    
    std::vector<double> data;
    if(angles->size() != this->numangles){
        DEBUGE("[WD] numangles mismatch. Expected %d, received %d\n",this->numangles, angles->size());
        *errormask = *errormask | 0x01;
    }

    //does this work?
    data.assign(angles->begin(), angles->end());
    if(vels != NULL){
        data.insert(data.end(), vels->begin(), vels->end());
        if(vels->size() != this->numangles){
            *errormask = *errormask | 0x01;
        }
    }
    if(accs != NULL){
        data.insert(data.end(), accs->begin(), accs->end());
        if(accs->size() != this->numangles){
            *errormask = *errormask | 0x01;
        }
    }

    if(*errormask != 0){
        DEBUGE("[WD] Format error, ignoring command\n");
        return;
    }

    servingMoveMutex.enter();
    this->servingMove = true;
    servingMoveMutex.exit();

    sendCommand(MOVEJOINTS,data);

}

void CWamDriver::moveInCartesian(std::vector<double>* pose, double vel, const double acc){

    std::vector<double> data;
    data.assign(pose->begin(), pose->end());

    if(vel != 0){
        data.push_back(vel);
    }

    if(acc != 0){
        data.push_back(acc);
    }

    servingMoveMutex.enter();
    this->servingMove = true;
    servingMoveMutex.exit();

    sendCommand(MOVECARTESIAN,data);

}

void CWamDriver::moveInSpeeds(std::vector<double> *vels, double acc){

    std::vector<double> data;
    data.assign(vels->begin(), vels->end());

    data.push_back(acc);

    servingMoveMutex.enter();
    this->servingMove = true;
    servingMoveMutex.exit();

    sendCommand(MOVEINSPEEDS,data);

}

int CWamDriver::getNumAngles(){

    return this->numangles;
}

int CWamDriver::getAxes(){

    WAMPacket packetaux;
    list<string>::iterator eviterator;
    sendQuery(GETAXES);
    DEBUG("[WD] Waiting reply event \n");

    //theres only datarx at the moment
    int eventpos = this->eventserver->wait_first(appevents);
    DEBUG("[WD] Reply event received, processing... \n");
    switch(eventpos){
        case 0: //reply received event
            {
                eviterator = appevents.begin();
                //advance(eviterator,eventpos); //not necessary since its the only one at the moment
                packetaux = packetqueue.front();
                packetqueue.pop();
                return (int) packetaux.data.front();
            }
        default:
            break;
    }
    return 0;
}

void CWamDriver::getCartesianPose(std::vector<double> *posrot){
    statemutex.enter(); 
    posrot->assign(state.pos.begin(), state.pos.end());
    statemutex.exit();

}

double CWamDriver::getJointAngle(int j){
    double angle;
    statemutex.enter(); 
    angle = state.angles.at(j);
    statemutex.exit();
    return angle;
}

void CWamDriver::getJointAngles(std::vector<double> *angles){
    statemutex.enter(); 
    angles->assign(state.angles.begin(),state.angles.end());
    statemutex.exit();
}

double CWamDriver::getSpeed(int j){

    double speedj;
    statemutex.enter(); 
    speedj = state.vels.at(j);
    statemutex.exit();
    return speedj;
}

void CWamDriver::getSpeeds(std::vector<double> *spds){

    statemutex.enter(); 
    spds->assign(state.vels.begin(),state.vels.end());
    statemutex.exit();

}

double CWamDriver::getAcceleration(int j){

    double accj;
    statemutex.enter(); 
    accj = state.accs.at(j);
    statemutex.exit();
    return accj;
}

void CWamDriver::getAccelerations(std::vector<double> *accs){

    statemutex.enter(); 
    accs->assign(state.accs.begin(),state.accs.end());
    statemutex.exit();

}

void CWamDriver::getTorqueLimits(std::vector<double> *torquesth){

    sendQuery(GETTORQUELIMITS);

    DEBUG("[WD] Waiting reply event \n");
    waitvector(torquesth);
    
}

void CWamDriver::setTorqueLimit(int joint, double threshold){

    std::vector<double> torquevec;
    torquevec.push_back(joint);
    torquevec.push_back(threshold);
    sendCommand(SETTORQUELIMITS,torquevec);

}

void CWamDriver::setTorqueLimits(std::vector<double> torquesth){

    sendCommand(SETTORQUELIMITS, torquesth);
}

double CWamDriver::getTorque(int j){

    std::vector<double> torques;
    getTorques(&torques);
    return torques.at(j); 

}

void CWamDriver::getTorques(std::vector<double> *torques){

    sendQuery(GETTORQUES);

    DEBUG("[WD] Waiting reply event \n");
    waitvector(torques);

}

void CWamDriver::moveAndWait(std::vector<double> *posrot){
    moveInCartesian(posrot);
    waitTillMotionDone();
}

void CWamDriver::waitTillMotionDone(){
    list<string> waitevent;

    servingMoveMutex.enter();
    if(this->servingMove == false){
        servingMoveMutex.exit();
        return;
    }
    servingMoveMutex.exit();

    //adding TRAJDONE event notification to my event list
    waitevent.push_back(TRAJDONE);
    //TODO set timeout wait and exception catch
    this->eventserver->wait_first(waitevent);
    DEBUG("[WD] Movement finished, returning \n");
}

void CWamDriver::home(){
    sendCommand(HOME);
}

void CWamDriver::startMovementsRecording(){

    sendCommand("recording on\n");
}

void CWamDriver::stopMovementsRecording(){

    sendCommand("recording on\n");

}

void CWamDriver::playMovementsRecording(){

    sendCommand("recording play\n");

}
