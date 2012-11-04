#include "WamDriver.h"
using namespace std;

WamDriver::WamDriver(int port, int stateRefreshRate){

    //create Server Socket
    this->socketserver = new CSocketServer("Connection server for WAM");
    //create Event Socket
    this->eventserver = CEventServer::instance();

    connected = false;
    lastid = 0;

    memset(&appPacket, 0, sizeof(WAMPacket));

}

void *socket_service_thread_function(void *param){

    WamDriver *mywam = (WamDriver *)param;
    mywam->serviceThread();

    pthread_exit(NULL);
}

bool WamDriver::waitData(unsigned char *buffer, int bytesToRead){

    int numbytes, eventpos;

    while(numbytes = socketserver->get_num_data_from(name_client->client_id) < bytesToRead){
        eventpos = this->eventserver->wait_first(events);
        cout << "[ServiceTH] Waiting for " << bytesToRead << " Bytes of data" << endl;
        switch(eventpos){
            case CONN_POS: //connection event
                cerr << "[ServiceTh][ERROR] Incoming connection while no more connections allowed. " << endl;
                break;
                //return false;
            case RX_ID_POS: //rx_id
                cout << "[ServiceTh] New data on socket (" << socketserver->get_num_data_from(name_client->client_id) << " Bytes)"<< endl;
                // read the piece of the cake here
                numbytes = this->socketserver->read_from(name_client->client_id,buffer,numbytes);
                cout << "[ServiceTH] "<< numbytes << " Bytes of state data read" << endl;
                break;
            case DISCONN_POS: //disconnection
                cout << "[ServiceTh][ERROR] Socket disconnected while waiting remaining bytes. " << endl;
                this->connected = false;
                return false;
            default:
                return false;
        }
    }
    return true;
}

void WamDriver::readPacket(WAMPacket *packet){
    int numbytes=0, offset, offset2, eventpos; 
    double buffer[64];

    cout << "[ServiceTH] Message received." << endl;

    if(!waitData((unsigned char *) &packet->header, sizeof(struct WAMPacketHeader))){
        cerr << "Data reception interrupted" << endl;
        return;
    }

    cout << "[ServiceTH] " << numbytes << " Bytes of header read" << endl;

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
            offset2 = offset + state.numangles;
            state.angles.assign(packet->data.begin() + offset, packet->data.begin() + offset2);

            state.vels.clear();
            offset = offset2 + state.numangles;
            state.vels.assign(packet->data.begin() + offset2, packet->data.begin() + offset);
            
            state.accs.clear();
            offset2 = offset + state.numangles;
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

void WamDriver::printPacketHeader(WAMPacket *packet){
    cout << "Packet ID: " << packet->header.packetid << endl;
    cout << "\tHeader:" << endl;
    cout << "\t\tID: " << packet->header.packetid << endl;
    cout << "\t\tMain Kind: " << packet->header.mainkind << endl;
    cout << "\t\tSub Kind: " << packet->header.subkind << endl;
    cout << "\t\tdata lenght: " << packet->header.datalength << endl;
}

void WamDriver::printPacketData(WAMPacket *packet){
    cout << "[WD] \tData:" << endl << "\t";
    if(packet->header.datalength > 0){
        for(int i=0;i<packet->data.size();i++){
            cout << " " << packet->data[i] << " ";
        }
        cout << endl;
    }else{
        cout << "[WD] \t\t None \n" << endl;
    }
}

void WamDriver::printPacket(WAMPacket *packet){
    printPacketHeader(packet);
    printPacketData(packet);
}

void WamDriver::serviceThread(){
    unsigned int numb;
    WAMPacket apacket;
    list<string>::iterator eviterator;

    while(connected){
        cout << "[ServiceTh] Waiting event" << endl;
        int eventpos = this->eventserver->wait_first(events);
        switch(eventpos){
            case CONN_POS: //connection event
                cerr << "[ServiceTh] Error: Incoming connection while no more connections allowed. " << endl;
                break;
            case RX_ID_POS: //rx_id
                cout << "[ServiceTh] New data on socket (" << socketserver->get_num_data_from(name_client->client_id) << " Bytes)"<< endl;
                readPacket(&apacket);
                break;
            case DISCONN_POS: //disconnection
                cout << "[ServiceTh] Socket disconnected. " << endl;
                this->connected = false;
                break;
            default:
                break;
        }
    }
}

string WamDriver::getTrajDoneEvent(){

    return string(TRAJDONE);

}

void WamDriver::errorToString(uint16_t errormask){

    if(errormask&0x01)
        cerr << "[WD] Format error" << endl;

    if(errormask&~0x01)
        cerr << "Unknown error or malformed errormask" << endl;

}

void WamDriver::open(){

    //Socket configuration
    int listen_queue=5;
    TSocket_info serverconfig;
    serverconfig.address = "127.0.0.1" ;
    serverconfig.port = DEFAULTPORT;

    //starts listening
    socketserver->open(&serverconfig);
    socketserver->config(&listen_queue);
    socketserver->set_max_clients(10);
    events.push_back(socketserver->get_new_connection_event_id());
    socketserver->start_server();
    cout << "[WD] Awaiting connection event: " << events.back() << endl;

    int eventpos = eventserver->wait_first(events);
    if(eventpos != 0){
        cerr << "[WD] Received an event instead of connection event: "<< eventpos << endl;
    } //check thats a connection_event

    cout << "[WD] received connection "<< eventpos << endl;
    this->name_client = socketserver->get_new_client();
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

    cout << "[WD] Starting service thread " << endl;
    servicethread->start();

}

void WamDriver::close(){

    //warn the other end (WAM)
    WAMPacket packet;
    packet.header.packetid = ++lastid;
    packet.header.mainkind = COMMAND; 
    packet.header.subkind = SHUTDOWN; 
    packet.header.datalength = 0;
    socketmutex.enter();
    cout << "[WD] Termination inform " << packet.header.packetid << endl;
    socketserver->write_to(name_client->client_id, (unsigned char *)&packet.header, sizeof(struct WAMPacketHeader));
    socketmutex.exit();
     
    //destroy everything
    cout << "[WD] Waiting service thread termination " << endl;
    servicethread->end();
    //servicethread->kill();
    //return control to the application
    cout << "[WD] Returning control to app " << endl;

}

void WamDriver::sendPacketHeader(int mainkind, int subkind){
    appPacket.header.packetid = ++lastid;
    appPacket.header.mainkind = mainkind;
    appPacket.header.subkind = subkind;
    appPacket.header.datalength = 0;

    socketmutex.enter();
    socketserver->write_to(name_client->client_id, (unsigned char *)&appPacket.header, sizeof(struct WAMPacketHeader));
    socketmutex.exit();
}

void WamDriver::sendQuery(int subkind){

    sendPacketHeader(REPLY,subkind);
}

void WamDriver::sendAsync(int subkind){

    sendPacketHeader(ASYNC,subkind);
}

void WamDriver::sendCommand(int subkind){

    sendPacketHeader(COMMAND,subkind);
}

void WamDriver::sendCommand(int subkind, std::vector<double> data){

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
    cout << "[WD] Writing packet " << appPacket.header.packetid << endl;
    socketserver->write_to(name_client->client_id, (unsigned char *)&appPacket.header, sizeof(struct WAMPacketHeader));
    if(appPacket.header.datalength > 0){
    //    cout << "[WD] Writing appPacket's " << appPacket.header.datalength << " Bytes of data" << endl;
        socketserver->write_to(name_client->client_id, (unsigned char *)&appPacket.data[0], appPacket.header.datalength);
    }
    cout << "[WD] Written " << endl;
    socketmutex.exit();

}

void WamDriver::setDefaultPosition(std::vector<double> angles){
    
    defaultPosition.assign(angles.begin(), angles.end());

}

void WamDriver::setGravityCompensation(double gravity){

    std::vector<double> gravityvec;
    gravityvec.push_back(gravity);
    sendCommand(GRAVITY, gravityvec);

}

void WamDriver::holdCurrentPosition(bool on){

    if(on)
        sendCommand(HOLDON);
    else
        sendCommand(HOLDOFF);

}

void WamDriver::incrementalMoveInJoints(uint16_t *errormask, std::vector<double>* angles, std::vector<double>* vels, std::vector<double>* accs){

    std::vector<double> data;
    //does this work?
    data.assign(angles->begin(), angles->end());
    if(vels != NULL)
        data.insert(data.end(), vels->begin(), vels->end());
    if(accs != NULL)
        data.insert(data.end(), accs->begin(), accs->end());

    sendCommand(INCMOVEJOINTS,data);
}

void WamDriver::moveInJoints(uint16_t *errormask, std::vector<double>* angles, std::vector<double>* vels, std::vector<double>* accs){
    
    std::vector<double> data;
    if(angles->size() != 7){
        *errormask = *errormask | 0x01;
    }

    //does this work?
    data.assign(angles->begin(), angles->end());
    if(vels != NULL){
        data.insert(data.end(), vels->begin(), vels->end());
        if(vels->size() != 7){
            *errormask = *errormask | 0x01;
        }
    }
    if(accs != NULL){
        data.insert(data.end(), accs->begin(), accs->end());
        if(accs->size() != 7){
            *errormask = *errormask | 0x01;
        }
    }

    if(*errormask != 0){
        cerr << "[WD] Format error, ignoring command" << endl;
        return;
    }
    sendCommand(MOVEJOINTS,data);

}

void WamDriver::moveInCartesian(std::vector<double>* pos, double vel, const double acc){

    std::vector<double> data;
    //does this work?
    data.assign(pos->begin(), pos->end());
/*
    for(int i=0;i<pos->size();i++){
        data.push_back(pos->at(i));
    }
*/
    if(vel != 0){
        data.push_back(vel);
    }

    if(acc != 0){
        data.push_back(acc);
    }
    sendCommand(MOVECARTESIAN,data);

}

int WamDriver::getAxes(){

    WAMPacket packetaux;
    list<string>::iterator eviterator;
    sendQuery(GETAXES);
    cout << "[WD] Waiting reply event " << endl;

    //theres only datarx at the moment
    int eventpos = this->eventserver->wait_first(appevents);
    cout << "[WD] Reply event received, processing... " << endl;
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
}

void WamDriver::getCartesianPose(std::vector<double> *posrot){
    statemutex.enter(); 
    posrot->assign(state.pos.begin(), state.pos.end());
    statemutex.exit();

}

double WamDriver::getJointAngle(int j){
    double angle;
    statemutex.enter(); 
    angle = state.angles.at(j);
    statemutex.exit();
    return angle;
}

void WamDriver::getJointAngles(std::vector<double> *angles){
    statemutex.enter(); 
    angles->assign(state.angles.begin(),state.angles.end());
    statemutex.exit();
}

/* @deprecated
void WamDriver::getJointAnglesCommand(std::vector<double> *angles){

    WAMPacket packetaux;
    list<string>::iterator eviterator;

    sendQuery(GETJOINTANGLES);

    cout << "[WD] Waiting reply event " << endl;

    //theres only datarx at the moment
    int eventpos = this->eventserver->wait_first(appevents);
    cout << "[WD] Reply event received, processing... " << endl;
    switch(eventpos){
        case 0: //reply received event
            {
                eviterator = appevents.begin();
                //advance(eviterator,eventpos); //not necessary since its the only one at the moment
                packetaux = packetqueue.front();
                packetqueue.pop();
                angles->assign(packetaux.data.begin(),packetaux.data.end());
            }
        default:
            break;
    }
}
*/

double WamDriver::getSpeed(int j){

    double speedj;
    statemutex.enter(); 
    speedj = state.vels.at(j);
    statemutex.exit();
    return speedj;
}

void WamDriver::getSpeeds(std::vector<double> *spds){

    statemutex.enter(); 
    spds->assign(state.vels.begin(),state.vels.end());
    statemutex.exit();

}

double WamDriver::getAcceleration(int j){

    double accj;
    statemutex.enter(); 
    accj = state.accs.at(j);
    statemutex.exit();
    return accj;
}

void WamDriver::getAccelerations(std::vector<double> *accs){

    statemutex.enter(); 
    accs->assign(state.accs.begin(),state.accs.end());
    statemutex.exit();

}

double WamDriver::getTorque(int j){

    std::vector<double> torques;
    getTorques(&torques);
    return torques.at(j); 

}

void WamDriver::getTorques(std::vector<double> *torques){

    WAMPacket packetaux;
    list<string>::iterator eviterator;

    sendQuery(GETTORQUES);

    cout << "[WD] Waiting reply event " << endl;

    //theres only datarx at the moment
    int eventpos = this->eventserver->wait_first(appevents);
    cout << "[WD] Reply event received, processing... " << endl;
    switch(eventpos){
        case 0: //reply received event
            {
                eviterator = appevents.begin();
                //advance(eviterator,eventpos); //not necessary since its the only one at the moment
                packetaux = packetqueue.front();
                packetqueue.pop();
                torques->assign(packetaux.data.begin(),packetaux.data.end());
            }
        default:
            break;
    }
}

void WamDriver::waitTillMotionDone(){
    list<string> waitevent;

    sendAsync(MOVEDONE);

    waitevent.push_back(TRAJDONE);
    this->eventserver->wait_first(waitevent);
    cout << "[WD] Movement finished, returning " << endl;
}

void WamDriver::home(){
    sendCommand(HOME);

}

