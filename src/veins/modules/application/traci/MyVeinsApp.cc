#include "veins/modules/application/traci/MyVeinsApp.h"
#include<string>
#include<vector>
#include<utility>

using namespace Veins;

Define_Module(Veins::MyVeinsApp);

std::set<int>NeighbouringClusterHead;
std::map< int,Coord> availableCars;
std::vector< std::pair<int,Coord>> PossibleClusterHead;
std::set<int>ClusterContent;
std::set<int>Neighbours;
double range=1000;

std::ofstream Stabilty;
std::ofstream Reaffiliation;
simtime_t end;
void MyVeinsApp::initialize(int stage)
{

    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        // Initializing members and pointers of your application goes here
        //EV << "Testing Initialization " << par("appName").stringValue()<< std::endl;
        countSignal = registerSignal("countClusterHead");
        //Stabilty.open("/home/shashwat-rai/Desktop/ClusterStability.txt",std::ios::app);
        //countLife=registerSignal("LifeTime");
        //countReaffiliation=registerSignal("Reaffiliation");
        //Stabilty<<(getParentModule()->getFullName())<<endl;
        //Stabilty.close();
        EV<<getParentModule()->getFullName()<<" came into existence";
        period=new cMessage("TimeOut");
        scheduleAt(simTime(),period);
    }
    else if (stage == 1) {
        // Initializing members that require initialized other modules goes here
    }
}

void MyVeinsApp::finish()
{
    DemoBaseApplLayer::finish();
    // statistics recording goes here
    //simtime_t end=(simTime()-start);
    //emit(countLife,end);
    cModule *parentMod=getParentModule();
    if(parentMod->par("isClusterHead")){
    	end=simTime();
    	std::string tempString=(parentMod->par("start"));
        EV<<(parentMod->getFullName())<<" "<<tempString<<" "<<end<<endl;
        Stabilty.open("/home/shashwat-rai/Desktop/DataSets/ClusterStability.txt",std::ios::app);
    	Stabilty<<(parentMod->getFullName())<<" "<<tempString<<" "<<end<<endl;
    	Stabilty.close();
    }


}

void MyVeinsApp::onBSM(DemoSafetyMessage* bsm)
{
    // Your application has received a beacon message from another car or RSU
    // code for handling the message goes here

    cModule *parentMod=getParentModule();
    Coord Sp=bsm->getSenderPos();
    int modId=bsm->getNodeId();
    cModule *senderMod=getSimulation()->getModule(modId)->getParentModule();
    
    availableCars.insert(std::make_pair(senderMod->getId(),Sp));
    
    if((bool)(parentMod->par("isMarked"))==false){
        //EV<<"emiting signal"<<endl;
        //emit(countReaffiliation,v);
        if((bool)(parentMod->par("firstTime"))==false){
            EV<<"Reaffilition "<<(parentMod->getFullName())<<endl;
            Reaffiliation.open("/home/shashwat-rai/Desktop/DataSets/reaffiliation.txt", std::ios::app);
            Reaffiliation<<1<<endl;
            Reaffiliation.close();
        }
        MyVeinsApp::CreateCluster();
        MyVeinsApp::definingGateway();

    }
    else {
        //std::vector<int> v;
        std::set<int>::iterator it;
        
        for(it=Neighbours.begin();it!=Neighbours.end();it++){
            if(getSimulation()->getModule(*it)==NULL){
                EV<<parentMod->getFullName()<<" removing "<< *it<<endl;
                Neighbours.erase(it);
                
            }
        }

        if(parentMod->par("isClusterHead")){
            TraCIMobility* mobility1 = TraCIMobilityAccess().get(parentMod);
            Coord RecieverPosition = mobility1->getPositionAt(simTime());
            double d=RecieverPosition.distance(Sp);
            
            if(d<=range){

                //ClusterContent.insert(senderMod->getId());
                if(senderMod->getId()<parentMod->getId()){
                    double temp=(parentMod->getParentModule())->par("noofClusterHead");
                    (parentMod->getParentModule())->par("noofClusterHead")=temp-1;
                    cTimestampedValue tmp(simTime(),(double)(parentMod->getParentModule())->par("noofClusterHead"));
                    emit(countSignal,&tmp);
                    end=simTime();
                    std::string tempString=(parentMod->par("start"));
                    EV<<(parentMod->getFullName())<<" "<<tempString<<" "<<end<<endl;
                    Stabilty.open("/home/shashwat-rai/Desktop/DataSets/ClusterStability.txt",std::ios::app); 
                    Stabilty<<(parentMod->getFullName())<<" "<<tempString<<" "<<end<<endl;
                    Stabilty.close();
                    parentMod->par("isClusterHead")=false;
                    parentMod->par("isMarked")=false;
                    
                    //parentMod->par("isGateway")=false;
                    //parentMod->par("isMarked")=false;
                    //EV<<"in maintaining phase"<<endl;
                    //emit(countReaffiliation,1);
                    //MyVeinsApp::CreateCluster();
                    //MyVeinsApp::definingGateway();
                    for(auto t:Neighbours){
                        if((getSimulation()->getModule(t))){
                            cModule *SenderParentMod=(getSimulation()->getModule(t));
                            if(SenderParentMod){
                                SenderParentMod->par("isMarked")=false;
                                SenderParentMod->par("isGateway")=false;
                            }
                        }
                    }
                    Neighbours.insert(senderMod->getId());
                    //EV<<"emiting signal"<<endl;
                    //emit(countReaffiliation,v);
                    EV<<"Reaffilition "<<(parentMod->getFullName())<<endl;
                    Reaffiliation.open("/home/shashwat-rai/Desktop/DataSets/reaffiliation.txt", std::ios::app);
                    Reaffiliation<<1<<endl;
                    Reaffiliation.close();
                    MyVeinsApp::CreateCluster();
                    MyVeinsApp::definingGateway();
                    
                }
                else{
                	Neighbours.insert(senderMod->getId());
                	ClusterContent.insert(senderMod->getId());
                    if((bool)senderMod->par("isClusterHead")){
                        end=simTime();
                        std::string tempString=(senderMod->par("start"));
                        EV<<(senderMod->getFullName())<<" "<<tempString<<" "<<end<<endl;
                        Stabilty.open("/home/shashwat-rai/Desktop/DataSets/ClusterStability.txt",std::ios::app);    
                        Stabilty<<(senderMod->getFullName())<<" "<<tempString<<" "<<end<<endl;
                        Stabilty.close();
                        double temp=(parentMod->getParentModule())->par("noofClusterHead");
                        (parentMod->getParentModule())->par("noofClusterHead")=temp-1;
                        cTimestampedValue tmp(simTime(),(double)(parentMod->getParentModule())->par("noofClusterHead"));
                        emit(countSignal,&tmp);
                    }
                    senderMod->par("isClusterHead")=false;
                    senderMod->par("isMarked")=true;
                    senderMod->par("firstTime")=false;
                }
            }
            else{
                if(Neighbours.find(senderMod->getId())!=Neighbours.end()){
                    Neighbours.erase(Neighbours.find(senderMod->getId()));

                }

	            if(Neighbours.size()==0){
		            if((bool)parentMod->par("isClusterHead")){
		                end=simTime();
		                std::string tempString=(parentMod->par("start"));
		                EV<<(parentMod->getFullName())<<" "<<tempString<<" "<<end<<endl;
		                Stabilty.open("/home/shashwat-rai/Desktop/DataSets/ClusterStability.txt",std::ios::app);
		                Stabilty<<(parentMod->getFullName())<<" "<<tempString<<" "<<end<<endl;
		                Stabilty.close();
		                double temp=(parentMod->getParentModule())->par("noofClusterHead");
		                (parentMod->getParentModule())->par("noofClusterHead")=temp-1;
		                cTimestampedValue tmp(simTime(),(double)(parentMod->getParentModule())->par("noofClusterHead"));
		                emit(countSignal,&tmp);
		            }

		            parentMod->par("isClusterHead")=false;
		            parentMod->par("isGateway")=false;
		            parentMod->par("isMarked")=false;
		            //EV<<"in maintaining phase"<<endl;
		            //EV<<"emiting signal"<<endl;
		            //emit(countReaffiliation,v);
		            EV<<"Reaffilition "<<(parentMod->getFullName())<<endl;
		            Reaffiliation.open("/home/shashwat-rai/Desktop/DataSets/reaffiliation.txt", std::ios::app);
                    Reaffiliation<<1<<endl;
                    Reaffiliation.close();
		            MyVeinsApp::CreateCluster();
		            MyVeinsApp::definingGateway();
            	}

        	}
        }
        else {
            TraCIMobility* mobility1 = TraCIMobilityAccess().get(parentMod);
            Coord RecieverPosition = mobility1->getPositionAt(simTime());
            double d=RecieverPosition.distance(Sp);
            
            if(d<=range){
                Neighbours.insert(senderMod->getId());
                if(senderMod->par("isClusterHead")){
                	if(senderMod->getId()>parentMod->getId()){
	                    parentMod->par("isClusterHead")=true;
	                    parentMod->par("firstTime")=false;
	                    std::ostringstream str1;
	                    str1<<simTime();
	                    parentMod->par("start")=str1.str();
	                    double temp=(parentMod->getParentModule())->par("noofClusterHead");
	                    (parentMod->getParentModule())->par("noofClusterHead")=temp+1;
	                    cTimestampedValue tmp(simTime(),(double)(parentMod->getParentModule())->par("noofClusterHead"));
	                    emit(countSignal,&tmp);
	                    for(auto t:Neighbours){
	                        if((getSimulation()->getModule(t))){
	                            cModule *SenderParentMod=(getSimulation()->getModule(t));
	                            if(SenderParentMod){
	                                bool check=SenderParentMod->par("isClusterHead");
	                                if(check==true){
	                                	end=simTime();
	                                	std::string tempString=(SenderParentMod->par("start"));
	                                	EV<<SenderParentMod->getFullName()<<tempString<<end<<endl;
	                                    Stabilty.open("/home/shashwat-rai/Desktop/DataSets/ClusterStability.txt",std::ios::app);
	                                	Stabilty<<SenderParentMod->getFullName()<<tempString<<end<<endl;
	                                	Stabilty.close();
	                                    SenderParentMod->par("isClusterHead")=false;
	                                    temp=(parentMod->getParentModule())->par("noofClusterHead");
	                                    (parentMod->getParentModule())->par("noofClusterHead")=temp-1;
	                                    cTimestampedValue tmp(simTime(),(double)(parentMod->getParentModule())->par("noofClusterHead"));
	                                    emit(countSignal,&tmp);
	                                }
	                                SenderParentMod->par("isMarked")=true;
	                                SenderParentMod->par("firstTime")=false;
	                                ClusterContent.insert(t);
	                            }
	                        }
	                    }
	                }
	                else{
	                	parentMod->par("isMarked")=true;
	                	MyVeinsApp::definingGateway();
	                }		

                }
	   
            }
            else{
                if(Neighbours.find(senderMod->getId())!=Neighbours.end()){
                    Neighbours.erase(Neighbours.find(senderMod->getId()));
                }
	            if(Neighbours.size()==0){
		            if((bool)parentMod->par("isClusterHead")){
		                end=simTime();
		                std::string tempString=(parentMod->par("start"));
		                EV<<parentMod->getFullName()<<" "<<tempString<<" "<<end<<endl;
		                Stabilty.open("/home/shashwat-rai/Desktop/DataSets/ClusterStability.txt",std::ios::app);
		                Stabilty<<parentMod->getFullName()<<" "<<tempString<<" "<<end<<endl;
		                Stabilty.close();
		                double temp=(parentMod->getParentModule())->par("noofClusterHead");
		                (parentMod->getParentModule())->par("noofClusterHead")=temp-1;
		                cTimestampedValue tmp(simTime(),(double)(parentMod->getParentModule())->par("noofClusterHead"));
		                emit(countSignal,&tmp);
		            }
		            parentMod->par("isClusterHead")=false;
		            parentMod->par("isGateway")=false;
		            parentMod->par("isMarked")=false;
		            //EV<<"in maintaining phase"<<endl;
		            //emit(countReaffiliation,1);
		            EV<<"Reaffilition "<<(parentMod->getFullName())<<endl;
		            Reaffiliation.open("/home/shashwat-rai/Desktop/DataSets/reaffiliation.txt", std::ios::app);
                    Reaffiliation<<1<<endl;
                    Reaffiliation.close();
		            MyVeinsApp::CreateCluster();
		            MyVeinsApp::definingGateway();
	        	}
            }
        }
        
	}

    if(parentMod->par("isClusterHead")){
            findHost()->getDisplayString().updateWith("r=1000,red");
            cDisplayString & displayString=parentMod->getDisplayString();
            displayString.setTagArg("i",0,"veins/node/truck");
    }
    else{
        if(parentMod->par("isGateway")){
            findHost()->getDisplayString().updateWith("r=150,blue");
            cDisplayString & displayString=parentMod->getDisplayString();
            displayString.setTagArg("i",0,"veins/node/car");
        }
        else{
            findHost()->getDisplayString().updateWith("r=0,red");
            cDisplayString & displayString=parentMod->getDisplayString();
            displayString.setTagArg("i",0,"veins/node/car");
        }
    }
}

void MyVeinsApp::findNeighbours(){
    Neighbours.clear();
    //REMOVING THE DISAPPEARED CARS 
    std::vector<int> v;
    cModule *parentMod=getParentModule();
    std::map<int,Coord> ::iterator it;
    for(it=availableCars.begin();it!=availableCars.end();it++){
        EV<<parentMod->getFullName()<<" knows "<<it->first<<endl;
        if(getSimulation()->getModule(it->first)==NULL){
            v.push_back(it->first);
            EV<<parentMod->getFullName()<<" removing "<< it->first<<endl;
        }
    }
    int MinID=parentMod->getId();

    for(auto i:v)
        availableCars.erase(i);
    if(availableCars.find(MinID)!=availableCars.end())
        availableCars.erase(MinID);
    
   
    TraCIMobility* mobility1 = TraCIMobilityAccess().get(parentMod);
    Coord RecieverPosition = mobility1->getPositionAt(simTime());

    
    for(it = availableCars.begin(); it != availableCars.end(); ++it)
    {
        Coord senderPosition = it->second;
        double d=RecieverPosition.distance(senderPosition);
        EV<<parentMod->getFullName()<<"---"<<(getSimulation()->getModule(it->first))->getFullName()<<"---"<<d<<endl;
        if(d<=range)
            Neighbours.insert(it->first);
    }
}

void MyVeinsApp::CreateCluster(){

    std::vector<int> v;
    cModule *parentMod=getParentModule();
    std::map<int,Coord> ::iterator it;
    for(it=availableCars.begin();it!=availableCars.end();it++){
        if(getSimulation()->getModule(it->first)==NULL){
            v.push_back(it->first);
        }
    }
    int MinID=parentMod->getId();

    for(auto i:v)
        availableCars.erase(i);
    if(availableCars.find(MinID)!=availableCars.end())
        availableCars.erase(MinID);

    MyVeinsApp::findNeighbours();
    if(Neighbours.find(MinID)!=Neighbours.end())
        Neighbours.erase(Neighbours.find(MinID));
    for(auto t:Neighbours){
        EV<<MinID<<" "<<t<<endl;
        if(MinID>t){

            MinID=t;
        }
    }
    EV<<MinID<<"["<<parentMod->getId()<<"]"<<endl;

    if((int )(parentMod->getId())==MinID){
        EV<<"making clusterhead"<<endl;
        parentMod->par("firstTime")=false;
        parentMod->par("isClusterHead")=true;
        std::ostringstream str1;
        str1<<simTime();
        parentMod->par("start")=str1.str();
        double temp=(parentMod->getParentModule())->par("noofClusterHead");
        (parentMod->getParentModule())->par("noofClusterHead")=temp+1;
        cTimestampedValue tmp(simTime(),(double)(parentMod->getParentModule())->par("noofClusterHead"));
        emit(countSignal,&tmp);
        parentMod->par("isMarked")=true;
        for(auto t:Neighbours){
            if((getSimulation()->getModule(t))){
                cModule *SenderParentMod=(getSimulation()->getModule(t));
                if(SenderParentMod){
                    bool check=SenderParentMod->par("isClusterHead");
                    if(check==true){
                    	end=simTime();
                    	std::string tempString=(SenderParentMod->par("start"));
                    	EV<<SenderParentMod->getFullName()<<" "<<tempString<<" "<<end<<endl;
                    	Stabilty.open("/home/shashwat-rai/Desktop/DataSets/ClusterStability.txt",std::ios::app);
                    	Stabilty<<SenderParentMod->getFullName()<<" "<<tempString<<" "<<end<<endl;
                        Stabilty.close();
                        SenderParentMod->par("isClusterHead")=false;
                        temp=(parentMod->getParentModule())->par("noofClusterHead");
                        (parentMod->getParentModule())->par("noofClusterHead")=temp-1;
                        cTimestampedValue tmp(simTime(),(double)(parentMod->getParentModule())->par("noofClusterHead"));
                        emit(countSignal,&tmp);
                    }
                    SenderParentMod->par("isMarked")=true;
                    SenderParentMod->par("firstTime")=false;
                    ClusterContent.insert(t);
                }
            }
        }
    }
    EV<<"inside CreateCluster"<<endl;
    std::string s=(parentMod->getFullName());
        bool ch=(parentMod->par("isClusterHead"));
        bool gw=(parentMod->par("isGateway"));
        EV<<s<<" "<<ch<<" "<<gw<<endl;
    for(auto t:Neighbours){
        cModule *SenderParentMod=(getSimulation()->getModule(t));
        if(SenderParentMod){
            EV<<(parentMod->getId())<<" "<<(parentMod->getFullName())<<"---"<<(SenderParentMod->getFullName())<<" "<<(SenderParentMod->getId())<<endl;
        }
    }
}


void MyVeinsApp::definingGateway(){

    std::vector<int> v;
    cModule *parentMod=getParentModule();
    std::map<int,Coord> ::iterator it;
    for(it=availableCars.begin();it!=availableCars.end();it++){
        if(getSimulation()->getModule(it->first)->getParentModule()==NULL){
            v.push_back(it->first);
        }
    }
    int MinID=parentMod->getId();
    if(availableCars.find(MinID)!=availableCars.end())
        availableCars.erase(MinID);

    for(auto i:v)
        availableCars.erase(i);

    MyVeinsApp::findNeighbours();
    if(Neighbours.find(MinID)!=Neighbours.end())
        Neighbours.erase(Neighbours.find(MinID));
    NeighbouringClusterHead.clear();

    for(auto t = Neighbours.begin(); t != Neighbours.end(); ++t){
        if((getSimulation()->getModule(*t))){
           cModule *tempNode=(getSimulation()->getModule(*t));
           if(tempNode !=NULL && (bool)tempNode->par("isClusterHead")==true)
               NeighbouringClusterHead.insert(*t);
        }
    }
    
    if(NeighbouringClusterHead.size()>=2){
        if((bool)(getParentModule())->par("isClusterHead")){
        	end=simTime();
        	std::string tempString=(parentMod->par("start"));
        	EV<<parentMod->getFullName()<<" "<<tempString<<" "<<end<<endl;
	        Stabilty.open("/home/shashwat-rai/Desktop/DataSets/ClusterStability.txt",std::ios::app);
        	Stabilty<<parentMod->getFullName()<<" "<<tempString<<" "<<end<<endl;
        	Stabilty.close();
            double temp=(parentMod->getParentModule())->par("noofClusterHead");
            (parentMod->getParentModule())->par("noofClusterHead")=temp-1;
            cTimestampedValue tmp(simTime(),(double)(parentMod->getParentModule())->par("noofClusterHead"));
            emit(countSignal,&tmp);
        }
       (getParentModule())->par("isClusterHead")=false;
       (getParentModule())->par("isGateway")=true;
       (getParentModule())->par("isMarked")=true;
       (getParentModule())->par("firstTime")=false;
    }
    else{
        //(mod->getParentModule())->par("isClusterHead")=false;
        (getParentModule())->par("isGateway")=false;
        if((bool)(getParentModule())->par("isClusterHead")==false&&NeighbouringClusterHead.size()==0)
            (getParentModule())->par("isMarked")=false;
    }
    EV<<"inside Gateway"<<endl;
    EV<<"ClusterHeads"<<endl;

    for(auto t:NeighbouringClusterHead){
        cModule *SenderParentMod=(getSimulation()->getModule(t));
        if(SenderParentMod){
            EV<<((getParentModule())->getFullName())<<"---"<<(SenderParentMod->getFullName())<<endl;
        }
    }
    EV<<"Neighbours"<<endl;

    for(auto t:Neighbours){
        cModule *SenderParentMod=(getSimulation()->getModule(t));
        if(SenderParentMod){
            EV<<(getParentModule()->getId())<<" "<<(getParentModule()->getFullName())<<"---"<<(SenderParentMod->getFullName())<<" "<<(SenderParentMod->getId())<<endl;
        }
    }

}

void MyVeinsApp::onWSM(BaseFrame1609_4* wsm)
{
    // Your application has received a data message from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
    EV<<"Message received"<<endl;
}

void MyVeinsApp::onWSA(DemoServiceAdvertisment* wsa)
{
    // Your application has received a service advertisement from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
}

void MyVeinsApp::handleSelfMsg(cMessage* msg)
{
    DemoBaseApplLayer::handleSelfMsg(msg);
    // this method is for self messages (mostly timers)
    // it is important to call the DemoBaseApplLayer function for BSM and WSM transmission
    EV<<getParentModule()->getId()<<" "<<getParentModule()->getFullName()<<endl;
    message=new DemoSafetyMessage();
    TraCIMobility* mobility1 = TraCIMobilityAccess().get(getParentModule());
    message->setSenderPos(mobility1->getPositionAt(simTime()));
    message->setNodeId(getId());
    //message sent to down layers
    DemoBaseApplLayer::sendDown(message);

    if(msg==period)
    {
        scheduleAt(simTime() +  10000,period);
    }
    else
    {
        cancelEvent(period);
        scheduleAt(simTime() + 5000,period);
    }
    //MyVeinsApp::definingGateway();
    cModule *parentMod=getParentModule();
    std::string s=(parentMod->getFullName());
    bool ch=(parentMod->par("isClusterHead"));
    bool gw=(parentMod->par("isGateway"));
    EV<<s<<" "<<ch<<" "<<gw<<endl;
}

void MyVeinsApp::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);
    // the vehicle has moved. Code that reacts to new positions goes here.
    // member variables such as currentPosition and currentSpeed are updated in the parent class
}






