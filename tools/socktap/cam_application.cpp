#include "cam_application.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <vanetza/asn1/denm.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <chrono>
#include <exception>
#include <functional>
#include <iostream>
#include <nlohmann/json.hpp>
// This is a very simple CA application sending CAMs at a fixed rate.

using namespace vanetza;
using namespace vanetza::facilities;
using namespace std::chrono;

CamApplication::CamApplication(PositionProvider& positioning, Runtime& rt) :
    positioning_(positioning), runtime_(rt), cam_interval_(seconds(1))
{
    schedule_timer();    
    this->station_id = 1;
    this->server_port = 9000;
    this->serverIP = strdup("192.168.1.125");
}

int CamApplication::createSocket(){

    

    return 0;
}

int CamApplication::closeSocket(){
    return close(this->sockfd);
}

void CamApplication::setSendToServer(bool send_to_server){
    this->send_to_server = send_to_server;
}

void CamApplication::setServerPort(int serverPort){
    this->server_port = serverPort;
}

void CamApplication::setServerIP(const char * serverIP){
    this->serverIP = serverIP;
}







void CamApplication::setStationID(int station_id){
    this->station_id = station_id;
}

int  CamApplication::sendToServer(u_int64_t* dataToSend, int size){
    int sent = sendto(this->sockfd, (const u_int64_t *)dataToSend, size, 
        MSG_CONFIRM, (const struct sockaddr *) &this->servaddr,  
            sizeof(this->servaddr)); 
        std::cout<<"Sending message to UDP Server:" << this->serverIP << " " << this->server_port <<std::endl; 
    return sent;
}

void CamApplication::set_interval(Clock::duration interval)
{
    cam_interval_ = interval;
    runtime_.cancel(this);
    if(cam_interval_<=vanetza::Clock::duration{0}){
        std::cout << "CAM period to low, disabling" << std::endl;
        return;
    }
    
    schedule_timer();
}

void CamApplication::print_generated_message(bool flag)
{
    print_tx_msg_ = flag;
}

void CamApplication::print_received_message(bool flag)
{
    print_rx_msg_ = flag;
}

CamApplication::PortType CamApplication::port()
{
    return btp::ports::CAM;
}

int decodeCAM(const asn1::Cam& recvd, char* message){
    const ItsPduHeader_t& header = recvd->header;
    const CoopAwareness_t& cam = recvd->cam;
    const BasicContainer_t& basic = cam.camParameters.basicContainer;
    const BasicVehicleContainerHighFrequency& bvc = cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
    //int size = sprintf(message, "%ld;%ld;%ld;%ld;ld\n",header.stationID,basic.referencePosition.latitude,basic.referencePosition.longitude,bvc.speed.speedValue,bvc.longitudinalAcceleration.longitudinalAccelerationValue);
    int size = sprintf(
        message, 
        "{\"objectID\":%ld,\"speed\":%ld,\"speedConfidence\":%ld,\"longAcc\":%ld,\"longAccConfidence\":%ld,\"heading\":%ld,\"headingConfidence\":%ld,\"lat\":%ld,\"lon\":%ld,\"length\":%ld,\"lengthConfidence\":%ld,\"lane\":%ld,\"laneConfidence\":%ld,\"altitude\":%ld,\"altitudeConfidence\":%ld,\"vehicleLength\":%ld,\"vehicleLengthConfidence\":%ld,\"positionConfidence\":%ld}\n",
        header.stationID,
        bvc.speed.speedValue,
        bvc.speed.speedConfidence,
        bvc.longitudinalAcceleration.longitudinalAccelerationValue,
        bvc.longitudinalAcceleration.longitudinalAccelerationConfidence,
        bvc.heading.headingValue,
        bvc.heading.headingConfidence,
        basic.referencePosition.latitude,
        basic.referencePosition.longitude,
        bvc.vehicleLength.vehicleLengthValue,
        bvc.vehicleLength.vehicleLengthConfidenceIndication,
        0,
        0,
        0,
        bvc.vehicleLength.vehicleLengthValue,
        bvc.vehicleLength.vehicleLengthConfidenceIndication,
        0,
        0
        );
    return strlen(message);
}

/*void CamApplication::indicate(const DataIndication& indication, UpPacketPtr packet)
{
    printf("Received MEssage\n\n");
    asn1::PacketVisitor<asn1::Cam> visitor;
    std::shared_ptr<const asn1::Cam> cam = boost::apply_visitor(visitor, *packet);

    packet.get();

    std::cout << "CAM application received a packet with " << (cam ? "decodable" : "broken") << " content" << std::endl;
    if (cam && print_rx_msg_) {
        std::cout << "Received CAM contains\n";
        print_indented(std::cout, *cam, "  ", 1);
    }
    
    if(cam && send_to_server){
        std::cout << "sending to udp server" << std::endl;
        char message [500];
        int size = decodeCAM(*cam, message);
        this->sendToServer((u_int64_t*)message, size);
    }
    
}*/
void print_indentedDENM(std::ostream& os, const asn1::Denm& message, const std::string& indent, unsigned level)
{
    auto prefix = [&](const char* field) -> std::ostream& {
        for (unsigned i = 0; i < level; ++i) {
            os << indent;
        }
        os << field << ": ";
        return os;
    };

    // Print ITS PDU Header
    const ItsPduHeader_t& header = message->header;
    prefix("ITS PDU Header") << "\n";
    ++level;
    prefix("Protocol Version") << static_cast<int>(header.protocolVersion) << "\n";
    prefix("Message ID") << static_cast<int>(header.messageID) << "\n";
    prefix("Station ID") << header.stationID << "\n";
    --level;

    // DENM content
   
    // Management Container
    prefix("Management Container") << "\n";
    ++level;
    const ManagementContainer_t& mgmt = message->denm.management;
    
    // ActionID
    prefix("ActionID") << "\n";
    ++level;
    prefix("Originating Station ID") << mgmt.actionID.originatingStationID << "\n";
    prefix("Sequence Number") << mgmt.actionID.sequenceNumber << "\n";
    --level;

    long detectionTimeValue = 0;
    if (asn_INTEGER2long(&mgmt.detectionTime, &detectionTimeValue) == 0) {
        prefix("Detection Time") << detectionTimeValue << "\n";
    } else {
        prefix("Detection Time") << "(invalid INTEGER)" << "\n";
    }

// Reference Time
    long referenceTimeValue = 0;
    if (asn_INTEGER2long(&mgmt.referenceTime, &referenceTimeValue) == 0) {
        prefix("Reference Time") << referenceTimeValue << "\n";
    } else {
        prefix("Reference Time") << "(invalid INTEGER)" << "\n";
    }
    // Event Position
    prefix("Event Position") << "\n";
    ++level;
    prefix("Latitude") << mgmt.eventPosition.latitude << "\n";
    prefix("Longitude") << mgmt.eventPosition.longitude << "\n";

    // Position Confidence Ellipse
    prefix("Position Confidence Ellipse") << "\n";
    ++level;
    prefix("Semi Major Confidence") << mgmt.eventPosition.positionConfidenceEllipse.semiMajorConfidence << "\n";
    prefix("Semi Minor Confidence") << mgmt.eventPosition.positionConfidenceEllipse.semiMinorConfidence << "\n";
    prefix("Semi Major Orientation") << mgmt.eventPosition.positionConfidenceEllipse.semiMajorOrientation << "\n";
    --level;

    // Altitude
    prefix("Altitude") << "\n";
    ++level;
    prefix("Altitude Value") << mgmt.eventPosition.altitude.altitudeValue << "\n";
    prefix("Altitude Confidence") << static_cast<int>(mgmt.eventPosition.altitude.altitudeConfidence) << "\n";
    --level;
    --level; // end event position
    
   if (mgmt.relevanceDistance != nullptr) {
    prefix("Relevance Distance") << static_cast<int>(*mgmt.relevanceDistance) << "\n";
} else {
    prefix("Relevance Distance") << "(null)" << "\n";
}
    prefix("Station Type") << static_cast<int>(mgmt.stationType) << "\n";
    --level; // end management container

    // Situation Container
    prefix("Situation Container") << "\n";
    ++level;
    if (message->denm.situation != nullptr) {
    const SituationContainer_t& situation = *(message->denm.situation);

    prefix("Information Quality") << static_cast<int>(situation.informationQuality) << "\n";

    // Event Type (CauseCode)
    prefix("Event Type") << "\n";
    ++level;
    prefix("Cause Code") << static_cast<int>(situation.eventType.causeCode) << "\n";
    prefix("Sub Cause Code") << static_cast<int>(situation.eventType.subCauseCode) << "\n";
    --level; // end event type

    --level; // end situation container
} else {
    prefix("Situation Container") << "(null)" << "\n";
}
}
void print_indentedCPM(std::ostream& os, const asn1::Cpm& message, const std::string& indent, unsigned level)
{
    auto prefix = [&](const char* field) -> std::ostream& {
        for (unsigned i = 0; i < level; ++i) {
            os << indent;
        }
        os << field << ": ";
        return os;
    };

    // ITS PDU Header
    const ItsPduHeader_t& header = message->header;
    prefix("ITS PDU Header") << "\n";
    ++level;
    prefix("Protocol Version") << static_cast<int>(header.protocolVersion) << "\n";
    prefix("Message ID") << static_cast<int>(header.messageID) << "\n";
    prefix("Station ID") << header.stationID << "\n";
    --level;

    // CPM content
    const CollectivePerceptionMessage_t& cpm = message->cpm;
    prefix("CPM") << "\n";
    ++level;

    prefix("Generation Delta Time") << cpm.generationDeltaTime << "\n";

    // CPM Parameters
    prefix("CPM Parameters") << "\n";
    ++level;

    // Management Container
    prefix("Management Container") << "\n";
    ++level;
    const CpmManagementContainer_t& mgmt = cpm.cpmParameters.managementContainer;

    prefix("Station Type") << static_cast<int>(mgmt.stationType) << "\n";

    prefix("Reference Position") << "\n";
    ++level;
    prefix("Latitude") << mgmt.referencePosition.latitude << "\n";
    prefix("Longitude") << mgmt.referencePosition.longitude << "\n";

    prefix("Position Confidence Ellipse") << "\n";
    ++level;
    prefix("Semi Major Confidence") << mgmt.referencePosition.positionConfidenceEllipse.semiMajorConfidence << "\n";
    prefix("Semi Minor Confidence") << mgmt.referencePosition.positionConfidenceEllipse.semiMinorConfidence << "\n";
    prefix("Semi Major Orientation") << mgmt.referencePosition.positionConfidenceEllipse.semiMajorOrientation << "\n";
    --level;

    prefix("Altitude") << "\n";
    ++level;
    prefix("Altitude Value") << mgmt.referencePosition.altitude.altitudeValue << "\n";
    prefix("Altitude Confidence") << static_cast<int>(mgmt.referencePosition.altitude.altitudeConfidence) << "\n";
    --level; // end altitude
    --level; // end reference position
    --level; // end management container

    // Perceived Object Container
   prefix("Perceived Object Container") << "\n";
    ++level;

    // Iterate over perceived objects (if there are multiple)
const PerceivedObjectContainer_t& perceived_objects = *cpm.cpmParameters.perceivedObjectContainer;

for (int i = 0; i < perceived_objects.list.count; ++i) {
    prefix(("Perceived Object " + std::to_string(i+1)).c_str()) << "\n";
        ++level;
        const PerceivedObject_t& obj = *perceived_objects.list.array[i];

        prefix("Object ID") << obj.objectID << "\n";
        prefix("Time Of Measurement") << obj.timeOfMeasurement << "\n";
        prefix("Object Confidence") << static_cast<int>(obj.objectConfidence) << "\n";

        prefix("X Distance") << "\n";
        ++level;
        prefix("Value") << obj.xDistance.value << "\n";
        prefix("Confidence") << static_cast<int>(obj.xDistance.confidence) << "\n";
        --level;

        prefix("Y Distance") << "\n";
        ++level;
        prefix("Value") << obj.yDistance.value << "\n";
        prefix("Confidence") << static_cast<int>(obj.yDistance.confidence) << "\n";
        --level;

        prefix("Z Distance") << "\n";
        ++level;
        prefix("Value") << obj.zDistance->value << "\n";
        prefix("Confidence") << static_cast<int>(obj.zDistance->confidence) << "\n";
        --level;

        prefix("X Speed") << "\n";
        ++level;
        prefix("Value") << obj.xSpeed.value << "\n";
        prefix("Confidence") << static_cast<int>(obj.xSpeed.confidence) << "\n";
        --level;

        prefix("Y Speed") << "\n";
        ++level;
        prefix("Value") << obj.ySpeed.value << "\n";
        prefix("Confidence") << static_cast<int>(obj.ySpeed.confidence) << "\n";
        --level;

        prefix("X Acceleration") << "\n";
        ++level;
        prefix("Longitudinal Acceleration Value") << obj.xAcceleration->longitudinalAccelerationValue << "\n";
        prefix("Longitudinal Acceleration Confidence") << static_cast<int>(obj.xAcceleration->longitudinalAccelerationConfidence) << "\n";
        --level;

        prefix("Yaw Angle") << "\n";
        ++level;
        prefix("Value") << obj.yawAngle->value << "\n";
        prefix("Confidence") << static_cast<int>(obj.yawAngle->confidence) << "\n";
        --level;

        prefix("Planar Object Dimension 1") << "\n";
        ++level;
        prefix("Value") << obj.planarObjectDimension1->value << "\n";
        prefix("Confidence") << static_cast<int>(obj.planarObjectDimension1->confidence) << "\n";
        --level;

        prefix("Vertical Object Dimension") << "\n";
        ++level;
        prefix("Value") << obj.verticalObjectDimension->value << "\n";
        
        prefix("Confidence") << static_cast<int>(obj.verticalObjectDimension->confidence) << "\n";
        --level;
        prefix("objectRefPoint") << obj.objectRefPoint << "\n";
}
    --level; // end perceived object container
    prefix("Number Of Perceived Objects") << cpm.cpmParameters.numberOfPerceivedObjects << "\n";

    
    --level; // end cpmParameters
    --level; // end CPM
}

void CamApplication::indicate(const DataIndication& indication, UpPacketPtr packet)
{
    printf("Received Message\n\n");

    // Try decode as CAM
    asn1::PacketVisitor<asn1::Cam> camVisitor;
    std::shared_ptr<const asn1::Cam> cam = boost::apply_visitor(camVisitor, *packet);

    // Try decode as DENM
    asn1::PacketVisitor<asn1::Denm> denmVisitor;
    std::shared_ptr<const asn1::Denm> denm = boost::apply_visitor(denmVisitor, *packet);

    // Try decode as CPM
    asn1::PacketVisitor<asn1::Cpm> cpmVisitor;
    std::shared_ptr<const asn1::Cpm> cpm = boost::apply_visitor(cpmVisitor, *packet);

    packet.get();

    if (cam) {
        std::cout << "Received CAM with decodable content" << std::endl;
        if (print_rx_msg_) {
            std::cout << "Received CAM contains\n";
            print_indented(std::cout, *cam, "  ", 1);
            
        }
        if(send_to_server){
            char message [500];
            int size = decodeCAM(*cam, message);
            this->sendToServer((u_int64_t*)message, size);
        }
    }
    else if (denm) {
        std::cout << "Received DENM with decodable content" << std::endl;
        if (print_rx_msg_) {  
            std::cout << "Received DENM contains\n";
              print_indentedDENM(std::cout, *denm, "  ", 1);
            
        }
       
    }
    else if (cpm) {
        
        std::cout << "Received CPM with decodable content" << std::endl;
        if (print_rx_msg_) {
             std::cout << "Received CPM contains\n";
         print_indentedCPM(std::cout, *cpm, "  ", 1);
        
    }

    }   else {
        std::cout << "Received packet with broken or unknown content" << std::endl;
    }
}



void CamApplication::schedule_timer()
{
    runtime_.schedule(cam_interval_, std::bind(&CamApplication::on_timer, this, std::placeholders::_1), this);
}

void CamApplication::on_timer(Clock::time_point)
{
    schedule_timer();

    
    vanetza::asn1::Cam message;

    ItsPduHeader_t& header = message->header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_cam;
    header.stationID = this->station_id; // some dummy value

    const auto time_now = duration_cast<milliseconds>(runtime_.now().time_since_epoch());
    uint16_t gen_delta_time = time_now.count();

    CoopAwareness_t& cam = message->cam;
    cam.generationDeltaTime = gen_delta_time * GenerationDeltaTime_oneMilliSec;

    auto position = positioning_.position_fix();

    if (!position.confidence) {
        std::cerr << "Skipping CAM, because no good position is available, yet." << position.confidence << std::endl;
        return;
    }

    BasicContainer_t& basic = cam.camParameters.basicContainer;
    basic.stationType = StationType_passengerCar;
    copy(position, basic.referencePosition);
   
    cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    BasicVehicleContainerHighFrequency& bvc = cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
    bvc.heading.headingValue = 0;
    bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;

    bvc.speed.speedValue = 0;
    bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec;

    bvc.longitudinalAcceleration.longitudinalAccelerationValue = 0;
    bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_pointOneMeterPerSecSquared;

    bvc.driveDirection = DriveDirection_forward;
    bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;

    bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
    bvc.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_noTrailerPresent;
    bvc.vehicleWidth = VehicleWidth_unavailable;

    bvc.curvature.curvatureValue = 0;
    bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
    bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;

    bvc.yawRate.yawRateValue = YawRateValue_unavailable;

    std::string error;
    if (!message.validate(error)) {
        throw std::runtime_error("Invalid high frequency CAM: %s" + error);
    }

    if (print_tx_msg_) {
        std::cout << "Generated CAM contains\n";
        print_indented(std::cout, message, "  ", 1);
    }

    DownPacketPtr packet { new DownPacket() };
    packet->layer(OsiLayer::Application) = std::move(message);

    DataRequest request;
    request.its_aid = aid::CA;
    request.transport_type = geonet::TransportType::SHB;
    request.communication_profile = geonet::CommunicationProfile::ITS_G5;

    auto confirm = Application::request(request, std::move(packet));
    if (!confirm.accepted()) {
        throw std::runtime_error("CAM application data request failed");
    }
}
