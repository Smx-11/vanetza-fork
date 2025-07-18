#include "its_application.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/denm.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <chrono>
#include <exception>
#include <functional>
#include <iostream>
#include <boost/asio/io_service.hpp>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
// This is a very simple CA application sending CAMs at a fixed rate.

using namespace vanetza;
using namespace vanetza::facilities;
using namespace std::chrono;
namespace asio = boost::asio;

ITSApplication::ITSApplication(PositionProvider& positioning, Runtime& rt, asio::io_service& io_service, unsigned short denm_port) :
    positioning_(positioning), runtime_(rt), cam_interval_(seconds(1)),
    denm_socket(io_service, asio::ip::udp::endpoint(asio::ip::udp::v4(), int(9001))),
    cam_socket(io_service)
{
    schedule_timer();    
    this->station_id = 1;
    this->server_port = 9000;
    this->serverIP = strdup("192.168.1.100");
    createSocket();
    this->start_receive();
	
}
void ITSApplication::sendCAMToServer(const std::string& data, int size) {
    std::cout << "sending to server" << std::endl;
    if (!cam_socket.is_open()) {
    std::cerr << "Socket not open!" << std::endl;
    return;
}

    auto bufferCopy = std::make_shared<std::string>(data);  // keep copy alive

   // Capture cam_endpoint by value in the lambda (along with bufferCopy)
auto endpoint = this->cam_endpoint;  // copy the endpoint

    this->cam_socket.async_send_to(
        asio::buffer(bufferCopy->data(), size),
        endpoint,
        [bufferCopy, endpoint](const std::error_code& ec, std::size_t bytes_sent) {
            if (!ec) {
                std::cout << "Sent " << bytes_sent << " bytes to "
                        << endpoint.address().to_string()
                        << ":" << endpoint.port() << std::endl;
            } else {
                std::cerr << "Send failed: " << ec.message() << std::endl;
            }
        }
    );

}


void ITSApplication::create_CPM(const json& j){
    vanetza::asn1::Cpm cpmmessage;
    ItsPduHeader_t& header = cpmmessage->header;
    header.protocolVersion = 2;
    header.messageID = 14; //header for cpm and to see on wireshark
    header.stationID = this->station_id;

    CollectivePerceptionMessage_t& cpm = cpmmessage->cpm;

    const auto time_now = duration_cast<milliseconds>(runtime_.now().time_since_epoch());
    uint16_t gen_delta_time = time_now.count();

    cpm.generationDeltaTime = gen_delta_time * GenerationDeltaTime_oneMilliSec;
    CpmParameters_t& cpmparams = cpm.cpmParameters;
    //managementContainer
    CpmManagementContainer_t& management = cpmparams.managementContainer;
    management.stationType = StationType_passengerCar;

    //dummy values
    management.referencePosition.altitude.altitudeValue = 0;
    management.referencePosition.latitude = 1;
    management.referencePosition.longitude = 2;
    management.referencePosition.positionConfidenceEllipse.semiMajorOrientation=0;
    management.referencePosition.altitude.altitudeValue=0;
   
  
    //perceivedObjectContainer
    cpmparams.perceivedObjectContainer=vanetza::asn1::allocate<PerceivedObjectContainer_t>();
    
     
   for (const auto& obj : j) {

        //  PerceivedObject_t* asn_obj = (PerceivedObject_t*)calloc(1, sizeof(PerceivedObject_t));
        auto asn_obj = vanetza::asn1::allocate<PerceivedObject_t>();
        
        // 1. objectID (convert "obj-001" → 1)
        std::string objIDStr = obj.value("objectID", "0");
        asn_obj->objectID = std::stoi(objIDStr.substr(objIDStr.find_last_of('-') + 1));

        // 2. TimeOfMeasurement (optional: set to now or 0)
        asn_obj->timeOfMeasurement = 0;  // set appropriately if needed
    
        // 3. x/y Distance (lat/lon scaled)
        asn_obj->xDistance.value = obj.value("lat", 0);  // WGS84 lat scaled
        asn_obj->xDistance.confidence = obj.value("positionConfidence", 100);

        asn_obj->yDistance.value = obj.value("lon", 0);
        asn_obj->yDistance.confidence = obj.value("positionConfidence", 100);

        // 4. zDistance (altitude, optional)
        if (obj.contains("altitude")) {
            //asn_obj->zDistance = (ObjectDistanceWithConfidence_t*)calloc(1, sizeof(ObjectDistanceWithConfidence_t));
            asn_obj->zDistance = vanetza::asn1::allocate<ObjectDistanceWithConfidence_t>();
            
            asn_obj->zDistance->value = obj.value("altitude", 0.0); 
            asn_obj->zDistance->confidence = obj.value("altitudeConfidence", 100);
        }

        // 5. Speed
        asn_obj->xSpeed.value = obj.value("speed", 0);;  
        asn_obj->xSpeed.confidence = obj.value("speedConfidence", 100);

        asn_obj->ySpeed.value = 0;
        asn_obj->ySpeed.confidence = 100;

        // 6. Acceleration (longAcc → xAcceleration)
        if (obj.contains("longAcc")) {
        // asn_obj->xAcceleration = (LongitudinalAcceleration_t*)calloc(1, sizeof(LongitudinalAcceleration_t));
            asn_obj->xAcceleration = vanetza::asn1::allocate<LongitudinalAcceleration_t>();
            asn_obj->xAcceleration->longitudinalAccelerationValue = obj.value("longAcc", 0.0);  // m/s² → scaled
            asn_obj->xAcceleration->longitudinalAccelerationConfidence = obj.value("longAccConfidence", 100);
        }

        // 7. Heading (mapped to yawAngle)
        if (obj.contains("heading")) {
        // asn_obj->yawAngle = (CartesianAngle_t*)calloc(1, sizeof(CartesianAngle_t));
        asn_obj->yawAngle = vanetza::asn1::allocate<CartesianAngle_t>();
            asn_obj->yawAngle->value = 0;  // degrees → 0.01 deg
            asn_obj->yawAngle->confidence = obj.value("headingConfidence", 100);
        }

        // 8. Object dimension (length, etc.)
        if (obj.contains("length")) {
            //asn_obj->planarObjectDimension1 = (ObjectDimension_t*)calloc(1, sizeof(ObjectDimension_t));
            asn_obj->planarObjectDimension1 = vanetza::asn1::allocate<ObjectDimension_t>();
        
            asn_obj->planarObjectDimension1->value =obj.value("length", 0.0); // meters → cm
            asn_obj->planarObjectDimension1->confidence = obj.value("lengthConfidence", 100);
        }
        
        // 9. Vertical dimension (vehicleLength)
        if (obj.contains("vehicleLength")) {
            //asn_obj->verticalObjectDimension = (ObjectDimension_t*)calloc(1, sizeof(ObjectDimension_t));
            asn_obj->verticalObjectDimension = vanetza::asn1::allocate<ObjectDimension_t>();
        
            asn_obj->verticalObjectDimension->value = obj.value("vehicleLength", 0.0);
            asn_obj->verticalObjectDimension->confidence = obj.value("vehicleLengthConfidence", 100);
        }
        // 10. Object confidence
        asn_obj->objectConfidence = 0;

        // 11. Set objectRefPoint explicitly (even if default in ASN.1)
        asn_obj->objectRefPoint = 0;


        // 12. Add to ASN.1 sequence
        ASN_SEQUENCE_ADD(&cpmparams.perceivedObjectContainer->list, asn_obj);
    }

    //number of perceivedobjects value
    cpmparams.numberOfPerceivedObjects = cpmparams.perceivedObjectContainer->list.count;


    std::cout << "Generated Full CPM contains:\n";
    asn_fprint(stdout, &asn_DEF_CPM, cpmmessage.operator->());

    DownPacketPtr packet { new DownPacket() };
    packet->layer(OsiLayer::Application) = std::move(cpmmessage);
    DataRequest request;
    request.its_aid = aid::CP;
    request.transport_type = geonet::TransportType::SHB;
    request.communication_profile = geonet::CommunicationProfile::ITS_G5;
 
    //print_indented_denm(std::cout, message, "  ", 1);
    
    try {
    auto confirm = Application::request(request, std::move(packet));
    if (!confirm.accepted()) {
        throw std::runtime_error("CPM application data request failed");
    }
    } catch(std::runtime_error& e) {
        std::cout << "-- Vanetza UPER Encoding Error --\nCheck that the message format follows ETSI spec\n" << e.what() << std::endl;
        
        
    }
}
void ITSApplication::handle_receive_error(const std::error_code& error){
    std::cerr << "Receive error: " << error.message() << std::endl;
    
}
void ITSApplication::handle_message(std::size_t bytes_transferred){

    std::string data(this->recv_buffer.data(), bytes_transferred);  // Only use valid part of buffer
    std::cout << "[Received UDP data]: " << data << std::endl; 
    try {
        // Parse JSON from received data
        nlohmann::json proto2json = nlohmann::json::parse(data);
        
        // print json
        std::cout << "[Parsed proto2 received]:\n" << proto2json.dump(4) << std::endl;
        if(!proto2json["objects"].empty()){
            //cpm
            this->create_CPM(proto2json["objects"]);
        }
        if (!proto2json["events"].empty()) {
    const auto& events = proto2json["events"];
    if (events.is_array()) {
        for (const auto& event : events) {
            // Create a JSON object with the single event
            nlohmann::json singleEventJson = proto2json;  // copy original JSON
            singleEventJson["events"] = event;            // replace with single event
            //oi
            std::cout << "Sending DENM for one event" << std::endl;
            this->sendDenm(singleEventJson);
        }
    } else if (events.is_object()) {
        std::cout << "Sending DENM for single event" << std::endl;
        this->sendDenm(proto2json);
    } else {
        std::cerr << "events is neither array nor object" << std::endl;
    }
}
       /*if (proto2json["proto2Objects"].empty()) {
            //objects empty is event object -> send DENM
            std::cout << "Only proto2Event filled " << std::endl;
            this->sendDenm(proto2json);
        }else if(proto2json["proto2Events"].empty()){
            //gonna be objects so prepare CPM
            
        }*/       
    } catch (nlohmann::json::parse_error& e) {
        std::cerr << "JSON parse error: " << e.what() << std::endl;
    }
}

void ITSApplication::start_receive(){
    
    this->denm_socket.async_receive_from(
        asio::buffer(this->recv_buffer), this->remote_endpoint,
        [this](std::error_code error, std::size_t bytes_transferred)
        {
            if (error || bytes_transferred == 0) {
                this->handle_receive_error(error);
                return;
            }
            this->handle_message(bytes_transferred);
            start_receive();  // Keep listening
        });
}

void ITSApplication::populateStruct(char* data, Denm_Data* denm_data, int index){
     switch (index)
    {
        case 0:
            denm_data->type = atoi(data);
            break;
        case 1:
            denm_data->lat = atoi(data);
            break;
        case 2:
            denm_data->lon = atoi(data);
            break;
        default:
            break;
    }
}

int ITSApplication::createSocket(){
    
    //this->cam_socket = asio::ip::udp::socket socket(io_service);
    cam_socket.open(asio::ip::udp::v4());
   this->cam_endpoint = asio::ip::udp::endpoint(asio::ip::address::from_string(this->serverIP), this->server_port);
   return 0;
}



void ITSApplication::setSendToServer(bool send_to_server){
    this->send_to_server = send_to_server;
}

void ITSApplication::setServerPort(int serverPort){
    this->server_port = serverPort;
}

void ITSApplication::setServerIP(const char * serverIP){
    this->serverIP = serverIP;
}


void ITSApplication::setStationID(int station_id){
    this->station_id = station_id;
}

void  ITSApplication::sendToServer(u_int64_t* dataToSend, int size){
    this->cam_socket.async_send_to(
            asio::buffer(dataToSend, size),
            this->cam_endpoint,
            [dataToSend](const std::error_code& ec, std::size_t bytes_sent) {
                if (!ec) {
                    std::cout << "Async sent "<< bytes_sent << " bytes)" << std::endl;
                } else {
                    std::cerr << "Send failed: " << ec.message() << std::endl;
                }
            }
    );
}

void ITSApplication::set_interval(Clock::duration interval)
{
    cam_interval_ = interval;
    runtime_.cancel(this);
    if(cam_interval_<=vanetza::Clock::duration{0}){
        std::cout << "CAM period to low, disabling" << std::endl;
        return;
    }
    
    schedule_timer();
}

void ITSApplication::print_generated_message(bool flag)
{
    print_tx_msg_ = flag;
}

void ITSApplication::print_received_message(bool flag)
{
    print_rx_msg_ = flag;
}

ITSApplication::PortType ITSApplication::port()
{
    return btp::ports::CAM;
}

int decode(const asn1::Cam& recvd, char* message){
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

void ITSApplication::indicate(const DataIndication& indication, UpPacketPtr packet)
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
        char message [500];
        int size = decode(*cam, message);
         json j = json::parse(message);  // parse the inner object

        json proto2;
        proto2["proto2Objects"] = json::array();    // create array
        proto2["proto2Objects"].push_back(j);       // add your object inside array

        proto2["proto2Event"] = json::object();     // add empty proto2Event object

        std::string jsonStr = proto2.dump();    
        this->sendCAMToServer(jsonStr,jsonStr.size());
        //this->sendToServer((u_int64_t*)message, size);
    }
    
}

void ITSApplication::schedule_timer()
{
    runtime_.schedule(cam_interval_, std::bind(&ITSApplication::on_timer, this, std::placeholders::_1), this);
}

void ITSApplication::on_timer(Clock::time_point)
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

void ITSApplication::sendDenm(const json& j){

   // printf("sending denm: %ld %ld %d\n", denm_data->type, denm_data->lat, denm_data->lon );
    const auto& proto2event = j["events"];
    int counter  = 1;
    vanetza::asn1::Denm message;

    // Header	
    ItsPduHeader_t& header = message->header;
	header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_denm;
    header.stationID = 1;
	

    // Management
    ManagementContainer_t& management = message->denm.management;
    //action id
    //management.actionID.originatingStationID = 1;
    management.actionID.originatingStationID = atoi(proto2event.value("origin", "1").c_str());
    management.actionID.sequenceNumber = counter;


        //detection time
    const auto time_now = duration_cast<milliseconds>(runtime_.now().time_since_epoch());
    //uint64_t time = 45000000000;
   /* uint64_t time = time_now.count();
    management.detectionTime.buf = (uint8_t*) malloc(sizeof(uint64_t));
    management.detectionTime.size = sizeof(uint64_t);
    
    for (size_t i = 0; i < management.detectionTime.size; ++i) {
        management.detectionTime.buf[i] = (time >> (8 * (management.detectionTime.size - 1 - i))) & 0xFF;
    }

    //reference time
   // uint64_t ref_time = 45000000000;
    uint64_t ref_time = time_now.count();
    management.referenceTime.buf = (uint8_t*) malloc(sizeof(uint64_t));
    management.referenceTime.size = sizeof(uint64_t);

    for (size_t i = 0; i < management.referenceTime.size; ++i) {
        management.referenceTime.buf[i] = (ref_time >> (8 * (management.referenceTime.size - 1 - i))) & 0xFF;
    }*/

    //detectionTime and referenceTime are set using asn_long2INTEGER() — no manual malloc/free, so no leaks there.
    INTEGER_t* detectionTime = &management.detectionTime;
    long timeValue = time_now.count();
    int ret = asn_long2INTEGER(detectionTime, timeValue);
    if (ret != 0) {
        throw std::runtime_error("Failed to set detectionTime integer");
    }

    // Same for referenceTime
    INTEGER_t* referenceTime = &management.referenceTime;
    ret = asn_long2INTEGER(referenceTime, timeValue);
    if (ret != 0) {
        throw std::runtime_error("Failed to set referenceTime integer");
    }
    //pos
    /*auto pos = positioning_.position_fix();
    if (!pos.confidence) {
        std::cerr << "Skipping DENM, no valid position" << std::endl;
        return;
    }
    copy(pos, management.eventPosition);*/

    //management event position
    management.eventPosition.altitude.altitudeValue= proto2event.value("altitude", 0);
    management.eventPosition.latitude= proto2event.value("lat", 0);
    management.eventPosition.longitude= proto2event.value("lon", 0);

    //xxx possible memory leak
   management.relevanceDistance = vanetza::asn1::allocate<RelevanceDistance_t>();
    if (management.relevanceDistance == nullptr) {
        // Handle malloc failure if needed
        // e.g., log error, return, throw, etc.
    }

    // Assign the correct enum value based on radius
    int radius = proto2event.value("radius", 0);

    if (radius < 50) {
        *management.relevanceDistance = RelevanceDistance_lessThan50m;
    } else if (radius < 100) {
        *management.relevanceDistance = RelevanceDistance_lessThan100m;
    } else if (radius < 200) {
        *management.relevanceDistance = RelevanceDistance_lessThan200m;
    } else if (radius < 500) {
        *management.relevanceDistance = RelevanceDistance_lessThan500m;
    } else if (radius < 1000) {
        *management.relevanceDistance = RelevanceDistance_lessThan1000m;
    } else if (radius < 5000) {
        *management.relevanceDistance = RelevanceDistance_lessThan5km;
    } else if (radius < 10000) {
        *management.relevanceDistance = RelevanceDistance_lessThan10km;
    } else {
        *management.relevanceDistance = RelevanceDistance_over10km;
    }
    
    management.stationType = StationType_passengerCar;

    SituationContainer* situation = vanetza::asn1::allocate<SituationContainer_t>();
   // situation->eventType.causeCode = 9;
    
    const std::string eventTypeStr = proto2event.value("eventType", "unknown");

    int causeCode = 0;  // default code if not recognized

    if (eventTypeStr == "speeding") {
        causeCode = 7;
    } else if (eventTypeStr == "accident") {
        causeCode = 10;
    } else if (eventTypeStr == "roadwork") {
        causeCode = 12;
    }
// add other mappings as needed

    situation->eventType.causeCode = causeCode;
    situation->eventType.subCauseCode = 0;
    message->denm.situation = situation;
     //print generated DENM
    std::cout << "Generated DENM contains\n";
    asn_fprint(stdout, &asn_DEF_DENM,message.operator->());
        
    std::string error;
	if (!message.validate(error)) {
		throw std::runtime_error("Invalid DENM: " + error);
	}
    
    // message is moved, so no need to worry about manual cleanups here
    // After this point, the ownership of `message` data has transferred to `packet`,
    // and it will be cleaned up automatically when `packet` is destroyed.
    // No further manual freeing of `message` resources is necessary.
    DownPacketPtr packet { new DownPacket() };
    packet->layer(OsiLayer::Application) = std::move(message);
    DataRequest request;
    request.its_aid = aid::DEN;
    request.transport_type = geonet::TransportType::SHB;
    request.communication_profile = geonet::CommunicationProfile::ITS_G5;

    //print_indented_denm(std::cout, message, "  ", 1);

    auto confirm = Application::request(request, std::move(packet));
    if (!confirm.accepted()) {
        throw std::runtime_error("DENM application data request failed");
    }


}

