#include <iostream>
#include <sstream>
using namespace std;

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>
namespace asio = boost::asio;

#include <narval_oculus/Oculus.h>

std::ostream& operator<<(std::ostream& os, const OculusMessageHeader& msg);
std::ostream& operator<<(std::ostream& os, const OculusStatusMsg& msg);

class StatusListener
{
    public:

    using Socket   = boost::asio::ip::udp::socket;
    using EndPoint = boost::asio::ip::udp::endpoint;

    protected:

    Socket          socket_;
    EndPoint        remote_;
    OculusStatusMsg msg_;

    public:

    StatusListener(boost::asio::io_service& service, unsigned short listeningPort = 52102) :
        socket_(service),
        remote_(asio::ip::address_v4::any(), listeningPort)
    {
        boost::system::error_code err;
        socket_.open(boost::asio::ip::udp::v4(), err);

        socket_.set_option(boost::asio::socket_base::broadcast(true));
        if(err)
            throw std::runtime_error("Error opening socket");

        socket_.bind(remote_);
        if(err)
            throw std::runtime_error("Socket remote error");

        cout << "listening to remote : " << remote_ << endl;
    }

    void listen()
    {
        socket_.async_receive(boost::asio::buffer(static_cast<void*>(&msg_), sizeof(msg_)),
                              boost::bind(&StatusListener::read_callback, this, _1, _2));
    }

    void read_callback(const boost::system::error_code& err, std::size_t bytesReceived)
    {
        if(err) {
            cout << "Reception error" << endl;
            return;
        }
        if(bytesReceived != sizeof(OculusStatusMsg)) {
            cout << "Did not received enough bytes" << endl;
            return;
        }
        
        // we are clean here
        cout << msg_ << endl;
        this->listen();
    }
};

int main()
{
    asio::io_service ioService;
    StatusListener listener(ioService);

    listener.listen();

    ioService.run();

    getchar();

    return 0;
}


std::ostream& operator<<(std::ostream& os, const OculusMessageHeader& msg)
{
    const std::string prefix("\n  ");
    os << "Oculus message Header : "
       << prefix << "oculusId    : " << msg.oculusId
       << prefix << "srcDeviceId : " << msg.srcDeviceId
       << prefix << "dstDeviceId : " << msg.dstDeviceId
       << prefix << "msgId       : " << msg.msgId
       << prefix << "msgVersion  : " << msg.msgVersion
       << prefix << "payloadSize : " << msg.payloadSize
       << prefix << "spare2      : " << msg.spare2 << "\n";
    return os;
}

void print_ipv4(std::ostream& os, uint32_t ip)
{
    os << (ip & 0x000000ff)         << "."
       << ((ip & 0x0000ff00) >>  8) << "."
       << ((ip & 0x00ff0000) >> 16) << "."
       << ((ip & 0xff000000) >> 24);
}

void print_mac(std::ostream& os, const uint8_t* mac)
{
    std::ostringstream oss;
    oss << std::hex << (unsigned int)mac[0] << ":"
                    << (unsigned int)mac[1] << ":"
                    << (unsigned int)mac[2] << ":"
                    << (unsigned int)mac[3] << ":"
                    << (unsigned int)mac[4] << ":"
                    << (unsigned int)mac[5];
    os << oss.str();
}

std::ostream& operator<<(std::ostream& os, const OculusStatusMsg& msg)
{
    const std::string prefix("\n  ");
    
    os << "Oculus status message :"
       << msg.hdr
       << prefix << "deviceId        : " << msg.deviceId
       << prefix << "status          : " << msg.status
       << prefix << "part number     : " << msg.partNumber
       << prefix << "ipAddr          : "; print_ipv4(os, msg.ipAddr); os
       << prefix << "ipMask          : "; print_ipv4(os, msg.ipMask); os
       << prefix << "connectedIpAddr : "; print_ipv4(os, msg.connectedIpAddr); os
       << prefix << "macAddr         : "; print_mac(os, &msg.macAddr0); os
       << prefix << "temperature0    : " << msg.temperature0
       << prefix << "temperature1    : " << msg.temperature1
       << prefix << "temperature2    : " << msg.temperature2
       << prefix << "temperature3    : " << msg.temperature3
       << prefix << "temperature4    : " << msg.temperature4
       << prefix << "temperature5    : " << msg.temperature5
       << prefix << "temperature6    : " << msg.temperature6
       << prefix << "temperature7    : " << msg.temperature7
       << prefix << "pressure        : " << msg.pressure
       << "\n";

    //OculusDeviceType   deviceType;
    //OculusPartNumberType partNumber;
    //uint32_t   status;
    //OculusVersionInfo versinInfo;

    return os;
}

