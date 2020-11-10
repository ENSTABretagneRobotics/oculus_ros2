#ifndef _NARVAL_OCULUS_CLIENT_H_
#define _NARVAL_OCULUS_CLIENT_H_

#include <iostream>
#include <cstring>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <narval_oculus/Oculus.h>
#include <narval_oculus/utils.h>
#include <narval_oculus/CallbackQueue.h>
#include <narval_oculus/StatusListener.h>

#include <narval_oculus/print_utils.h>

namespace narval { namespace oculus {

class Client
{
    public:

    using Socket     = boost::asio::ip::tcp::socket;
    using EndPoint   = boost::asio::ip::tcp::endpoint;

    static EndPoint remote_from_status(const OculusStatusMsg& status)
    {
        // going through string conversion allows to not care about
        // endianess. (fix this)
        return EndPoint(boost::asio::ip::address_v4::from_string(
            ip_to_string(status.ipAddr)), 52100);
    }

    static OculusSimpleFireMessage default_configuration()
    {
        OculusSimpleFireMessage msg;
        std::memset(&msg, 0, sizeof(msg));

        msg.head.oculusId    = OCULUS_CHECK_ID;
        msg.head.msgId       = messageSimpleFire;
        msg.head.srcDeviceId = 0;
        msg.head.dstDeviceId = 0;
        msg.head.payloadSize = sizeof(OculusSimpleFireMessage) - sizeof(OculusMessageHeader);

        msg.masterMode      = 2;
        msg.networkSpeed    = 0xff;
        msg.gammaCorrection = 127;
        msg.pingRate        = pingRateNormal;
        msg.range           = 2;
        msg.gainPercent     = 50;
        msg.flags           = 0x19;
        msg.speedOfSound    = 0.0;
        msg.salinity        = 0.0;
        
        return msg;
    }

    protected:

    Socket   socket_;
    EndPoint remote_;
    
    StatusListener             statusListener_;
    StatusListener::CallbackId statusCallbackId_;

    uint16_t sourceDevice_;

    OculusMessageHeader initialHeader_;

    public:

    Client(boost::asio::io_service& service);

    void on_first_status(const OculusStatusMsg& msg);
    void on_connect(const boost::system::error_code& err);

    void send_config(const OculusSimpleFireMessage& config);

    void initiate_receive();
    void initiate_callback(const boost::system::error_code err,
                           std::size_t receivedByteCount);

    bool validate_header(const OculusMessageHeader& header);
    bool connected() const;
};

}; //namespace oculus
}; //namespace narval

#endif //_NARVAL_OCULUS_CLIENT_H_
