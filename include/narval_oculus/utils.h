#ifndef _DEF_NARVAL_OCULUS_UTILS_H_
#define _DEF_NARVAL_OCULUS_UTILS_H_

#include <iostream>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/asio.hpp>

#include <narval_oculus/Oculus.h>
#include <narval_oculus/print_utils.h>

namespace narval { namespace oculus {

template <typename EndPointT>
inline EndPointT remote_from_status(const OculusStatusMsg& status)
{
    // going through string conversion allows to not care about
    // endianess. (fix this)
    return EndPointT(boost::asio::ip::address_v4::from_string(
        ip_to_string(status.ipAddr)), 52100);
}

inline OculusSimpleFireMessage default_fire_config()
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
    //msg.pingRate        = pingRateHigh;
    msg.range           = 2.54;
    msg.gainPercent     = 50;
    //msg.flags           = 0x19;
    //msg.flags           = 0x09; // seems to be no difference with 0x19
    //msg.flags           = 0x11; // Ping result not implemented and not in SDK (investigate ?)

    msg.flags  = 0x1;  // always in meters
    //msg.flags |= 0x2;  // 16bits data
    msg.flags |= 0x4;  // gain assist control enabled
    msg.flags |= 0x8;  // simple ping 
    //msg.flags |= 0x40; // 512 beams ???! (found in sdk)

    msg.speedOfSound    = 0.0;
    msg.salinity        = 0.0;
    
    return msg;
}

}; //namespace oculus
}; //namespace narval

#endif //_DEF_NARVAL_OCULUS_UTILS_H_
