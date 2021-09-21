/******************************************************************************
 * narval_oculus driver library for Blueprint Subsea Oculus sonar.
 * Copyright (C) 2020 ENSTA-Bretagne
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *****************************************************************************/

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

inline OculusSimpleFireMessage default_ping_config()
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
    msg.flags |= 0x4;  // Sends gain for each range in returned message
    msg.flags |= 0x8;  // simple ping 
    //msg.flags |= 0x40; // 512 beams ???! (found in sdk)

    msg.speedOfSound    = 0.0;
    msg.salinity        = 0.0;
    
    return msg;
}


inline bool check_config_feedback(const OculusSimpleFireMessage& requested,
                                  const OculusSimpleFireMessage& feedback)
{
    // returns true if feedback coherent with requested config.
    if(requested.pingRate == pingRateStandby) {
        // If in standby, expecting a dummy message
        if(feedback.head.msgId == messageDummy)
            return true;
    }
    else {
        // If got a simple ping result, checking relevant parameters
        if(feedback.head.msgId == messageSimplePingResult
           && requested.masterMode       == feedback.masterMode
           // feedback is broken on pingRate field
           //&& requested.pingRate         == feedback.pingRate 
           && requested.gammaCorrection  == feedback.gammaCorrection
           && requested.flags            == feedback.flags
           && requested.range            == feedback.range
           && std::abs(requested.gainPercent  - feedback.gainPercent)  < 1.0e-1)
        {
            //return true; // bypassing checks on sound speed
            // changing soundspeed is very slow (up to 6 seconds, maybe more)

            // For now simple ping is ok. Checking sound speed / salinity
            // parameters If speed of sound is 0.0, the sonar is using salinity
            // to calculate speed of sound.
            if(requested.speedOfSound != 0.0) {
                if(std::abs(requested.speedOfSound - feedback.speedOfSound) < 1.0e-1)
                    return true;
            }
            else {
                if(std::abs(requested.salinity - feedback.salinity) < 1.0e-1)
                    return true;
            }
        }
    }
    return false;
}
}; //namespace oculus
}; //namespace narval

#endif //_DEF_NARVAL_OCULUS_UTILS_H_
