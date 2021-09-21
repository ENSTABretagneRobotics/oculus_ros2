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

#include <narval_oculus/print_utils.h>

namespace narval { namespace oculus {

std::string ip_to_string(uint32_t ip)
{
    std::ostringstream oss;
    oss <<  (ip & 0x000000ff)        << "."
        << ((ip & 0x0000ff00) >>  8) << "."
        << ((ip & 0x00ff0000) >> 16) << "."
        << ((ip & 0xff000000) >> 24);
    return oss.str();
}

std::string mac_to_string(const uint8_t* mac)
{
    std::ostringstream oss;
    oss << std::hex << (unsigned int)mac[0] << ":"
                    << (unsigned int)mac[1] << ":"
                    << (unsigned int)mac[2] << ":"
                    << (unsigned int)mac[3] << ":"
                    << (unsigned int)mac[4] << ":"
                    << (unsigned int)mac[5];
    return oss.str();
}

std::string to_string(DataSizeType dataType)
{
    switch(dataType)
    {
        case dataSize8Bit:
            return "8bit";
        case dataSize16Bit:
            return "16bit";
        case dataSize24Bit:
            return "24bit";
        case dataSize32Bit:
            return "32bit";
        default:
            return "invalid";
    }
}

std::string to_string(PingRateType pingRate)
{
    switch(pingRate)
    {
        case pingRateNormal:
            return "normal (10Hz)";
        case pingRateHigh:
            return "high (15Hz)";
        case pingRateHighest:
            return "highest (40Hz)";
        case pingRateLow:
            return "low (5Hz)";
        case pingRateLowest:
            return "lowest (2Hz)";
        case pingRateStandby:
            return "standby (0Hz)";
        default:
            return "invalid (" + std::to_string(pingRate) + ")";
    }
}

std::string to_string(OculusPartNumberType partNumber)
{
    switch(partNumber)
    {
        case partNumberM370s:
            return "M370s";
        case partNumberM370s_Artemis:
            return "M370s_Artemis";
        case partNumberM370s_Deep:
            return "M370s_Deep";
        case partNumberM373s:
            return "M373s";
        case partNumberM373s_Deep:
            return "M373s_Deep";
        case partNumberM750d:
            return "M750d";
        case partNumberM750d_Fusion:
            return "M750d_Fusion";
        case partNumberM750d_Artemis:
            return "M750d_Artemis";
        case partNumberM1200d:
            return "M1200d";
        case partNumberM1200d_Deep:
            return "M1200d_Deep";
        case partNumberM1200d_Artemis:
            return "M1200d_Artemis";
        case partNumberN1200s:
            return "N1200s";
        case partNumberN1200s_Deep:
            return "N1200s_Deep";
        default:
            return "unknown";
    }
}

std::string to_string(const OculusMessageHeader& msg, const std::string& prefix)
{
    std::ostringstream oss;
    oss << prefix << "oculusId    : " << msg.oculusId
        << prefix << "srcDeviceId : " << msg.srcDeviceId
        << prefix << "dstDeviceId : " << msg.dstDeviceId
        << prefix << "msgId       : " << msg.msgId
        << prefix << "msgVersion  : " << msg.msgVersion
        << prefix << "payloadSize : " << msg.payloadSize
        << prefix << "spare2      : " << msg.spare2;
    return oss.str();
}

std::string to_string(const OculusStatusMsg& msg, const std::string& prefix)
{
    std::ostringstream oss;
    oss << prefix << "deviceId        : " << msg.deviceId
        << prefix << "status          : " << msg.status
        << prefix << "part number     : " << msg.partNumber
        << prefix << "ipAddr          : " << ip_to_string(msg.ipAddr)
        << prefix << "ipMask          : " << ip_to_string(msg.ipMask)
        << prefix << "connectedIpAddr : " << ip_to_string(msg.connectedIpAddr)
        << prefix << "macAddr         : " << mac_to_string(&msg.macAddr0)
        << prefix << "temperature0    : " << msg.temperature0
        << prefix << "temperature1    : " << msg.temperature1
        << prefix << "temperature2    : " << msg.temperature2
        << prefix << "temperature3    : " << msg.temperature3
        << prefix << "temperature4    : " << msg.temperature4
        << prefix << "temperature5    : " << msg.temperature5
        << prefix << "temperature6    : " << msg.temperature6
        << prefix << "temperature7    : " << msg.temperature7
        << prefix << "pressure        : " << msg.pressure;
    return oss.str();
}

std::string to_string(const OculusSimpleFireMessage& msg, const std::string& prefix)
{
    std::ostringstream oss;
    oss << prefix << "masterMode      : " << (int)msg.masterMode
        << prefix << "pingRate        : " << msg.pingRate
        << prefix << "networkSpeed    : " << (int)msg.networkSpeed
        << prefix << "gammaCorrection : " << (int)msg.gammaCorrection
        << prefix << "flags           : " << std::hex <<(int)msg.flags
        << prefix << "range           : " << msg.range
        << prefix << "gainPercent     : " << msg.gainPercent
        << prefix << "speedOfSound    : " << msg.speedOfSound
        << prefix << "salinity        : " << msg.salinity;
    return oss.str();
}

std::string to_string(const OculusSimplePingResult& msg, const std::string& prefix)
{
    std::ostringstream oss;
    oss << prefix << "pingId            : " <<  msg.pingId
        << prefix << "status            : " <<  msg.status
        << prefix << "frequency         : " <<  msg.frequency
        << prefix << "temperature       : " <<  msg.temperature
        << prefix << "pressure          : " <<  msg.pressure
        << prefix << "speeedOfSoundUsed : " <<  msg.speeedOfSoundUsed
        << prefix << "pingStartTime     : " <<  msg.pingStartTime
        << prefix << "dataSize          : " <<  msg.dataSize
        << prefix << "rangeResolution   : " <<  msg.rangeResolution
        << prefix << "nRanges           : " <<  msg.nRanges
        << prefix << "nBeams            : " <<  msg.nBeams
        << prefix << "imageOffset       : " <<  msg.imageOffset
        << prefix << "imageSize         : " <<  msg.imageSize
        << prefix << "messageSize       : " <<  msg.messageSize;
    return oss.str();
}

}; //namespace oculus
}; //namespace narval

std::ostream& operator<<(std::ostream& os, DataSizeType dataType)
{
    os << narval::oculus::to_string(dataType);
    return os;
}

std::ostream& operator<<(std::ostream& os, PingRateType pingRate)
{
    os << narval::oculus::to_string(pingRate);
    return os;
}

std::ostream& operator<<(std::ostream& os, OculusPartNumberType partNumber)
{
    os << narval::oculus::to_string(partNumber);
    return os;
}

std::ostream& operator<<(std::ostream& os, const OculusMessageHeader& msg)
{
    os << "OculusMessageHeader :" << narval::oculus::to_string(msg);
    return os;
}

std::ostream& operator<<(std::ostream& os, const OculusStatusMsg& msg)
{
    const std::string prefix("\n  - ");
    
    os << "OculusStatusMsg :"
       << "\n- header :" << narval::oculus::to_string(msg.hdr, prefix)
       << "\n- status :" << narval::oculus::to_string(msg, prefix);
    return os;
}

std::ostream& operator<<(std::ostream& os, const OculusSimpleFireMessage& msg)
{
    const std::string prefix("\n  - ");
    os << "OculusSimpleFireMessage :"
       << "\n- header :"      << narval::oculus::to_string(msg.head, prefix)
       << "\n- simple fire :" << narval::oculus::to_string(msg, prefix);
    return os;
}

std::ostream& operator<<(std::ostream& os, const OculusSimplePingResult& msg)
{
    const std::string prefix("\n  - ");
    os << "OculusSimplePingMessage :"
       << "\n- header :"      << narval::oculus::to_string(msg.fireMessage.head, prefix)
       << "\n- simple fire :" << narval::oculus::to_string(msg.fireMessage, prefix)
       << "\n- simple ping :" << narval::oculus::to_string(msg, prefix);
    return os;
}

