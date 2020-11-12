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
        << prefix << "flags           : " << (int)msg.flags
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

