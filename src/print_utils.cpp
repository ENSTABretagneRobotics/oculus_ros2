#include <narval_oculus/print_utils.h>

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

std::ostream& operator<<(std::ostream& os, const OculusMessageHeader& msg)
{
    const std::string prefix("\n  ");
    os << prefix << "oculusId    : " << msg.oculusId
       << prefix << "srcDeviceId : " << msg.srcDeviceId
       << prefix << "dstDeviceId : " << msg.dstDeviceId
       << prefix << "msgId       : " << msg.msgId
       << prefix << "msgVersion  : " << msg.msgVersion
       << prefix << "payloadSize : " << msg.payloadSize
       << prefix << "spare2      : " << msg.spare2 << "\n";
    return os;
}

std::ostream& operator<<(std::ostream& os, const OculusStatusMsg& msg)
{
    const std::string prefix("\n  ");
    
    os << "Oculus status message :"
       << "\nHeader:"
       << msg.hdr
       << "Status :"
       << prefix << "deviceId        : " << msg.deviceId
       << prefix << "status          : " << msg.status
       << prefix << "part number     : " << msg.partNumber
       << prefix << "ipAddr          : " <<  narval::oculus::ip_to_string(msg.ipAddr)
       << prefix << "ipMask          : " << narval::oculus::ip_to_string(msg.ipMask)
       << prefix << "connectedIpAddr : " << narval::oculus::ip_to_string(msg.connectedIpAddr)
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
    return os;
}
