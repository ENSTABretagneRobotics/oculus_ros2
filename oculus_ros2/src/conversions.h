#ifndef _DEF_NARVAL_OCULUS_ROS_CONVERSIONS_H_
#define _DEF_NARVAL_OCULUS_ROS_CONVERSIONS_H_

#include <oculus_driver/Oculus.h>
#include "oculus_interfaces/msg/oculus_header.hpp"
#include "oculus_interfaces/msg/oculus_version_info.hpp"
#include "oculus_interfaces/msg/oculus_status.hpp"
#include "oculus_interfaces/msg/oculus_fire_config.hpp"
#include "oculus_interfaces/msg/oculus_ping.hpp"

namespace narval { namespace oculus {

void copy_to_ros(oculus_interfaces::msg::OculusHeader &msg, const OculusMessageHeader& header)
{
    msg.oculus_id     = header.oculusId;
    msg.src_device_id  = header.srcDeviceId;
    msg.dst_device_id  = header.dstDeviceId;
    msg.msg_id        = header.msgId;
    msg.msg_version   = header.msgVersion;
    msg.payload_size  = header.payloadSize;
    msg.spare2       = header.spare2;
}

void copy_to_ros(oculus_interfaces::msg::OculusVersionInfo &msg, const OculusVersionInfo& version)
{
    msg.firmware_version0 = version.firmwareVersion0;
    msg.firmware_date0    = version.firmwareDate0;
    msg.firmware_version1 = version.firmwareVersion1;
    msg.firmware_date1    = version.firmwareDate1;
    msg.firmware_version2 = version.firmwareVersion2;
    msg.firmware_date2    = version.firmwareDate2;
}

void copy_to_ros(oculus_interfaces::msg::OculusStatus &msg, const OculusStatusMsg& status)
{
    copy_to_ros(msg.hdr, status.hdr);

    msg.device_id        = status.deviceId;
    msg.device_type      = status.deviceType;
    msg.part_number      = status.partNumber;
    msg.status          = status.status;

    copy_to_ros(msg.versin_info,status.versinInfo);

    msg.ip_addr          = status.ipAddr;
    msg.ip_mask          = status.ipMask;
    msg.connected_ip_addr = status.connectedIpAddr;

    msg.mac_addr0        = status.macAddr0;
    msg.mac_addr1        = status.macAddr1;
    msg.mac_addr2        = status.macAddr2;
    msg.mac_addr3        = status.macAddr3;
    msg.mac_addr4        = status.macAddr4;
    msg.mac_addr5        = status.macAddr5;

    msg.temperature0    = status.temperature0;
    msg.temperature1    = status.temperature1;
    msg.temperature2    = status.temperature2;
    msg.temperature3    = status.temperature3;
    msg.temperature4    = status.temperature4;
    msg.temperature5    = status.temperature5;
    msg.temperature6    = status.temperature6;
    msg.temperature7    = status.temperature7;
    msg.pressure        = status.pressure;
}

void copy_to_ros(oculus_interfaces::msg::OculusFireConfig &msg, const OculusSimpleFireMessage& fireConfig)
{
    copy_to_ros(msg.head, fireConfig.head);

    msg.master_mode      = fireConfig.masterMode;
    msg.ping_rate        = fireConfig.pingRate;
    msg.network_speed    = fireConfig.networkSpeed;
    msg.gamma_correction = fireConfig.gammaCorrection;
    msg.flags           = fireConfig.flags;
    msg.range           = fireConfig.range;
    msg.gain_percent     = fireConfig.gainPercent;
    msg.speed_of_sound    = fireConfig.speedOfSound;
    msg.salinity        = fireConfig.salinity;
}

void copy_to_ros(oculus_interfaces::msg::OculusPing &msg, const OculusSimplePingResult& ping)
{
    copy_to_ros(msg.fire_message, ping.fireMessage);
    msg.ping_id            = ping.pingId;
    msg.status            = ping.status;
    msg.frequency         = ping.frequency;
    msg.temperature       = ping.temperature;
    msg.pressure          = ping.pressure;
    msg.speeed_of_sound_used = ping.speeedOfSoundUsed;
    msg.ping_start_time     = ping.pingStartTime;
    msg.data_size          = ping.dataSize;
    msg.range_resolution   = ping.rangeResolution;
    msg.n_ranges           = ping.nRanges;
    msg.n_beams            = ping.nBeams;
    msg.image_offset       = ping.imageOffset;
    msg.image_size         = ping.imageSize;
    msg.message_size       = ping.messageSize;
}

} //namespace oculus
} //namespace narval

#endif //_DEF_NARVAL_OCULUS_ROS_CONVERSIONS_H_
