#ifndef _DEF_NARVAL_OCULUS_ROS_CONVERSIONS_H_
#define _DEF_NARVAL_OCULUS_ROS_CONVERSIONS_H_

#include <narval_oculus/Oculus.h>
#include <oculus_sonar/OculusHeader.h>
#include <oculus_sonar/OculusVersionInfo.h>
#include <oculus_sonar/OculusStatus.h>
#include <oculus_sonar/OculusFireConfig.h>
#include <oculus_sonar/OculusPing.h>

namespace narval { namespace oculus {

void copy_to_ros(oculus_sonar::OculusHeader& msg, const OculusMessageHeader& header)
{
    msg.oculusId     = header.oculusId;
    msg.srcDeviceId  = header.srcDeviceId;
    msg.dstDeviceId  = header.dstDeviceId;
    msg.msgId        = header.msgId;
    msg.msgVersion   = header.msgVersion;
    msg.payloadSize  = header.payloadSize;
    msg.spare2       = header.spare2;
}

void copy_to_ros(oculus_sonar::OculusVersionInfo& msg, const OculusVersionInfo& version)
{
    msg.firmwareVersion0 = version.firmwareVersion0;
    msg.firmwareDate0    = version.firmwareDate0;
    msg.firmwareVersion1 = version.firmwareVersion1;
    msg.firmwareDate1    = version.firmwareDate1;
    msg.firmwareVersion2 = version.firmwareVersion2;
    msg.firmwareDate2    = version.firmwareDate2;
}

void copy_to_ros(oculus_sonar::OculusStatus& msg, const OculusStatusMsg& status)
{
    copy_to_ros(msg.hdr, status.hdr);

    msg.deviceId        = status.deviceId;
    msg.deviceType      = status.deviceType;
    msg.partNumber      = status.partNumber;
    msg.status          = status.status;

    copy_to_ros(msg.versinInfo,status.versinInfo);

    msg.ipAddr          = status.ipAddr;
    msg.ipMask          = status.ipMask;
    msg.connectedIpAddr = status.connectedIpAddr;

    msg.macAddr0        = status.macAddr0;
    msg.macAddr1        = status.macAddr1;
    msg.macAddr2        = status.macAddr2;
    msg.macAddr3        = status.macAddr3;
    msg.macAddr4        = status.macAddr4;
    msg.macAddr5        = status.macAddr5;

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

void copy_to_ros(oculus_sonar::OculusFireConfig& msg, const OculusSimpleFireMessage& fireConfig)
{
    copy_to_ros(msg.head, fireConfig.head);

    msg.masterMode      = fireConfig.masterMode;
    msg.pingRate        = fireConfig.pingRate;
    msg.networkSpeed    = fireConfig.networkSpeed;
    msg.gammaCorrection = fireConfig.gammaCorrection;
    msg.flags           = fireConfig.flags;
    msg.range           = fireConfig.range;
    msg.gainPercent     = fireConfig.gainPercent;
    msg.speedOfSound    = fireConfig.speedOfSound;
    msg.salinity        = fireConfig.salinity;
}

void copy_to_ros(oculus_sonar::OculusPing& msg, const OculusSimplePingResult& ping)
{
    copy_to_ros(msg.fireMessage, ping.fireMessage);
    msg.pingId            = ping.pingId;
    msg.status            = ping.status;
    msg.frequency         = ping.frequency;
    msg.temperature       = ping.temperature;
    msg.pressure          = ping.pressure;
    msg.speeedOfSoundUsed = ping.speeedOfSoundUsed;
    msg.pingStartTime     = ping.pingStartTime;
    msg.dataSize          = ping.dataSize;
    msg.rangeResolution   = ping.rangeResolution;
    msg.nRanges           = ping.nRanges;
    msg.nBeams            = ping.nBeams;
    msg.imageOffset       = ping.imageOffset;
    msg.imageSize         = ping.imageSize;
    msg.messageSize       = ping.messageSize;
}

}; //namespace oculus
}; //namespace narval

#endif //_DEF_NARVAL_OCULUS_ROS_CONVERSIONS_H_
