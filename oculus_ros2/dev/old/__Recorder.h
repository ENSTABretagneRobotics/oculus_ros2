#ifndef _DEF_OCULUS_DRIVER_RECORDER_H_
#define _DEF_OCULUS_DRIVER_RECORDER_H_

#include <fstream>

#include <oculus_driver/SonarDriver.h>

namespace oculus {

namespace blueprint {

// // these are taken as-is from Blueprint SDK
// struct RmLogHeader
// {
// public:
//   unsigned       fileHeader;   // Fixed 4 byte header ident type
//   unsigned       sizeHeader;   // Size of this structure
//   char           source[16];   // 12 character max source identifier
//   unsigned short version;      // File version type
//   unsigned short encryption;   // Encryption style (0 = none)
//   qint64         key;          // Possibly saved encryption key (0 otherwise)
//   double         time;         // Time of file creation
// };
// 
// // ----------------------------------------------------------------------------
// // The post-ping fire message received back from the sonar ???
// struct RmLogItem
// {
// public:
//   unsigned       itemHeader;   // Fixed 4 byte header byte
//   unsigned       sizeHeader;   // Size of this structure
//   unsigned short type;         // Identifer for the contained data type
//   unsigned short version;      // Version for the data type
//   double         time;         // Time item creation
//   unsigned short compression;  // Compression type 0 = none, 1 = qCompress
//   unsigned       originalSize; // Size of the payload prior to any compression
//   unsigned       payloadSize;  // Size of the following payload
// };

// Replacing with fixed size types to avoid discrepancies between platforms.
// Unchecked alignment still an issue (probably)
struct LogHeader
{
    uint32_t fileHeader;   // Fixed 4 byte header ident type
    uint32_t sizeHeader;   // Size of this structure
    char     source[16];   // 12 character max source identifier
    uint16_t version;      // File version type
    uint16_t encryption;   // Encryption style (0 = none)
    int64_t  key;          // Possibly saved encryption key (0 otherwise)
    double   time;         // Time of file creation
};

struct LogItem
{
    uint32_t itemHeader;   // Fixed 4 byte header byte
    uint32_t sizeHeader;   // Size of this structure
    uint16_t type;         // Identifer for the contained data type
    uint16_t version;      // Version for the data type
    double   time;         // Time item creation
    uint16_t compression;  // Compression type 0 = none, 1 = qCompress
    uint32_t originalSize; // Size of the payload prior to any compression
    uint32_t payloadSize;  // Size of the following payload
};

enum RecordTypes
{
    rt_settings          = 1,    // RmSettingsLogger packet
    rt_serialPort        = 2,    // Raw serial string - version contains the port number
    rt_oculusSonar       = 10,   // Raw oculus sonar data
    rt_blueviewSonar     = 11,   // Blueview data log image (raw)
    rt_rawVideo          = 12,   // Raw video logg
    rt_h264Video         = 13,   // H264 compresses video log
    rt_apBattery         = 14,   // ApBattery structure
    rt_apMissionProgress = 15,   // ApMissionProgress structure
    rt_nortekDVL         = 16,   // The complete Nortek DVL structure
    rt_apNavData         = 17,   // ApNavData structures
    rt_apDvlData         = 18,   // ApDvlData structures
    rt_apAhrsData        = 19,   // ApAhrsData structure
    rt_apSonarHeader     = 20,   // ApSonarHeader followed by image
    rt_rawSonarImage     = 21,   // Raw sonar image
    rt_ahrsMtData2       = 22,   // XSens MtData2 message
    rt_apVehicleInfo     = 23,   // Artemis ApVehicleInfo structures
    rt_apMarker          = 24,   // ApMarker structure
    rt_apGeoImageHeader  = 25,   // ApGeoImageHeader
    rt_apGeoImageData    = 26,   // ApGeoImage data of image
    rt_sbgData           = 30,   // SBG compass data message
    rt_ocViewInfo        = 500,  // Oculus view information
    rt_oculusSonarStamp  = 1010
};

}

class Recorder
{
    public:

    static constexpr uint32_t    FileMagicNumber = 0x11223344;
    static constexpr uint32_t    ItemMagicNumber = 0xaabbccdd;
    static constexpr const char* SourceId        = "Oculus";

    // nanoseconds not necessary but ROS compatible
    struct TimeStamp {
        uint64_t seconds;
        uint64_t nanoseconds;

        TimeStamp& operator=(const SonarDriver::TimePoint& other) {
            return *this;
        }
        
        template <typename T>
        T to_seconds() const { return (T)seconds + 1.0e-9*nanoseconds; }
    };

    protected:

    std::string           filename_;
    mutable std::ofstream file_;

    public:

    Recorder();
    ~Recorder();

    void open(const std::string& filename, bool force = false);
    void close();
    bool is_open() const { return file_.is_open(); }

    std::size_t write(const OculusMessageHeader& header,
                      const std::vector<uint8_t>& data,
                      const SonarDriver::TimePoint& timestamp) const;
};

} // namespace oculus

#endif //_DEF_OCULUS_DRIVER_RECORDER_H_


