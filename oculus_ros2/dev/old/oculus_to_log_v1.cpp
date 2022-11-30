

#include <iostream> // bibliothèque d’entrées/sorties
#include <fstream>
// #include <assert.h>
#include <oculus_driver/Oculus.h>
#include <oculus_driver/Recorder.h>

// #include <unistd.h>
// #include <stdio.h>

/*
Voir Recorder.h dans le repo oculus_driver, les messages d'intérêt sont
"OculusSimplePingResult"
 */


std::string display_LogHeader(oculus::blueprint::LogHeader header)
{
    // uint32_t fileHeader;   // Fixed 4 byte header ident type
    // uint32_t sizeHeader;   // Size of this structure
    // char     source[16];   // 12 character max source identifier
    // uint16_t version;      // File version type
    // uint16_t encryption;   // Encryption style (0 = none)
    // int64_t  key;          // Possibly saved encryption key (0 otherwise)
    // double   time;         // Time of file creation

    std::ostringstream str_header;
    str_header << "header.fileHeader: " << header.fileHeader << "    // Fixed 4 byte header ident type" << std::endl;
    str_header << "header.sizeHeader: " << header.sizeHeader << "    // Size of this structure" << std::endl;
    str_header << "source[16]: " << header.source[16] << "    // 12 character max source identifier" << std::endl;
    str_header << "header.version: " << header.version << "    // File version type" << std::endl;
    str_header << "header.encryption: " << header.encryption << "    // Encryption style (0 = none)" << std::endl;
    str_header << "header.key: " << header.key << "    // Possibly saved encryption key (0 otherwise)" << std::endl;
    str_header << "header.time: " << header.time << "    // Time of file creation" << std::endl;

    return str_header.str();
};


std::string display_LogItem(oculus::blueprint::LogItem item)
{
    /*
        uint32_t itemHeader;   // Fixed 4 byte header byte
        uint32_t sizeHeader;   // Size of this structure
        uint16_t type;         // Identifer for the contained data type
        uint16_t version;      // Version for the data type
        double   time;         // Time item creation
        uint16_t compression;  // Compression type 0 = none, 1 = qCompress
        uint32_t originalSize; // Size of the payload prior to any compression
        uint32_t payloadSize;  // Size of the following payload
     */

    std::ostringstream str_item;
    str_item << "itemHeader: " << item.itemHeader << "   // Fixed 4 byte header byte" << std::endl;
    str_item << "sizeHeader: " << item.sizeHeader << "   // Size of this structure" << std::endl;
    str_item << "type: " << item.type << "         // Identifer for the contained data type" << std::endl;
    str_item << "version: " << item.version << "      // Version for the data type" << std::endl;
    str_item << "  time: " << item.time << "         // Time item creation" << std::endl;
    str_item << "compression: " << item.compression << "  // Compression type 0 = none, 1 = qCompress" << std::endl;
    str_item << "originalSize: " << item.originalSize << " // Size of the payload prior to any compression" << std::endl;
    str_item << "payloadSize: " << item.payloadSize << "  // Size of the following payload" << std::endl;
    return str_item.str();
};

std::string display_OculusSimplePingResult(OculusSimplePingResult ping)
{
    /*
        OculusSimpleFireMessage fireMessage;
        uint32_t pingId;            // An incrementing number
        uint32_t status;
        double   frequency;         // The acoustic frequency (Hz)
        double   temperature;       // The external temperature (deg C)
        double   pressure;          // The external pressure (bar)
        double   speeedOfSoundUsed; // The actual used speed of sound (m/s). May be different to the speed of sound set in the fire message
        uint32_t pingStartTime;
        uint8_t  dataSize;          // The size of the individual data entries   // was DataSizeType
        double   rangeResolution;   // The range in metres corresponding to a single range line
        uint16_t nRanges;           // The number of range lines in the image
        uint16_t nBeams;            // The number of bearings in the image
        uint32_t imageOffset;       // The offset in bytes of the image data from the start of the network message
        uint32_t imageSize;         // The size in bytes of the image data
        uint32_t messageSize;       // The total size in bytes of the network message
        // *** NOT ADDITIONAL VARIABLES BEYOND THIS POINT ***
        // There will be an array of bearings (shorts) found at the end of the message structure
        // Allocated at run time
        // short bearings[];
        // The bearings to each of the beams in 0.01 degree resolution
     */

    std::ostringstream str_ping;
    str_ping << "ping.pingId : " << ping.pingId << "            // An incrementing number" << std::endl;
    str_ping << "ping.status : " << ping.status << "" << std::endl;
    str_ping << "ping.frequency : " << ping.frequency << "         // The acoustic frequency (Hz)" << std::endl;
    str_ping << "ping.temperature : " << ping.temperature << "       // The external temperature (deg C)" << std::endl;
    str_ping << "ping.pressure : " << ping.pressure << "          // The external pressure (bar)" << std::endl;
    str_ping << "ping.speeedOfSoundUsed : " << ping.speeedOfSoundUsed << " // The actual used speed of sound (m/s). May be different to the speed of sound set in the fire message" << std::endl;
    str_ping << "ping.pingStartTime : " << ping.pingStartTime << "" << std::endl;
    str_ping << "ping.dataSize : " << ping.dataSize << "          // The size of the individual data entries   // was DataSizeType" << std::endl;
    str_ping << "ping.rangeResolution : " << ping.rangeResolution << "   // The range in metres corresponding to a single range line" << std::endl;
    str_ping << "ping.nRanges : " << ping.nRanges << "           // The number of range lines in the image" << std::endl;
    str_ping << "ping.nBeams : " << ping.nBeams << "            // The number of bearings in the image" << std::endl;
    str_ping << "ping.imageOffset : " << ping.imageOffset << "       // The offset in bytes of the image data from the start of the network message" << std::endl;
    str_ping << "ping.imageSize : " << ping.imageSize << "         // The size in bytes of the image data" << std::endl;
    str_ping << "ping.messageSize : " << ping.messageSize << "       // The total size in bytes of the network message" << std::endl;

    return str_ping.str();
};

main(int argc, char *argv[]) // CLIUtils  ou ros ? pour chemain d'accès
{
    // Welcom
    std::cout << "coucou" << std::endl;

    /* where the code is executed
    // #include <unistd.h>
    // #include <stdio.h>
    char cwd[1024];
    getcwd(cwd, sizeof(cwd));
    printf("Current working dir: %s\n", cwd);
    */

    // Opening the .oculus file
    std::ifstream oculus_file;
    // Access only for developpement
    // std::string oculus_file_directory = "/home/hugo/Documents/stage_pfe/guerledan/ros2_ws/src/slamacc/oculus_ros2/oculus_ros2/dev";
    // std::string oculus_file_name = "Oculus_20221011_143224.oculus";
    std::string oculus_file_directory = "/home/hugo/Documents/stage_pfe/guerledan/ros2_ws/src/slamacc/oculus_ros2/oculus_ros2/dev";
    std::string oculus_file_name = "Exampl Logfiles/Oculus_M750d_1200kHz_River_Fish.oculus";
    oculus_file.open(oculus_file_directory + "/" + oculus_file_name, std::ios::in | std::ios::binary);
    if (!oculus_file.is_open())
    {
        std::cout << " ====== Issue with file opening ====== " << std::endl;
        std::cout << "Check the path of the .oculus file." << std::endl;
    }
    assert(oculus_file.is_open());

    // Get the header of the file
    oculus::blueprint::LogHeader file_header;
    // oculus_file.seekg(0, std::ios::beg);
    oculus_file.read((char *)&file_header, sizeof(file_header));
    std::cout << "display_LogHeader(file_header) : \n"
              << display_LogHeader(file_header) << std::endl;

    // On going
    oculus::blueprint::LogItem item_header;
    // oculus_file.seekg(0, std::ios::beg);
    oculus_file.read((char *)&item_header, sizeof(item_header));
    std::cout << "display_LogHeader(item_header) : \n"
              << display_LogItem(item_header) << std::endl;


    std::cout << std::endl << "oculus::blueprint::RecordTypes::rt_oculusSonar = " <<  oculus::blueprint::rt_oculusSonar << std::endl;
    std::cout << "item_header.type = " << item_header.type << std::endl << std::endl;



    OculusSimplePingResult ping;
    oculus_file.read((char *)&ping, sizeof(ping));
    std::cout << "display_LogHeader(ping) : \n"
              << display_OculusSimplePingResult(ping) << std::endl;

    oculus_file.close();
    return EXIT_SUCCESS;
}

/*

#include <iostream> // bibliothèque d’entrées/sorties
#include <fstream>
#include <assert.h>
#include <cmath>
#include <stdio.h>
#include <opencv2/opencv.hpp>

#include "header.hpp"

typedef uint32_t U32;
typedef float_t F32;





std::ifstream sdf_file;

struct PING
{
    SYS5000HEADER header;
    uint16_t chanels_size[10];
    uint16_t *data[10];
};



void load_ping(PING &ping)
{
    U32 mkr;
    // sdf_file.seekg(4 * 1); // pass the marker
    sdf_file.read((char *)&mkr, sizeof(mkr));
    assert(mkr == 0xFFFFFFFF);
    int marker_cursor = sdf_file.tellg();
    sdf_file.read((char *)&ping.header, sizeof(ping.header));
    for (int k = 0; k < 10; k++)
    {
        ping.chanels_size[k] = 2276;
        ping.data[k] = new uint16_t[ping.chanels_size[k]];
        sdf_file.seekg(sizeof(uint16_t), sdf_file.cur);
        sdf_file.read((char *)ping.data[k], ping.chanels_size[k] * sizeof(uint16_t));
    }
    // int cursor = sdf_file.tellg();
    sdf_file.seekg(ping.header.numberBytes - sdf_file.tellg() + marker_cursor, sdf_file.cur);
}

void load_pings(std::vector<PING> &pings)
{
    sdf_file.open("../s5k20110414_082223.sdf");
    sdf_file.seekg(0, std::ios::end);
    long last_byte = sdf_file.tellg();
    std::cout << "last_byte = " << last_byte << std::endl;
    sdf_file.seekg(0, std::ios::beg);
    std::cout << "sdf_file.tellg() = " << sdf_file.tellg() << std::endl;
    // last_byte = 2*45800;

    while (sdf_file.tellg() < last_byte) //(sdf_file.tellg()!=-1) //
    {
        PING ping;
        // std::cout << "sdf_file.tellg() = " << sdf_file.tellg() << std::endl;
        load_ping(ping);
        // std::cout << "sdf_file.tellg()2 = " << sdf_file.tellg() << std::endl;
        pings.push_back(ping);
        // std::cout << "coucou " << std::endl;
    }

    sdf_file.close();
}

main(int argc, char *argv[])
{

    sdf_file.open("../s5k20110414_082223.sdf");
    load_ping(ping);
    std::cout << "sizeof(ping) = " << sizeof(ping) << std::endl;
    std::cout << "ping.header.pageVersion = " << ping.header.pageVersion << std::endl;
    std::cout << "ping.chanels_size[9] = " << ping.chanels_size[9] << std::endl;
    sdf_file.close();

    std::vector<PING> pings;
    load_pings(pings);
    return EXIT_SUCCESS;
}
 */