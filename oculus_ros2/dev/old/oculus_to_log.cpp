#include <iostream>
#include <oculus_driver/Recorder.h>
#include <oculus_driver/print_utils.h>

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

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        throw std::runtime_error("Must give a .oculus file as parameter");
    }
    std::cout << "Opening file : " << argv[1] << std::endl;

    // oculus::FileReader file(argv[1]);
    // oculus::blueprint::LogItem header;
    // std::vector<uint8_t> data;
    // while (file.read_next_item(header, data))
    // {
    //     std::cout << "Got item : " << header.type << std::endl;
    //     std::cout << "data: " << data[0] << std::endl;
    //     std::cout << "size data: " << std::size(data) << std::endl;
    //     std::cout << "display_LogHeader(header) : \n"
    //               << display_LogItem(header) << std::endl;
    // }

    oculus::FileReader file(argv[1]);
    auto msg = file.read_next_ping();
    std::cout << "Got ping    : " << msg->header() << std::endl;
    std::cout << "size        : " << msg->bearing_count() << 'x' << msg->range_count() << std::endl;
    std::cout << "master mode : " << (unsigned int)msg->master_mode() << std::endl;
    std::cout << "bearings :\n";
    auto bearingData = msg->bearing_data();
    for (int i = 0; i < msg->bearing_count(); i++)
    {
        std::cout << ' ' << 0.01 * bearingData[i];
    }
    std::cout << std::endl;

    return 0;
}
