



#include <iostream> // bibliothèque d’entrées/sorties
// #include <fstream>
// #include <assert.h>
#include "../../../oculus_driver/include/oculus_driver/Oculus.h"


main(int argc, char *argv[])
{
    std::cout << "coucou" << std::endl;
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