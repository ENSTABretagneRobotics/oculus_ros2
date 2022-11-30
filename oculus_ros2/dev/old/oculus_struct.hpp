
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

typedef uint32_t U32;
typedef float_t F32;

struct HEADER
{
    U32 numberBytes; // total number of bytes in page
    U32 pageVersion;
    U32 configuration;
    U32 pingNumber;

};


HEADER get_header()
{
    std::ifstream sdf_file;
    HEADER header;
    sdf_file.open("../s5k20110414_082223.sdf");
    sdf_file.seekg(4 * 1); // pass the marker
    sdf_file.read((char *)&header, sizeof(header));
    sdf_file.close();
    return header;
}

