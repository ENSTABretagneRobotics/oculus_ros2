#include <iostream>
using namespace std;

#include <oculus_driver/Recorder.h>
using namespace oculus;

int main(int argc, char** argv)
{
    if(argc < 2) {
        throw std::runtime_error("Must give a .oculus file as parameter");
    }
    cout << "Opening file : " << argv[1] << endl;

    FileReader file(argv[1]);
    blueprint::LogItem header;
    std::vector<uint8_t> data;
    while(file.read_next(header, data)) {
        cout << "Got item : " << header.type << endl;
        cout << "data: " << data[0] << endl;
        cout << "size data: " << std::size(data) << endl;
    }

    return 0;
}
