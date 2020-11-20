#include <iostream>
#include <sstream>
using namespace std;

#include <narval_oculus/Sonar.h>
using namespace narval::oculus;

void print_ping(const OculusSimplePingResult& pingMetadata,
                const std::vector<uint8_t>& pingData)
{
    cout << "=============== Got Ping :" << endl;
    //cout << pingMetadata << endl;
}

void print_dummy(const OculusMessageHeader& msg)
{
    cout << "=============== Got dummy :" << endl;
    //cout << msg << endl;
}


int main()
{
    Sonar sonar;
    
    sonar.add_ping_callback(&print_ping);
    sonar.add_dummy_callback(&print_dummy);

    sonar.start();

    sonar.on_next_ping([](const SonarClient::PingResult& pingMetadata,
                          const std::vector<uint8_t>& data){
        std::cout << "Got awaited ping !" << std::endl;
    });
    cout << "Got awaited ping" << endl;
    sonar.on_next_dummy([](const OculusMessageHeader& header) {
        std::cout << "Got awaited dummy !" << std::endl;
    });

    getchar();

    return 0;
}


