#include <iostream>
#include <sstream>
#include <thread>
using namespace std;

#include <narval_oculus/Sonar.h>
using namespace narval::oculus;

void print_ping(const OculusSimplePingResult& pingMetadata,
                const std::vector<uint8_t>& pingData)
{
    cout << "=============== Got Ping :" << endl;
    //cout << pingMetadata << endl;
    cout << pingMetadata.fireMessage.gainPercent << endl;
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

    //sonar.request_fire_config(default_fire_config());
    //sonar.request_fire_config(default_fire_config());

    getchar();

    return 0;
}


