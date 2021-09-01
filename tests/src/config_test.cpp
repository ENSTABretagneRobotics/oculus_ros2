#include <iostream>
#include <sstream>
#include <thread>
using namespace std;

#include <narval_oculus/AsyncService.h>
#include <narval_oculus/SonarDriver.h>
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
    //Sonar sonar;
    AsyncService ioService;
    SonarDriver sonar(ioService.io_service());
    
    sonar.add_ping_callback(&print_ping);
    sonar.add_dummy_callback(&print_dummy);

    ioService.start();

    //sonar.request_fire_config(default_fire_config());
    //sonar.request_fire_config(default_fire_config());

    getchar();

    ioService.stop();

    return 0;
}


