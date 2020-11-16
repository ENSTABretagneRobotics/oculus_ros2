#include <iostream>
#include <sstream>
using namespace std;

#include <narval_oculus/Sonar.h>
using namespace narval::oculus;

void print_ping(const OculusSimplePingResult& pingMetadata,
                const std::vector<uint8_t>& pingData)
{
    cout << pingMetadata << endl;
}

int main()
{
    Sonar sonar;
    
    sonar.add_ping_callback(&print_ping);

    sonar.start(); // is blocking

    getchar();

    return 0;
}


