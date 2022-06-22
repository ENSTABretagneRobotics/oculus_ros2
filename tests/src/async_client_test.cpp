/******************************************************************************
 * narval_oculus driver library for Blueprint Subsea Oculus sonar.
 * Copyright (C) 2020 ENSTA-Bretagne
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *****************************************************************************/

#include <iostream>
#include <sstream>
using namespace std;

#include <narval_oculus/AsyncService.h>
#include <narval_oculus/SonarDriver.h>
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

void print_all(const OculusMessageHeader& header,
               const std::vector<uint8_t>& data)
{
    switch(header.msgId) {
        case messageSimplePingResult:
            std::cout << "Got messageSimplePingResult" << endl;
            break;
        case messageDummy:
            std::cout << "Got messageDummy" << endl;
            break;
        case messageSimpleFire:
            std::cout << "Got messageSimpleFire" << endl;
            break;
        case messagePingResult:
            std::cout << "Got messagePingResult" << endl;
            break;
        case messageUserConfig:
            std::cout << "Got messageUserConfig" << endl;
            break;
        default:
            break;
    }

}


int main()
{
    //Sonar sonar;
    AsyncService ioService;
    SonarDriver sonar(ioService.io_service());
    
    //sonar.add_ping_callback(&print_ping);
    //sonar.add_dummy_callback(&print_dummy);
    sonar.add_message_callback(&print_all);

    ioService.start();

    // sonar.on_next_ping([](const SonarDriver::PingResult& pingMetadata,
    //                       const std::vector<uint8_t>& data){
    //     std::cout << "Got awaited ping !" << std::endl;
    // });
    cout << "After awaited ping" << endl;
    
    // stopping sonar firing
    auto config = default_ping_config();
    config.pingRate = pingRateStandby;
    sonar.send_ping_config(config);
    sonar.on_next_dummy([](const OculusMessageHeader& header) {
        std::cout << "Got awaited dummy !" << std::endl;
    });
    std::cout << "After awaited dummy" << std::endl;
    
    //sonar.on_next_status([](const OculusStatusMsg& msg) {
    //    std::cout << "Got awaited status !" << std::endl;
    //});
    std::cout << "After awaited status" << std::endl;
    getchar();

    ioService.stop();

    return 0;
}


