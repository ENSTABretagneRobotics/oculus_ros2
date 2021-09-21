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


