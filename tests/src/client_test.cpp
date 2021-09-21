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

#include <narval_oculus/SonarDriver.h>
using namespace narval::oculus;

void print_ping(const OculusSimplePingResult& pingMetadata,
                const std::vector<uint8_t>& pingData)
{
    static unsigned int count = 0;
    cout << "=============== Got Ping : " << count++ << endl;
    //cout << pingMetadata << endl;
}

void print_dummy(const OculusMessageHeader& msg)
{
    static unsigned int count = 0;
    cout << "=============== Got dummy : " << count++ << endl;
    //cout << msg << endl;
}

int main()
{
    auto ioService = std::make_shared<SonarDriver::IoService>();
    SonarDriver driver(ioService);
    
    driver.add_ping_callback(&print_ping);
    driver.add_dummy_callback(&print_dummy);

    ioService->run(); // is blocking

    return 0;
}


