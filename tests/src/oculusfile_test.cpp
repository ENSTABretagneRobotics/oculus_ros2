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
#include <cstdlib>
#include <fstream>
using namespace std;

#include <narval_base/files.h>
using namespace narval::files;

#include <narval_oculus/Oculus.h>
#include <narval_oculus/print_utils.h>
using namespace narval::oculus;

const uint16_t oculusId = 0x4f53;
const uint16_t deviceId = 7915;

bool is_valid(const OculusMessageHeader& header)
{
    return header.oculusId == oculusId && header.srcDeviceId == deviceId;
}

bool find_header(std::ifstream& f, OculusMessageHeader& header)
{
    int counter = 0;
    while(f.read(reinterpret_cast<char*>(&header), 4) && !is_valid(header))
        counter += 4;
    if(!f)
        return false;

    f.read(reinterpret_cast<char*>(&header) + 4, sizeof(header) - 4);

    cout << "Found header at byte : " << counter << endl;
    //cout << header << endl;

    return true;
}

void process(const OculusSimplePingResult* ping)
{
    //cout << *ping << endl;
}

void parse(const std::vector<char>& buffer)
{
    switch((reinterpret_cast<const OculusMessageHeader*>(buffer.data()))->msgId)
    {
        case messageSimplePingResult:
            process(reinterpret_cast<const OculusSimplePingResult*>(buffer.data()));
            break;
        case messageSimpleFire: cout << "Got simpleFire"   << endl; break;
        case messagePingResult: cout << "Got pingResult"   << endl; break;
        case messageUserConfig: cout << "Got userConfig"   << endl; break;
        case messageDummy:      cout << "Got messageDummy" << endl; break;
        default:
            cout << "unknown message type" << endl;
            break;
    }
}

void read_messages(std::ifstream& f)
{
    OculusMessageHeader header;
    std::vector<char> buffer;
    
    while(find_header(f, header)) {
        buffer.resize(header.payloadSize + sizeof(header));
        *(reinterpret_cast<OculusMessageHeader*>(buffer.data())) = header;
        f.read(buffer.data() + sizeof(header), header.payloadSize);
        parse(buffer);
    }
}

int main()
{
    auto path = find_one(".*.oculus", std::string(std::getenv("HOME")) + "/Documents");
    cout << "path : " << path << endl;

    std::ifstream f(path, std::ios::binary);
    if(!f.is_open())
        throw std::runtime_error("Could not open file : " + path);
    
    read_messages(f);

    return 0;
}
