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

#include <narval_oculus/Oculus.h>
#include <narval_oculus/print_utils.h>
#include <narval_oculus/StatusListener.h>
using namespace narval::oculus;

void print_callback(const OculusStatusMsg& msg)
{
    cout << msg << endl;
}

void callback1(const OculusStatusMsg& msg)
{
    cout << "callback1" << endl;
}

struct CallbackTest
{
    void callback2(const OculusStatusMsg& msg)
    {
        cout << "callback2" << endl;
    }
    
    // name of function must be unique (otherwise fails at overload resolution)
    void callback3(int value, const OculusStatusMsg& msg)
    {
        cout << "callback3, value : " << value << endl;
    }
    
    // cannot bind this one. msg must be the last parameter
    void callback4(const OculusStatusMsg& msg, int value)
    {
        cout << "callback4, value : " << value << endl;
    }
};

int main()
{
    auto ioService = std::make_shared<StatusListener::IoService>();
    StatusListener listener(ioService);

    listener.add_callback(&print_callback);

    listener.add_callback(&callback1);

    CallbackTest test0;
    listener.add_callback(&CallbackTest::callback2, &test0);
    listener.add_callback(&CallbackTest::callback3, &test0, 14);

    // this fail at compile time
    //listener.add_callback(&CallbackTest::callback4, &test0, 14);
    
    ioService->run(); // is blocking

    return 0;
}


