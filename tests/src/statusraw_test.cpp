#include <iostream>
#include <sstream>
using namespace std;

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
    boost::asio::io_service ioService;
    StatusListener listener(ioService);

    listener.add_callback(&print_callback);

    listener.add_callback(&callback1);

    CallbackTest test0;
    listener.add_callback(&CallbackTest::callback2, &test0);
    listener.add_callback(&CallbackTest::callback3, &test0, 14);

    // this fail at compile time
    //listener.add_callback(&CallbackTest::callback4, &test0, 14);
    
    ioService.run(); // is blocking

    return 0;
}


