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

int main()
{
    boost::asio::io_service ioService;
    StatusListener listener(ioService);

    listener.add_callback(&print_callback);
    
    ioService.run(); // is blocking

    return 0;
}


