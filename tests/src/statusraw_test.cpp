#include <iostream>
#include <sstream>
using namespace std;

#include <narval_oculus/StatusListener.h>
using namespace narval::oculus;

int main()
{
    boost::asio::io_service ioService;
    StatusListener listener(ioService);
    
    ioService.run(); // is blocking

    return 0;
}


