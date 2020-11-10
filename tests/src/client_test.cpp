#include <iostream>
#include <sstream>
using namespace std;

#include <narval_oculus/Client.h>
using namespace narval::oculus;

int main()
{
    boost::asio::io_service ioService;
    Client client(ioService);
    
    ioService.run(); // is blocking

    return 0;
}


