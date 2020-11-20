#include <iostream>
#include <sstream>
#include <thread>
using namespace std;

#include <narval_oculus/SonarClient.h>
using namespace narval::oculus;

#include <narval_display/Display.h>
#include <narval_display/GLVector.h>
#include <narval_display/renderers/ImageRenderer.h>
using namespace narval::display;

Display* display;
ImageRenderer::Ptr renderer;
boost::asio::io_service* service;

void display_ping(const OculusSimplePingResult& pingMetadata,
                  const std::vector<uint8_t>& pingData)
{
    cout << "\rGot ping " << pingMetadata.pingId << flush;
    cout << endl << "Image offset : " << pingMetadata.imageOffset << endl;
    renderer->set_image({pingMetadata.nBeams, pingMetadata.nRanges},
                         pingData.data() + pingMetadata.imageOffset);
    display->draw();

    if(display->should_close())
        service->stop();
}

int main()
{
    Display disp;
    display = &disp;
    renderer = ImageRenderer::New();
    display->add_renderer(renderer);

    boost::asio::io_service ioService;
    service = &ioService;
    SonarClient client(ioService);
    
    client.add_ping_callback(&display_ping);

    ioService.run(); // is blocking

    return 0;
}


