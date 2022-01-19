#include "PingRenderer.h"

namespace rtac { namespace display {

PingRenderer::PingRenderer(const GLContext::Ptr& context) :
    FanRenderer(context),
    currentMasterMode_(255)
{}

PingRenderer::Ptr PingRenderer::Create(const GLContext::Ptr& context)
{
    return Ptr(new PingRenderer(context));
}

void PingRenderer::load_geometry(const oculus_sonar::OculusPing& ping)
{
    auto bearingData = reinterpret_cast<const int16_t*>(ping.data.data() + sizeof(PingResult));

    Interpolator::Vector bearings(ping.nBeams);
    for(int i = 0; i < ping.nBeams; i++) {
        bearings(i) = 0.01f * M_PI * bearingData[i] / 180.0;
    }

    this->set_geometry({bearings(0), bearings(ping.nBeams-1)},
                       {0.0f, (float)ping.fireMessage.range});
    //this->set_bearings(ping.nBeams, bearings.data());
    this->set_bearings(ping.nBeams, bearings.data(), 1024);
}

void PingRenderer::load_gains(const oculus_sonar::OculusPing& ping)
{
    gains_.resize(ping.nRanges);
    if(!(ping.fireMessage.flags & 0x4)) {
        // gains not sent setting gains to 1
        for(auto& g : gains_) g = 1.0f;
        return;
    }

    unsigned int stride = 0;
    if(ping.fireMessage.flags & 0x2) {
        // 16bits data
        stride = 4 + 2*ping.nBeams;
    }
    else {
        // 8bits data
        stride = 4 + ping.nBeams;
    }

    // auto p = gains_.map();
    // const uint8_t* data = ping.data.data() + ping.imageOffset;
    // for(int i = 0; i < ping.nRanges; i++) {
    //     p[i] = 1.0f / sqrt((float)((const uint32_t*)data)[0]);
    //     data += stride;
    // }
    
    const uint8_t* data = ping.data.data() + ping.imageOffset;
    for(int i = 0; i < ping.nRanges; i++) {
        gains_[i] = 1.0f / sqrt((float)((const uint32_t*)data)[0]);
        data += stride;
    }
}

void PingRenderer::set_ping_data(const oculus_sonar::OculusPing& ping)
{
    this->load_geometry(ping);
    this->load_gains(ping);

    pingData_.resize(ping.nBeams*ping.nRanges);
    {
    auto pout = pingData_.map();
    if(ping.fireMessage.flags & 0x2) {
        // 16bits data
        auto pin = reinterpret_cast<const uint16_t*>(ping.data.data() + ping.imageOffset);
        unsigned int stride = ping.nBeams;
        if(ping.fireMessage.flags & 0x4) {
            pin    += 2;
            stride += 2;
        }
        for(int i = 0; i < ping.nRanges; i++) {
            for(int j = 0; j < ping.nBeams; j++) {
                pout[ping.nBeams*i + j] = gains_[i]*pin[j];
            }
            pin += stride;
        }
    }
    else {
        // 8bits data
        auto pin = ping.data.data() + ping.imageOffset;
        unsigned int stride = ping.nBeams;
        if(ping.fireMessage.flags & 0x4) {
            pin    += 4;
            stride += 4;
        }
        for(int i = 0; i < ping.nRanges; i++) {
            for(int j = 0; j < ping.nBeams; j++) {
                if(i < 10) {
                    pout[ping.nBeams*i + j] = 0.0f;
                    continue;
                }
                pout[ping.nBeams*i + j] = gains_[i]*pin[j];
            }
            pin += stride;
        }
    }
    }

    this->set_data({ping.nBeams, ping.nRanges}, pingData_, true);
}

}; //namespace display
}; //namespace rtac
