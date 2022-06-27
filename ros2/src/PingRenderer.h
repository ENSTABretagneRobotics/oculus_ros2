#ifndef _DEF_OCULUS_SONAR_PING_RENDERER_H_
#define _DEF_OCULUS_SONAR_PING_RENDERER_H_

#include <rtac_base/types/Handle.h>
#include <rtac_base/interpolation.h>

#include <rtac_display/renderers/FanRenderer.h>

#include <narval_oculus/Oculus.h>
#include <oculus_sonar/OculusPing.h>

namespace rtac { namespace display {

template <>
struct GLFormat<uint16_t>
{
    using Scalar = uint16_t;

    static constexpr unsigned int Size  = 1;
    static constexpr GLenum PixelFormat = GL_RED;
    static constexpr GLenum Type        = GL_UNSIGNED_SHORT;
};

class PingRenderer : public FanRenderer
{
    public:

    using Ptr      = rtac::types::Handle<PingRenderer>;
    using ConstPtr = rtac::types::Handle<const PingRenderer>;

    using PingResult = OculusSimplePingResult;

    using Interpolator = rtac::algorithm::InterpolatorLinear<float>;
    //using Interpolator = rtac::algorithm::InterpolatorCubicSpline<float>;

    protected:
    
    uint8_t         currentMasterMode_;
    GLVector<float> pingData_;
    //GLVector<float> gains_;
    std::vector<float> gains_;
    GLVector<float> iBearings_;

    void load_geometry(const oculus_sonar::OculusPing& ping);
    void load_gains(const oculus_sonar::OculusPing& ping);

    PingRenderer(const GLContext::Ptr& context);

    public:

    static Ptr Create(const GLContext::Ptr& context);
    
    void set_ping_data(const oculus_sonar::OculusPing& ping);
};

}; //namespace display
}; //namespace rtac

#endif //_DEF_OCULUS_SONAR_PING_RENDERER_H_
