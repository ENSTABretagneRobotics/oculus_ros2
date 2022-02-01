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

#ifndef _DEF_NARVAL_OCULUS_SONAR_DRIVER_H_
#define _DEF_NARVAL_OCULUS_SONAR_DRIVER_H_

#include <narval_oculus/Oculus.h>
#include <narval_oculus/utils.h>
#include <narval_oculus/print_utils.h>
#include <narval_oculus/CallbackQueue.h>
#include <narval_oculus/SonarClient.h>

namespace narval { namespace oculus {

template <typename TimeSourceT = std::chrono::system_clock>
class SonarDriver : public SonarClient<TimeSourceT>
{
    public:

    using Duration     = typename SonarClient<TimeSourceT>::Duration;

    using IoService    = boost::asio::io_service;
    using IoServicePtr = std::shared_ptr<IoService>;

    using PingConfig    = OculusSimpleFireMessage;
    using PingResult    = OculusSimplePingResult;
    using PingCallbacks = CallbackQueue<const PingResult&,
                                        const std::vector<uint8_t>&>; 
    using DummyCallbacks   = CallbackQueue<const OculusMessageHeader&>;
    using MessageCallbacks = CallbackQueue<const OculusMessageHeader&,
                                           const std::vector<uint8_t>&>;

    using TimeSource = TimeSourceT;
    using TimePoint  = typename std::invoke_result<decltype(&TimeSource::now)>::type;

    protected:

    PingConfig   lastConfig_;
    PingRateType lastPingRate_;

    PingCallbacks    pingCallbacks_;
    DummyCallbacks   dummyCallbacks_;
    MessageCallbacks messageCallbacks_; // will be called on every received message.

    public:

    SonarDriver(const IoServicePtr& service,
                const Duration& checkerPeriod = boost::posix_time::seconds(1));

    bool send_ping_config(PingConfig config);
    PingConfig current_ping_config();
    PingConfig request_ping_config(const PingConfig& request);
    PingConfig last_ping_config() const;

    // Stanby mode (saves current ping rate and set it to 0 on the sonar
    void standby();
    void resume();
    
    virtual void on_connect();
    virtual void handle_message(const OculusMessageHeader& header,
                                const std::vector<uint8_t>& data);

    /////////////////////////////////////////////
    // All remaining member function are related to callbacks and are merely
    // helpers to add callbacks.

    // status callbacks managing functions
    template <typename F, class... Args>
    unsigned int add_status_callback(F&& func, Args&&... args);
    unsigned int add_status_callback(const StatusListener::CallbackT& callback);
    bool remove_status_callback(unsigned int callbackId);
    template <typename F, class... Args>
    bool on_next_status(F&& func, Args&&... args);
    bool on_next_status(const StatusListener::CallbackT& callback);

    // ping callbacks managing functions
    template <typename F, class... Args>
    unsigned int add_ping_callback(F&& func, Args&&... args);
    unsigned int add_ping_callback(const PingCallbacks::CallbackT& callback);
    bool remove_ping_callback(unsigned int callbackId);
    // these are synchronous function which will wait for the next callback call.
    template <typename F, class... Args>
    bool on_next_ping(F&& func, Args&&... args);
    bool on_next_ping(const PingCallbacks::CallbackT& callback);
    
    // dummy message callbacks managing functions
    template <typename F, class... Args>
    unsigned int add_dummy_callback(F&& func, Args&&... args);
    unsigned int add_dummy_callback(const DummyCallbacks::CallbackT& callback);
    bool remove_dummy_callback(unsigned int callbackId);
    // these are synchronous function which will wait for the next callback call.
    template <typename F, class... Args>
    bool on_next_dummy(F&& func, Args&&... args);
    bool on_next_dummy(const DummyCallbacks::CallbackT& callback);
    
    // generic message callbacks managing functions (will be called on any
    // message received).
    template <typename F, class... Args>
    unsigned int add_message_callback(F&& func, Args&&... args);
    unsigned int add_message_callback(const MessageCallbacks::CallbackT& callback);
    bool remove_message_callback(unsigned int callbackId);
    // these are synchronous function which will wait for the next callback call.
    template <typename F, class... Args>
    bool on_next_message(F&& func, Args&&... args);
    bool on_next_message(const MessageCallbacks::CallbackT& callback);
    bool wait_next_message();
};

template <typename T>
SonarDriver<T>::SonarDriver(const IoServicePtr& service,
                            const Duration& checkerPeriod) :
    SonarClient<T>(service, checkerPeriod),
    lastConfig_(default_ping_config()),
    lastPingRate_(pingRateNormal)
{}

template <typename T>
bool SonarDriver<T>::send_ping_config(PingConfig config)
{
    config.head.oculusId    = OCULUS_CHECK_ID;
    config.head.msgId       = messageSimpleFire;
    config.head.srcDeviceId = 0;
    config.head.dstDeviceId = this->sonarId_;
    config.head.payloadSize = sizeof(PingConfig) - sizeof(OculusMessageHeader);

    // Other non runtime-configurable parameters (TODO : make then launch parameters)
    config.networkSpeed = 0xff;

    boost::asio::streambuf buf;
    buf.sputn(reinterpret_cast<const char*>(&config), sizeof(config));
    
    auto bytesSent = this->send(buf);
    if(bytesSent != sizeof(config)) {
        std::cerr << "Could not send whole fire message(" << bytesSent
                  << "/" << sizeof(config) << ")" << std::endl;
        return false;
    }

    // BUG IN THE SONAR FIRMWARE : the sonar never sets the
    // config.pingRate field in the SimplePing message -> there is no
    // feedback saying if this parameter is effectively set by the sonar. The
    // line below allows to keep a trace of the requested ping rate but there
    // is no clean way to check.
    lastConfig_.pingRate = config.pingRate;
    
    // Also saving the last pingRate which is not standby to be able to resume
    // the sonar to the last ping rate in the resume() method.
    if(lastConfig_.pingRate != pingRateStandby) {
        lastPingRate_ = lastConfig_.pingRate;
    }
    return true;
}

template <typename T>
typename SonarDriver<T>::PingConfig SonarDriver<T>::last_ping_config() const
{
    return lastConfig_;
}

template <typename T>
typename SonarDriver<T>::PingConfig SonarDriver<T>::current_ping_config()
{
    PingConfig config;
    if(!this->on_next_message([&](const OculusMessageHeader& header,
                                  const std::vector<uint8_t>& data) {
        // lastConfig_ is ALWAYS updated before the callbacks are called.
        // We only need to wait for the next message to get the current ping
        // configuration.
        config = lastConfig_;
        config.head = header;
    }))
    {
        throw MessageCallbacks::TimeoutReached();
    }
    return config;
}

template <typename T>
typename SonarDriver<T>::PingConfig SonarDriver<T>::request_ping_config(const PingConfig& request)
{
    // Waiting for a ping or a dummy message to have a feedback on the config changes.
    PingConfig feedback;
    int count = 0;
    const int maxCount = 100; // TODO make a parameter out of this
    do {
        if(this->send_ping_config(request)) {
            try {
                feedback = this->current_ping_config();
                if(check_config_feedback(request, feedback))
                    break;
            }
            catch(const MessageCallbacks::TimeoutReached& e) {
                std::cerr << "Timeout reached while requesting config" << std::endl;
                continue;
            }
        }
        count++;
    } while(count < maxCount);
    //std::cout << "Count is : " << count << std::endl << std::flush;

    if(count >= maxCount) {
        std::cerr << "Could not get a proper feedback from the sonar."
                  << "Assuming the configuration is ok (fix this)" << std::endl;
        feedback = request;
        feedback.head.msgId = 0; // invalid, will be checkable.
    }
    
    return feedback;
}

template <typename T>
void SonarDriver<T>::standby()
{
    auto request = lastConfig_;

    request.pingRate = pingRateStandby;
    
    this->send_ping_config(request);
}

template <typename T>
void SonarDriver<T>::resume()
{
    auto request = lastConfig_;

    request.pingRate = lastPingRate_;
    
    this->send_ping_config(request);
}

/**
 * Called when the driver first connection with the sonar.
 *
 * Will be called again if a new connection append.
 */
template <typename T>
void SonarDriver<T>::on_connect()
{
    // This makes the oculus fire right away.
    // On first connection lastConfig_ is equal to default_ping_config().
    this->send_ping_config(lastConfig_);
}

/**
 * Called when a new complete message is received (any type).
 */
template <typename T>
void SonarDriver<T>::handle_message(const OculusMessageHeader& header,
                                 const std::vector<uint8_t>& data)
{
    // Setting lastConfig_ BEFORE calling any callback.
    switch(header.msgId) {
        case messageSimplePingResult:
            {
                // Overwritting pingRate. The feedback of the sonar on this
                // parameter is broken.  lastConfig_.pingRate have been set in
                // send_ping_config().  (Bracket block is to keep the temporary
                // lastPingRate variable local to this block.)
                auto lastPingRate = lastConfig_.pingRate;
                lastConfig_ = reinterpret_cast<const PingResult*>(data.data())->fireMessage;
                lastConfig_.pingRate = lastPingRate;
            }
            // When masterMode = 2, the sonar force gainPercent between 40& and
            // 100%, BUT still needs resquested gainPercent to be between 0%
            // and 100%. (If you request a gainPercent=0 in masterMode=2, the
            // fireMessage in the ping results will be 40%). The gainPercent is
            // rescaled here to ensure consistent parameter handling on client
            // side).
            if(lastConfig_.masterMode == 2) {
                lastConfig_.gainPercent = (lastConfig_.gainPercent - 40.0) * 100.0 / 60.0;
            }
            break;
        default:
            lastConfig_.pingRate = pingRateStandby;
            break;
    };

    // Calling generic message callbacks first (in case we want to do something
    // before calling the specialized callbacks).
    messageCallbacks_.call(header, data);
    switch(header.msgId) {
        case messageSimplePingResult:
            pingCallbacks_.call(*(reinterpret_cast<const PingResult*>(data.data())), data);
            break;
        case messageDummy:
            dummyCallbacks_.call(header);
            break;
        case messageSimpleFire:
            std::cerr << "messageSimpleFire parsing not implemented." << std::endl;
            break;
        case messagePingResult:
            std::cerr << "messagePingResult parsing not implemented." << std::endl;
            break;
        case messageUserConfig:
            std::cerr << "messageUserConfig parsing not implemented." << std::endl;
            break;
        default:
            break;
    }
}

// status callbacks
template <typename T>
unsigned int SonarDriver<T>::add_status_callback(const StatusListener::CallbackT& callback)
{
    return this->statusListener_.add_callback(callback);
}

template <typename T>
bool SonarDriver<T>::remove_status_callback(unsigned int callbackId)
{
    return this->statusListener_.remove_callback(callbackId);
}

template <typename T>
bool SonarDriver<T>::on_next_status(const StatusListener::CallbackT& callback)
{
    return this->statusListener_.on_next_status(callback);
}

// ping callbacks
template <typename T>
unsigned int SonarDriver<T>::add_ping_callback(const PingCallbacks::CallbackT& callback)
{
    return pingCallbacks_.add_callback(callback);
}

template <typename T>
bool SonarDriver<T>::remove_ping_callback(unsigned int callbackId)
{
    return pingCallbacks_.remove_callback(callbackId);
}

template <typename T>
bool SonarDriver<T>::on_next_ping(const PingCallbacks::CallbackT& callback)
{
    return pingCallbacks_.add_single_shot(callback);
}

// dummy callbacks
template <typename T>
unsigned int SonarDriver<T>::add_dummy_callback(const DummyCallbacks::CallbackT& callback)
{
    return dummyCallbacks_.add_callback(callback);
}

template <typename T>
bool SonarDriver<T>::remove_dummy_callback(unsigned int callbackId)
{
    return dummyCallbacks_.remove_callback(callbackId);
}

template <typename T>
bool SonarDriver<T>::on_next_dummy(const DummyCallbacks::CallbackT& callback)
{
    return dummyCallbacks_.add_single_shot(callback);
}

// message callbacks
template <typename T>
unsigned int SonarDriver<T>::add_message_callback(const MessageCallbacks::CallbackT& callback)
{
    return messageCallbacks_.add_callback(callback);
}

template <typename T>
bool SonarDriver<T>::remove_message_callback(unsigned int callbackId)
{
    return messageCallbacks_.remove_callback(callbackId);
}

template <typename T>
bool SonarDriver<T>::on_next_message(const MessageCallbacks::CallbackT& callback)
{
    return messageCallbacks_.add_single_shot(callback);
}

/**
 * This is a synchronization primitive allowing for waiting for the sonar to be
 * ready for example.
 */
template <typename T>
bool SonarDriver<T>::wait_next_message()
{
    auto dummy = [](const OculusMessageHeader&, const std::vector<uint8_t>&) {};
    return this->on_next_message(dummy);
}

// status callbacks
template <typename T> template <typename F, class... Args>
unsigned int SonarDriver<T>::add_status_callback(F&& func, Args&&... args)
{
    return this->statusListener_.add_callback(func, args...);
}

template <typename T> template <typename F, class... Args>
bool SonarDriver<T>::on_next_status(F&& func, Args&&... args)
{
    return this->statusListener_.on_next_status(func, args...);
}

// ping callbacks
template <typename T> template <typename F, class... Args>
unsigned int SonarDriver<T>::add_ping_callback(F&& func, Args&&... args)
{
    // static_cast is to avoid infinite loop at type resolution at compile time
    return this->add_ping_callback(static_cast<const PingCallbacks::CallbackT&>(
        std::bind(func, args..., std::placeholders::_1, std::placeholders::_2)));
}

template <typename T> template <typename F, class... Args>
bool SonarDriver<T>::on_next_ping(F&& func, Args&&... args)
{
    return this->on_next_ping(static_cast<const PingCallbacks::CallbackT&>(
        std::bind(func, args..., std::placeholders::_1, std::placeholders::_2)));
}

// dummy callbacks
template <typename T> template <typename F, class... Args>
unsigned int SonarDriver<T>::add_dummy_callback(F&& func, Args&&... args)
{
    // static_cast is to avoid infinite loop at type resolution at compile time
    return this->add_dummy_callback(static_cast<const DummyCallbacks::CallbackT&>(
        std::bind(func, args..., std::placeholders::_1)));
}

template <typename T> template <typename F, class... Args>
bool SonarDriver<T>::on_next_dummy(F&& func, Args&&... args)
{
    return this->on_next_dummy(static_cast<const DummyCallbacks::CallbackT&>(
        std::bind(func, args..., std::placeholders::_1)));
}

// generic messages callbacks
template <typename T> template <typename F, class... Args>
unsigned int SonarDriver<T>::add_message_callback(F&& func, Args&&... args)
{
    // static_cast is to avoid infinite loop at type resolution at compile time
    return this->add_message_callback(static_cast<const MessageCallbacks::CallbackT&>(
        std::bind(func, args..., std::placeholders::_1, std::placeholders::_2)));
}

template <typename T> template <typename F, class... Args>
bool SonarDriver<T>::on_next_message(F&& func, Args&&... args)
{
    return this->on_next_message(static_cast<const MessageCallbacks::CallbackT&>(
        std::bind(func, args..., std::placeholders::_1, std::placeholders::_2)));
}


}; //namespace oculus
}; //namespace narval

#endif //_DEF_NARVAL_OCULUS_SONAR_DRIVER_H_


