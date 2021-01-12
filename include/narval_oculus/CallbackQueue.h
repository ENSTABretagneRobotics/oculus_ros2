#ifndef _DEF_NARVAL_OCULUS_CALLBACK_QUEUE_H_
#define _DEF_NARVAL_OCULUS_CALLBACK_QUEUE_H_

#include <functional>
#include <unordered_map>
#include <mutex>
#include <condition_variable>

#include <thread>

namespace narval { namespace oculus {

template <class ...ArgTypes>
class CallbackQueue
{
    // Thread safe callback handler
    // /!\ call order not guarantied !

    public:

    using CallbackId    = unsigned int;
    using CallbackT     = std::function<void(ArgTypes...)>;
    using CallbackDict  = std::unordered_map<CallbackId, CallbackT>;

    // deleted functions to prevent copy
    CallbackQueue(const CallbackQueue&) = delete;
    CallbackQueue& operator=(const CallbackQueue&) = delete;

    protected:
    
    CallbackDict       callbacks_;
    mutable std::mutex mutex_;

    CallbackDict            singleShots_;
    std::mutex              sShotsMutex_;
    std::condition_variable sShotsCv_;
    bool                    sShotsCalled_;

    public:

    CallbackQueue() {}

    CallbackId add_callback(const CallbackT& callback);
    bool remove_callback(CallbackId index);
    
    bool add_single_shot(const CallbackT& callback);

    void call(ArgTypes... args);
};

template <class ...ArgTypes>
typename CallbackQueue<ArgTypes...>::CallbackId 
CallbackQueue<ArgTypes...>::add_callback(const CallbackT& callback)
{
    const std::lock_guard<std::mutex> lock(mutex_); // will release mutex when out of scope

    // finding index value not already used
    CallbackId newId = callbacks_.size();
    for(; callbacks_.find(newId) != callbacks_.end(); newId++);

    callbacks_[newId] = callback;
    return newId;
}

template <class ...ArgTypes>
bool CallbackQueue<ArgTypes...>::remove_callback(CallbackId index)
{
    const std::lock_guard<std::mutex> lock(mutex_); // will release mutex when out of scope
    return callbacks_.erase(index) > 0;
}

template <class ...ArgTypes>
bool CallbackQueue<ArgTypes...>::add_single_shot(const CallbackT& callback)
{
    std::unique_lock<std::mutex> lock(sShotsMutex_);
    // Using a single boolean to check for spurious wake up for all threads
    // works because the lock is reaquired before sShotsCv_.wait exists.
    // Therefore, the sShotsMutex_ won't be available, i.e. sShotsCalled will
    // retain its value until all already registered singleShots_ callbacks are
    // returned.
    sShotsCalled_ = false;

    CallbackId callbackId = singleShots_.size();
    for(; singleShots_.find(callbackId) != singleShots_.end(); callbackId++);
    singleShots_[callbackId] = callback;

    sShotsCv_.wait(lock, [&]{return sShotsCalled_; });
    
    return sShotsCalled_;
}

template <class ...ArgTypes>
void CallbackQueue<ArgTypes...>::call(ArgTypes... args)
{
    CallbackDict callbacks;
    
    {
        // Copying callbacks first allows to release the mutex while callbacks
        // are called. This allows callbacks to modify this object (
        // adding/removing callbacks...)  without creating deadlocks.
        const std::lock_guard<std::mutex> lock(mutex_); // will release mutex when out of scope
        callbacks = callbacks_;
    }
    
    // calling regular callbacks
    for(auto& item : callbacks) {
        item.second(args...);
    }

    // calling single shots
    {
        std::unique_lock<std::mutex> lock(sShotsMutex_);
        for(auto& item : singleShots_) {
            item.second(args...);
        }
        singleShots_.clear();
        sShotsCalled_ = true;
    }

    sShotsCv_.notify_all();
}

}; //namespace oculus
}; //namespace narval

#endif //_DEF_NARVAL_OCULUS_CALLBACK_QUEUE_H_
