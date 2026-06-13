#pragma once

#include <stdint.h>
#include <string.h>

#if defined(__GNUC__) || defined(__CC_ARM) || defined(__ICCARM__)
extern "C" {
    #include "cmsis_compiler.h"
}
#else
    #define __get_PRIMASK() 0
    #define __disable_irq()
    #define __set_PRIMASK(x)
#endif

namespace uORB {

/**
 * @brief Topic - Nơi lưu trữ dữ liệu thực sự (Shared Memory Block)
 */
template <typename T, uint8_t MAX_INSTANCES = 3>
class Topic {
public:
    Topic() {
        memset(_data, 0, sizeof(_data));
        memset((void*)_update_counter, 0, sizeof(_update_counter));
        memset((void*)_registered, 0, sizeof(_registered));
    }

    // Đăng ký instance mới (Trả về id)
    uint8_t register_instance() {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uint8_t id = 255;
        for (uint8_t i = 0; i < MAX_INSTANCES; i++) {
            if (!_registered[i]) {
                _registered[i] = true;
                id = i;
                break;
            }
        }
        __set_PRIMASK(primask);
        return id;
    }

    // Publisher sẽ gọi hàm này theo instance_id
    void publish(uint8_t instance, const T& msg) {
        if (instance >= MAX_INSTANCES) return;
        uint32_t primask = __get_PRIMASK();
        __disable_irq(); 
        
        _data[instance] = msg;
        _update_counter[instance]++;
        
        __set_PRIMASK(primask); 
    }

    void copy(uint8_t instance, T& msg, uint32_t& counter_out) const {
        if (instance >= MAX_INSTANCES) return;
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        
        msg = _data[instance];
        counter_out = _update_counter[instance];
        
        __set_PRIMASK(primask);
    }

    uint32_t get_update_count(uint8_t instance) const {
        if (instance >= MAX_INSTANCES) return 0;
        return _update_counter[instance];
    }

private:
    T _data[MAX_INSTANCES];
    volatile uint32_t _update_counter[MAX_INSTANCES];
    volatile bool _registered[MAX_INSTANCES];
};

template <typename T>
class PublicationMulti {
public:
    PublicationMulti(Topic<T>& topic) : _topic(topic) {
        _instance = _topic.register_instance();
    }

    bool publish(const T& msg) {
        if (_instance == 255) return false; // Hết slot
        _topic.publish(_instance, msg);
        return true;
    }
private:
    Topic<T>& _topic;
    uint8_t _instance;
};

template <typename T>
class SubscriptionMulti {
public:
    SubscriptionMulti(Topic<T>& topic, uint8_t instance) : _topic(topic), _instance(instance), _last_seen_counter(0) {}

    bool updated() const {
        if (_instance == 255) return false;
        return _topic.get_update_count(_instance) != _last_seen_counter;
    }

    bool update(T& msg) {
        if (updated()) {
            _topic.copy(_instance, msg, _last_seen_counter);
            return true;
        }
        return false;
    }

    void copy(T& msg) {
        if (_instance == 255) return;
        _topic.copy(_instance, msg, _last_seen_counter);
    }

private:
    Topic<T>& _topic;
    uint8_t _instance;
    uint32_t _last_seen_counter;
};

} // namespace uORB
