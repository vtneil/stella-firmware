#ifndef LUNA_STATE_DEF_H
#define LUNA_STATE_DEF_H

namespace stella {
    enum class state_t : uint8_t {
        STARTUP = 0,
        IDLE,
        START_RECORD,
        RECORDING,
        STOP_RECORD
    };

    inline const char *state_string(const state_t state) {
        switch (state) {
            case state_t::STARTUP:
                return "STARTUP";
            case state_t::IDLE:
                return "IDLE";
            case state_t::START_RECORD:
                return "START_RECORD";
            case state_t::RECORDING:
                return "RECORING";
            case state_t::STOP_RECORD:
                return "STOP_RECORD";
            default:
                __builtin_unreachable();
        }
    }
}  // namespace luna

#endif  //LUNA_STATE_DEF_H
