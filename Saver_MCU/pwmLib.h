
#include <stdint.h>

class pwmChannel {
    private:
        uint32_t *arr_ptr;
        uint32_t *ccr_ptr;
        uint32_t min_ccr;
        uint32_t max_ccr;
        uint32_t pwm_target;
        uint32_t pwm_current;

        uint32_t rate;
        uint32_t last_ms;

    public:
        pwmChannel(uint32_t *arr, uint32_t *ccr, uint32_t base_freq, uint32_t out_freq, uint32_t min_ccr, uint32_t max_ccr);
        void setPwm(float percent);
        void setTgt(float percent);
        void setSignedTgt(float percent);
        void update();

}


