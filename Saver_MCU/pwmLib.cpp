
#include <stdint.h>
#include <pwmLib.h>
/*
class pwmChannel {
    private:
        uint32_t *arr_ptr;
        uint32_t *ccr_ptr;
        uint32_t min_ccr;
        uint32_t max_ccr;
        uint32_t pwm_target;
        uint32_t pwm_current;

        uint32_t rate;

    public:
        pwmChannel(uint32_t *arr, uint32_t *ccr, uint32_t base_freq, uint32_t out_freq, uint32_t min_ccr, uint32_t max_ccr);
        setPercent(uint32_t percent);
        setPercentTgt(uint32_t percent);
        setSPercentTgt(int32_t percent);
        update();

}
*/


pwmChannel::pwmChannel(uint32_t *arr, uint32_t *ccr, uint32_t base_freq, uint32_t out_freq, uint32_t min_ccr, uint32_t max_ccr) {
    arr_ptr = arr;
    ccr_ptr = ccr;
    this.min_ccr = min_ccr;
    this.max_ccr = max_ccr;
    this.rate = 0;

    *arr_ptr = (base_freq / out_freq);
    *ccr_ptr = 0;
}

void pwmChannel::setSignedTgt(float percent) {
    uint32_t ccr_val;
    uint32_t range = max_ccr - min_ccr;
    if (percent > 1.0 || percent < -1.0) {
        return;
    }
    float offset = range * (percent + 1.0) / 2.0;
    ccr_target = min_ccr + (uint32_t)offset;
}

void pwmChannel::update(uint32_t ms_count) {
    uint32_t dt = ms_count - last_ms;
    last_ms = ms_count;

    
}