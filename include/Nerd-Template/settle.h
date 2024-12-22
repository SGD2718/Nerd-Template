#include "vex.h"


class Settle {
public:
    Settle() = default;
    Settle(const SettleConfig& config, float dt = 10);
    
    void update(float error);

    bool is_settled() const;
    bool is_early_stop(float current) const;
    bool is_exit(float current) const;

    float get_time_spent_running() const;
    float get_time_spent_settled() const;
    float get_dt() const;

    float settle_error = 0;
    unsigned int settle_time = 0;
    unsigned int timeout = 0;
    float max_current = 2.5;

private:
    unsigned int time_spent_running = 0;
    unsigned int time_spent_settled = 0;
    unsigned int dt = 10;
};