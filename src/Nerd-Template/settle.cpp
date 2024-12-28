#include "vex.h"


Settle::Settle(const SettleConfig& config, float dt): 
        settle_error(config.settle_error), settle_time(config.settle_time), timeout(config.timeout), max_current(config.max_current), dt(roundf(dt)), current_moving_avg(0.f) {}
    
void Settle::update(float error) {
    if (fabsf(error) < this->settle_error)
        this->time_spent_settled += this->dt;
    else   
        this->time_spent_settled = 0;
    
    this->time_spent_running += this->dt;
}

bool Settle::is_settled() const {
    return this->settle_error != 0 && this->time_spent_settled > this->settle_time;
}

bool Settle::is_early_stop(float current) {
    this->current_moving_avg = this->current_moving_avg + (current - this->current_moving_avg) * 0.1;
    return (this->timeout != 0 && this->time_spent_running > this->timeout) || (this->max_current != 0 && fabsf(this->current_moving_avg) > this->max_current);
}

bool Settle::is_exit(float current) {
    return this->is_settled() || this->is_early_stop(current);
}

float Settle::get_time_spent_running() const {
    return this->time_spent_running;
}

float Settle::get_time_spent_settled() const {
    return this->time_spent_settled;
}

float Settle::get_dt() const {
    return this->dt;
}