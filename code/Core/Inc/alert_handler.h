#ifndef ALERT_HANDLER_H
#define ALERT_HANDLER_H

bool check_system(void);
void check_fault(uint8_t sys_stat, uint8_t bit1, uint8_t bit2, uint8_t wait_duration);
void alert_handler(uint8_t sys_stat);
#endif
