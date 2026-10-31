/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_FCW_TRIAL_H
#define SIMPLECLOCK_FCW_TRIAL_H
#include <stdbool.h>
#include <stdint.h>

/* `clock trial`: bounded actuator experiment, never a phase/frequency servo.
 * Counts use the SiT5801 signed 39-bit FCW, full scale 800000 ppb. Integer
 * floor rounding makes every 1-second step strictly <=0.05 ppb; the final
 * small step reaches the nearest representable +1 ppb exactly. */
#define SC_FCW_STEP_COUNTS INT64_C(17179)
#define SC_FCW_TEST_COUNTS INT64_C(343597)
#define SC_FCW_SETTLE_AFTER_BOOT_MS 1800000U

typedef enum
{
    SC_FCW_IDLE,
    SC_FCW_BASELINE,
    SC_FCW_UP,
    SC_FCW_HIGH,
    SC_FCW_DOWN,
    SC_FCW_RECOVERY,
    SC_FCW_DONE,
    SC_FCW_ABORTED,
    SC_FCW_FAULT
} sc_fcw_phase_t;

typedef struct
{
    int64_t original, current, requested;
    uint32_t phase_ms, applied_ms, writes;
    sc_fcw_phase_t phase;
    bool pending, aborted;
} sc_fcw_trial_t;

/* Zero-initialize once. Call in foreground with wrap-safe MCU milliseconds.
 * Start requires a valid fresh read, quality qualification and boot settling
 * supplied by the adapter. One experiment per boot; no automatic retries. */
bool sc_fcw_trial_start(sc_fcw_trial_t *s, int64_t original, uint16_t clip, uint32_t now_ms,
                        bool qualified);
bool sc_fcw_trial_active(const sc_fcw_trial_t *s);
/* A proposal reserves exactly one transaction. Acknowledge physical completion
 * through written(), then verify a fresh complete read with observed(). Never
 * advance from an I2C ACK alone. No catch-up writes after a scheduling stall. */
bool sc_fcw_trial_next(sc_fcw_trial_t *s, uint32_t now_ms, bool quality, int64_t *word);
void sc_fcw_trial_written(sc_fcw_trial_t *s, uint32_t now_ms, bool success);
void sc_fcw_trial_observed(sc_fcw_trial_t *s, uint32_t now_ms, bool valid, int64_t word);
/* Quality loss / operator abort returns toward the original at the same slew.
 * A bus or readback fault latches FAULT: actual applied word is then uncertain
 * and further writes require investigation, not a guessed restoration. */
void sc_fcw_trial_abort(sc_fcw_trial_t *s, uint32_t now_ms);
#endif
