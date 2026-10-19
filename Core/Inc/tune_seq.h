#ifndef __TUNE_SEQ_H__
#define __TUNE_SEQ_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * Step-test sequencer — pure timing logic, no hardware (host-tested).
 *
 * One rep is a doublet, so a rate test rotates the frame out and back and
 * an angle test returns to level:
 *
 *   ref
 *   +amp       ┌──────┐
 *     0  ──────┘ seg1 └─seg2──┐      ┌─seg4──  ... next rep
 *   -amp                      └seg3──┘
 *        lead   step   hold    step   hold
 *
 *   lead = TUNE_LEAD_MS at 0 (segment 0), step = step_ms,
 *   hold = max(step_ms, TUNE_HOLD_MIN_MS)
 *
 * Segments are numbered 1, 2, 3 ... through the whole run, and every T log
 * line carries its segment, so the PC analyzer can cut the log into steps.
 */

#define TUNE_LEAD_MS      500u
#define TUNE_HOLD_MIN_MS  500u
#define TUNE_SEGS_PER_REP 4u

typedef struct {
    float    amp;        /* deg/s or deg, sign = direction of the first step */
    uint16_t step_ms;
    uint8_t  reps;
} TuneSeqCfg;

typedef struct {
    TuneSeqCfg cfg;
    uint32_t   start_ms;
    uint8_t    active;
} TuneSeq;

void TuneSeq_Start(TuneSeq *s, const TuneSeqCfg *cfg, uint32_t now_ms);
void TuneSeq_Stop(TuneSeq *s);

/* Advance to now_ms. Returns 1 while the run is in progress and writes the
 * reference value and segment number; returns 0 (ref 0, segment 0) once
 * the last hold has ended or when stopped. */
uint8_t TuneSeq_Step(TuneSeq *s, uint32_t now_ms, float *ref, uint8_t *segment);

uint32_t TuneSeq_DurationMs(const TuneSeqCfg *cfg);

#ifdef __cplusplus
}
#endif

#endif /* __TUNE_SEQ_H__ */
