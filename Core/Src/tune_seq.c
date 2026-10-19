#include "tune_seq.h"

static uint32_t hold_ms(const TuneSeqCfg *cfg)
{
    return cfg->step_ms > TUNE_HOLD_MIN_MS ? cfg->step_ms : TUNE_HOLD_MIN_MS;
}

uint32_t TuneSeq_DurationMs(const TuneSeqCfg *cfg)
{
    return TUNE_LEAD_MS + (uint32_t)cfg->reps * 2u * ((uint32_t)cfg->step_ms + hold_ms(cfg));
}

void TuneSeq_Start(TuneSeq *s, const TuneSeqCfg *cfg, uint32_t now_ms)
{
    s->cfg      = *cfg;
    s->start_ms = now_ms;
    s->active   = 1;
}

void TuneSeq_Stop(TuneSeq *s)
{
    s->active = 0;
}

uint8_t TuneSeq_Step(TuneSeq *s, uint32_t now_ms, float *ref, uint8_t *segment)
{
    *ref = 0.0f;
    *segment = 0;
    if (!s->active) {
        return 0;
    }

    const uint32_t t = now_ms - s->start_ms;
    if (t >= TuneSeq_DurationMs(&s->cfg)) {
        s->active = 0;
        return 0;
    }
    if (t < TUNE_LEAD_MS) {
        return 1;
    }

    const uint32_t step = s->cfg.step_ms;
    const uint32_t hold = hold_ms(&s->cfg);
    const uint32_t rep_ms = 2u * (step + hold);
    const uint32_t in_run = t - TUNE_LEAD_MS;
    const uint32_t rep = in_run / rep_ms;
    const uint32_t in_rep = in_run % rep_ms;

    uint32_t seg_in_rep;
    if (in_rep < step) {
        seg_in_rep = 0;
        *ref = s->cfg.amp;
    } else if (in_rep < step + hold) {
        seg_in_rep = 1;
    } else if (in_rep < 2u * step + hold) {
        seg_in_rep = 2;
        *ref = -s->cfg.amp;
    } else {
        seg_in_rep = 3;
    }
    *segment = (uint8_t)(rep * TUNE_SEGS_PER_REP + seg_in_rep + 1u);
    return 1;
}
