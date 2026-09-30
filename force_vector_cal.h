#ifndef _FORCE_VECTOR_CAL_H_
#define _FORCE_VECTOR_CAL_H_

#include <stdbool.h>
#include <stdint.h>

/* Low-raw hardware needs a larger force-output scale without changing angle. */
#define SHEAR_KX (1.5f)
#define SHEAR_KY (1.5f)

/*
 * Touch detection thresholds (noise multipliers).
 * Larger = higher bar for touch on/off, fewer false triggers.
 * These affect whether the state machine enters TOUCH_ACTIVE.
 */
#define DIR_TOUCH_ON_SCALE (2.0f)
#define DIR_TOUCH_OFF_SCALE (0.75f)

/*
 * Soft deadzone: attenuates weak signals instead of hard-cutting them.
 *   SCALE * noise[i] = deadzone width. Larger = wider noise gate.
 *   LEAK = attenuation factor inside deadzone. Smaller = less noise passes.
 * For ultra-light touch: use a narrow deadzone and retain more signal in it.
 */
#define DIR_SOFT_DZ_SCALE (0.25f)
#define DIR_SOFT_DZ_LEAK (0.50f)

/*
 * Low-confidence hold: when confidence < this, direction filter is frozen.
 * Higher = more stable during weak signals, but less responsive.
 * Lower = more responsive but may jitter during light touch.
 * Low-raw sensors have less effective resolution, so release the direction
 * hold at low confidence to preserve ultra-light stroke response.
 */
#define DIR_CONF_HOLD_TH (0.12f)

/*
 * While confidence is still below the hold threshold, keep a very slow
 * internal vector follow so the angle does not jump when confidence recovers.
 */
#define DIR_ALPHA_PREHOLD (0.030f)

/*
 * Short-term vector consensus for ultra-light strokes.
 * A real stroke keeps roughly the same direction over adjacent frames, while
 * quantization/noise vectors cancel each other.  Confidence is multiplied by
 * this temporal coherence before it can release the direction hold.
 */
#define DIR_CONSENSUS_ALPHA (0.12f)

/*
 * Adaptive IIR filter coefficients for direction vector (vx_f, vy_f).
 *   alpha(C) = ALPHA_MIN + (ALPHA_MAX - ALPHA_MIN) * confidence
 * When confidence is low  → alpha ≈ 0.010 (smooth, suppress jitter)
 * When confidence is high → alpha ≈ 0.11 (responsive without chasing noise)
 */
#define DIR_ALPHA_MIN (0.010f)
#define DIR_ALPHA_MAX (0.11f)

/*
 * Secondary filter for the angle output itself.
 * This damps residual vector wobble so theta does not swing on every frame.
 */
#define DIR_THETA_ALPHA_MIN (0.025f)
#define DIR_THETA_ALPHA_MAX (0.07f)

/* Maximum angle movement per frame, in radians. */
#define DIR_THETA_STEP_MIN (0.012f)
#define DIR_THETA_STEP_MAX (0.050f)

/*
 * COP (Center of Pressure) denominator floor.
 */
#define DIR_COP_DEN_SCALE (32.0f)
#define DIR_COP_DEN_MIN (20.0f)
#define DIR_COP_DEN_MAX (200.0f)
#define DIR_EPS (1e-6f)

/*
 * Baseline following parameters.
 *
 * IDLE_ALPHA: How fast the idle baseline tracks slow environmental drift.
 *   Smaller = slower tracking, won't eat into weak touch signals.
 *   Time constant ≈ 1/alpha frames (0.05 → ~20 frames at 300Hz).
 */
#define BASE_IDLE_ALPHA (0.05f)

/* Release only after force returns to the measured rebound envelope. */
#define BASE_RELEASE_RATIO (0.20f)

/*
 * Rebound recovery: after release, material rebounds cause slow signal decay.
 *   CM  = common-mode rate (fast): pulls all 4 channels together toward idle.
 *   DIFF = differential-mode rate (slow): preserves direction info during recovery.
 *   Ratio CM/DIFF is about 6, meaning common-mode recovers faster than differential.
 */
#define BASE_RECOVER_ALPHA_CM (0.08f)
#define BASE_RECOVER_ALPHA_DIFF (0.015f)

/*
 * Recovery complete when residual < noise * 4 * SCALE for STABLE_FRAMES.
 */
#define BASE_RECOVER_DONE_SCALE (2.5f)
#define BASE_RECOVER_STABLE_FRAMES (10u)

/*
 * Re-touch detection during rebound: signal must rise above threshold
 * AND have positive slope > noise * 4 * SCALE.
 */
#define BASE_RETOUCH_RISE_SCALE (2.0f)

/*
 * Contact-time creep observer.
 *
 * A real force transition moves the 4-channel state faster than silicone
 * creep.  Classify the 4-channel displacement rate every short window: slow
 * motion advances the contact-time creep baseline, fast motion changes force.
 */
#define FORCE_OBSERVER_WINDOW_FRAMES (4u)
#define FORCE_OBSERVER_CREEP_NOISE_SCALE (2.0f)
#define FORCE_OBSERVER_CREEP_MIN_COUNTS_PER_FRAME (16.0f)

/*
 * Per-channel K (gain) normalization coefficients.
 * Compensate for sensitivity differences across the 4 sensor positions.
 * 1.0 = no correction. Increase for less sensitive positions.
 *
 *   K00  K10       p00=ch6, p01=ch7, p10=ch4, p11=ch5  (8-ch layout)
 *   K01  K11
 */
#define FV_CHANNEL_K_00 (1.0f)
#define FV_CHANNEL_K_01 (1.0f)
#define FV_CHANNEL_K_10 (1.0f)
#define FV_CHANNEL_K_11 (1.0f)

typedef enum {
	BASE_STATE_IDLE = 0,
	BASE_STATE_TOUCH_ACTIVE = 1,
	BASE_STATE_REBOUND_RECOVERY = 2,
} base_track_state_t;

typedef struct {
	base_track_state_t base_state;
	float baseline_idle[4];
	float baseline_for_touch[4];
	float baseline_touch[4];
	float observed_delta[4];
	float creep_offset[4];
	float force_delta[4];
	float s_now;
	float s_prev;
	float s_peak;
	float vx_f;
	float vy_f;
	float consensus_x;
	float consensus_y;
	float dir_amp_envelope;
	float theta;
	float confidence;
	float confidence_smoothed;
	uint16_t recover_stable_cnt;
	uint8_t touched;
	uint8_t theta_valid;
	uint8_t force_observer_valid;
	uint8_t force_window_cnt;
} dir_4ch_state_t;

typedef struct {
	float theta;
	float tangential_force;
	float confidence;
	float normal_force;
	float cop_x;
	float cop_y;
	float diff_x;
	float diff_y;
	float force_x;
	float force_y;
	float p00;
	float p01;
	float p10;
	float p11;
} force_vector_debug_t;

typedef struct {
	dir_4ch_state_t dir_state;
	bool initialized;
	bool touch_detection_enabled;
	bool contact_active;
	float theta;
	float tangential_force;
	float confidence;
	force_vector_debug_t debug;
} force_vector_context_t;

void force_vector_context_reset(force_vector_context_t *context);
/* noise参数保留兼容；内部统一按每通道固定噪声1进行K换算。 */
void force_vector_context_cal(force_vector_context_t *context, const float raw[4], const float noise[4]);
void force_vector_context_set_touch_detection_enabled(force_vector_context_t *context, bool enabled);
void force_vector_context_force_recovery(force_vector_context_t *context);
void force_vector_context_confirm_recontact(force_vector_context_t *context);
void force_vector_context_finish_recovery(force_vector_context_t *context);
void force_vector_context_get_idle_baseline(const force_vector_context_t *context, float baseline[4]);

void force_vector_reset(void);
/* 旧接口同样采用固定噪声，不使用传入的noise数组调整门槛。 */
void force_vector_cal(const float raw[4], const float noise[4]);

extern bool g_force_vector_contact_active;
extern float g_theta;
extern float g_tangential_force;
extern float g_confidence;
extern force_vector_debug_t g_force_vector_debug;

#endif /* _FORCE_VECTOR_CAL_H_ */
