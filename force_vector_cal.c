#include "force_vector_cal.h"

/* 三维力算法与通道状态机共用固定噪声策略。 */
#include "sensor_algorithm.h"

#include <math.h>

typedef struct {
	float delta_idle[4];
	float delta_touch[4];
	float q_out[4];
	float d_soft[4];
	float u[4];
	float vx;
	float vy;
	float dir_amp;
	float avg_noise;
	float touch_energy;
	float q_out_sum;
	float cop_x;
	float cop_y;
} dir_4ch_debug_t;

bool g_force_vector_contact_active = false;
float g_theta = 0.0f;
float g_tangential_force = 0.0f;
float g_confidence = 0.0f;
force_vector_debug_t g_force_vector_debug = {0};

static force_vector_context_t g_default_context = {0};
static const float k_pi_f = 3.14159265358979323846f;
static const float k_two_pi_f = 6.28318530717958647692f;

static float vector_magnitude(float x, float y) {
	return sqrtf((x * x) + (y * y));
}

static float clampf(float value, float min_value, float max_value) {
	if (value < min_value) {
		return min_value;
	}

	if (value > max_value) {
		return max_value;
	}

	return value;
}

static float wrap_angle_pi(float angle) {
	while (angle > k_pi_f) {
		angle -= k_two_pi_f;
	}

	while (angle < -k_pi_f) {
		angle += k_two_pi_f;
	}

	return angle;
}

static void clear_force_vector_debug(force_vector_debug_t *debug) {
	debug->theta = 0.0f;
	debug->tangential_force = 0.0f;
	debug->confidence = 0.0f;
	debug->normal_force = 0.0f;
	debug->cop_x = 0.0f;
	debug->cop_y = 0.0f;
	debug->diff_x = 0.0f;
	debug->diff_y = 0.0f;
	debug->force_x = 0.0f;
	debug->force_y = 0.0f;
	debug->p00 = 0.0f;
	debug->p01 = 0.0f;
	debug->p10 = 0.0f;
	debug->p11 = 0.0f;
}

static void copy4(float dst[4], const float src[4]) {
	unsigned int index;

	for (index = 0u; index < 4u; index++) {
		dst[index] = src[index];
	}
}

static void clear4(float values[4]) {
	unsigned int index;

	for (index = 0u; index < 4u; index++) {
		values[index] = 0.0f;
	}
}

static float soft_deadzone(float x, float threshold, float leak) {
	float ax = fabsf(x);
	float sign = (x >= 0.0f) ? 1.0f : -1.0f;

	if (ax < threshold) {
		return sign * ax * leak;
	}

	return sign * ((threshold * leak) + (ax - threshold));
}

static void baseline_rebound_recover_4ch(float baseline[4],
                                         const float raw[4],
                                         float alpha_common,
                                         float alpha_diff) {
	float residual[4];
	float common = 0.0f;
	unsigned int index;

	for (index = 0u; index < 4u; index++) {
		residual[index] = raw[index] - baseline[index];
		common += residual[index];
	}

	common *= 0.25f;

	for (index = 0u; index < 4u; index++) {
		float diff = residual[index] - common;

		baseline[index] += (alpha_common * common) + (alpha_diff * diff);
	}
}

static void enter_touch_active(dir_4ch_state_t *state) {
	state->base_state = BASE_STATE_TOUCH_ACTIVE;
	state->touched = 1u;
	state->s_peak = state->s_now;
	state->recover_stable_cnt = 0u;
	state->vx_f = 0.0f;
	state->vy_f = 0.0f;
	state->consensus_x = 0.0f;
	state->consensus_y = 0.0f;
	state->dir_amp_envelope = 0.0f;
	state->confidence = 0.0f;
	state->confidence_smoothed = 0.0f;
	state->theta_valid = 0u;
	state->force_observer_valid = 0u;
	state->force_window_cnt = 0u;
	copy4(state->baseline_touch, state->baseline_for_touch);
}

static void enter_rebound_recovery(dir_4ch_state_t *state) {
	state->base_state = BASE_STATE_REBOUND_RECOVERY;
	state->touched = 0u;
	state->vx_f = 0.0f;
	state->vy_f = 0.0f;
	state->consensus_x = 0.0f;
	state->consensus_y = 0.0f;
	state->dir_amp_envelope = 0.0f;
	state->theta = 0.0f;
	state->confidence = 0.0f;
	state->confidence_smoothed = 0.0f;
	state->recover_stable_cnt = 0u;
	state->theta_valid = 0u;
	state->force_observer_valid = 0u;
	state->force_window_cnt = 0u;
}

static void initialize_direction_state_if_needed(dir_4ch_state_t *state, bool *initialized, const float raw[4]) {
	unsigned int index;

	if (*initialized) {
		return;
	}

	for (index = 0u; index < 4u; index++) {
		state->baseline_idle[index] = raw[index];
		state->baseline_for_touch[index] = raw[index];
		state->baseline_touch[index] = raw[index];
	}

	state->base_state = BASE_STATE_IDLE;
	state->s_now = 0.0f;
	state->s_prev = 0.0f;
	state->s_peak = 0.0f;
	state->vx_f = 0.0f;
	state->vy_f = 0.0f;
	state->consensus_x = 0.0f;
	state->consensus_y = 0.0f;
	state->dir_amp_envelope = 0.0f;
	state->theta = 0.0f;
	state->confidence = 0.0f;
	state->confidence_smoothed = 0.0f;
	state->recover_stable_cnt = 0u;
	state->touched = 0u;
	state->theta_valid = 0u;
	state->force_observer_valid = 0u;
	state->force_window_cnt = 0u;
	clear4(state->observed_delta);
	clear4(state->creep_offset);
	clear4(state->force_delta);
	*initialized = true;
}

static void update_force_observer(dir_4ch_state_t *state,
								  const float observed_delta[4],
								  float avg_noise) {
	float window_square_sum = 0.0f;
	float creep_speed_limit = fmaxf(
		avg_noise * FORCE_OBSERVER_CREEP_NOISE_SCALE,
		FORCE_OBSERVER_CREEP_MIN_COUNTS_PER_FRAME);
	unsigned int index;

	if (!state->force_observer_valid) {
		copy4(state->observed_delta, observed_delta);
		clear4(state->creep_offset);
		copy4(state->force_delta, observed_delta);
		state->force_observer_valid = 1u;
		state->force_window_cnt = 0u;
		return;
	}

	if (state->force_window_cnt < FORCE_OBSERVER_WINDOW_FRAMES) {
		state->force_window_cnt++;
	}
	if (state->force_window_cnt < FORCE_OBSERVER_WINDOW_FRAMES) {
		return;
	}

	for (index = 0u; index < 4u; index++) {
		float window_delta = observed_delta[index] - state->observed_delta[index];

		window_square_sum += window_delta * window_delta;
	}

	if ((sqrtf(window_square_sum * 0.25f) / (float)FORCE_OBSERVER_WINDOW_FRAMES)
		<= creep_speed_limit) {
		/* Slow displacement belongs to silicone creep, so move its baseline. */
		for (index = 0u; index < 4u; index++) {
			state->creep_offset[index] += observed_delta[index] - state->observed_delta[index];
		}
	}

	for (index = 0u; index < 4u; index++) {
		state->force_delta[index] = observed_delta[index] - state->creep_offset[index];
	}
	copy4(state->observed_delta, observed_delta);
	state->force_window_cnt = 0u;
}

static void calc_direction_4ch(dir_4ch_state_t *state,
                               const float raw[4],
                               const float inv_k[4],
                               const float noise[4],
                               bool touch_detection_enabled,
                               float *theta_out,
                               float *conf_out,
                               dir_4ch_debug_t *debug_out) {
	static const float sensor_x[4] = {-1.0f, -1.0f, 1.0f, 1.0f};
	static const float sensor_y[4] = {1.0f, -1.0f, 1.0f, -1.0f};
	float noise_norm[4];
	float touch_on_th;
	float touch_off_th;
	float recover_done_th;
	float mean = 0.0f;
	float noise_dir;
	float consensus_amp;
	float direction_coherence;
	float residual = 0.0f;
	float ds = 0.0f;
	float alpha;
	float theta_alpha;
	unsigned int index;
	bool retouch_detected = false;
	bool release_by_threshold = false;
	bool release_by_drop = false;

	clear4(debug_out->delta_idle);
	clear4(debug_out->delta_touch);
	clear4(debug_out->q_out);
	clear4(debug_out->d_soft);
	clear4(debug_out->u);
	debug_out->vx = 0.0f;
	debug_out->vy = 0.0f;
	debug_out->dir_amp = 0.0f;
	debug_out->avg_noise = 0.0f;
	debug_out->touch_energy = 0.0f;
	debug_out->q_out_sum = 0.0f;
	debug_out->cop_x = 0.0f;
	debug_out->cop_y = 0.0f;

	/* 保留旧接口参数，但不再让外部噪声值改变任何力计算门槛。 */
	(void)noise;
	for (index = 0u; index < 4u; index++) {
		debug_out->delta_idle[index] = (raw[index] - state->baseline_idle[index]) * inv_k[index];

		/* 原始域噪声固定为1，仍按各通道K系数换算到力算法数值域。 */
		noise_norm[index] = (float)SENSOR_ALGO_FIXED_NOISE * inv_k[index];
		debug_out->avg_noise += noise_norm[index];

		/* Contact deformation is signed; both compression polarities carry energy. */
		debug_out->touch_energy += fabsf(debug_out->delta_idle[index]);
	}

	debug_out->avg_noise *= 0.25f;
	state->s_now = debug_out->touch_energy;
	touch_on_th = debug_out->avg_noise * 4.0f * DIR_TOUCH_ON_SCALE;
	touch_off_th = debug_out->avg_noise * 4.0f * DIR_TOUCH_OFF_SCALE;
	recover_done_th = debug_out->avg_noise * 4.0f * BASE_RECOVER_DONE_SCALE;

	switch (state->base_state) {
	case BASE_STATE_IDLE:
		for (index = 0u; index < 4u; index++) {
			state->baseline_idle[index] += (raw[index] - state->baseline_idle[index]) * BASE_IDLE_ALPHA;
		}
		copy4(state->baseline_for_touch, state->baseline_idle);

		if (touch_detection_enabled && (state->s_now > touch_on_th)) {
			enter_touch_active(state);
		}
		break;

	case BASE_STATE_TOUCH_ACTIVE:
		if (state->s_now > state->s_peak) {
			state->s_peak = state->s_now;
		}

		release_by_threshold = (state->s_now < touch_off_th);
		release_by_drop = (state->s_peak > touch_on_th)
			&& (state->s_now < (state->s_peak * BASE_RELEASE_RATIO));
		if (release_by_threshold || release_by_drop) {
			enter_rebound_recovery(state);
		}
		break;

	case BASE_STATE_REBOUND_RECOVERY:
		baseline_rebound_recover_4ch(state->baseline_idle, raw, BASE_RECOVER_ALPHA_CM, BASE_RECOVER_ALPHA_DIFF);

		ds = state->s_now - state->s_prev;
		retouch_detected = (state->s_now > (touch_on_th * 1.2f))
			&& (ds > (debug_out->avg_noise * 4.0f * BASE_RETOUCH_RISE_SCALE));

		if (touch_detection_enabled && retouch_detected) {
			enter_touch_active(state);
			break;
		}

		for (index = 0u; index < 4u; index++) {
			residual += fabsf((raw[index] - state->baseline_idle[index]) * inv_k[index]);
		}

		if (residual < recover_done_th) {
			state->recover_stable_cnt++;
		} else {
			state->recover_stable_cnt = 0u;
		}

		if (state->recover_stable_cnt >= BASE_RECOVER_STABLE_FRAMES) {
			copy4(state->baseline_for_touch, state->baseline_idle);
			state->base_state = BASE_STATE_IDLE;
		}
		break;
	}

	if (state->base_state != BASE_STATE_TOUCH_ACTIVE) {
		state->touched = 0u;
		state->confidence = 0.0f;
		state->confidence_smoothed = 0.0f;
		*theta_out = 0.0f;
		*conf_out = 0.0f;
		state->s_prev = state->s_now;
		return;
	}

	state->touched = 1u;
	debug_out->touch_energy = state->s_now;

	for (index = 0u; index < 4u; index++) {
		debug_out->delta_touch[index] = (raw[index] - state->baseline_touch[index]) * inv_k[index];
	}
	update_force_observer(state, debug_out->delta_touch, debug_out->avg_noise);

	for (index = 0u; index < 4u; index++) {
		float threshold;

		debug_out->q_out[index] = (state->force_delta[index] > 0.0f) ? state->force_delta[index] : 0.0f;
		debug_out->q_out_sum += debug_out->q_out[index];
		threshold = noise_norm[index] * DIR_SOFT_DZ_SCALE;
		debug_out->d_soft[index] = soft_deadzone(state->force_delta[index], threshold, DIR_SOFT_DZ_LEAK);
		mean += debug_out->d_soft[index];
	}

	mean *= 0.25f;

	for (index = 0u; index < 4u; index++) {
		debug_out->u[index] = debug_out->d_soft[index] - mean;
		debug_out->vx += debug_out->u[index] * sensor_x[index];
		debug_out->vy += debug_out->u[index] * sensor_y[index];
	}

	debug_out->dir_amp = vector_magnitude(debug_out->vx, debug_out->vy);
	state->consensus_x += (debug_out->vx - state->consensus_x) * DIR_CONSENSUS_ALPHA;
	state->consensus_y += (debug_out->vy - state->consensus_y) * DIR_CONSENSUS_ALPHA;
	state->dir_amp_envelope += (debug_out->dir_amp - state->dir_amp_envelope) * DIR_CONSENSUS_ALPHA;
	consensus_amp = vector_magnitude(state->consensus_x, state->consensus_y);
	direction_coherence = clampf(consensus_amp / (state->dir_amp_envelope + DIR_EPS), 0.0f, 1.0f);
	/*
	 * Conservative noise floor: avg_noise * 4 * 2.4 = avg_noise * 9.6.
	 * Higher multiplier → lower raw confidence → lower adaptive alpha → smoother.
	 * This prevents light-touch jitter without needing to freeze direction entirely.
	 */
	noise_dir = debug_out->avg_noise * 4.0f * 2.4f;
	state->confidence = debug_out->dir_amp / (debug_out->dir_amp + noise_dir + DIR_EPS);
	state->confidence *= direction_coherence;

	/*
	 * Smooth confidence with a slow IIR to prevent oscillation around the
	 * hold threshold. alpha_c = 0.1 — responds to genuine changes but
	 * filters out frame-to-frame noise in the confidence estimate.
	 */
	state->confidence_smoothed += 0.1f * (state->confidence - state->confidence_smoothed);

	if (state->confidence_smoothed < DIR_CONF_HOLD_TH) {
		state->vx_f += (state->consensus_x - state->vx_f) * DIR_ALPHA_PREHOLD;
		state->vy_f += (state->consensus_y - state->vy_f) * DIR_ALPHA_PREHOLD;
		*theta_out = state->theta;
		*conf_out = state->confidence_smoothed;
		state->s_prev = state->s_now;
		return;
	}

	alpha = DIR_ALPHA_MIN + ((DIR_ALPHA_MAX - DIR_ALPHA_MIN) * state->confidence_smoothed);
	state->vx_f += (state->consensus_x - state->vx_f) * alpha;
	state->vy_f += (state->consensus_y - state->vy_f) * alpha;

	if (vector_magnitude(state->vx_f, state->vy_f) > DIR_EPS) {
		float theta_target = atan2f(state->vy_f, state->vx_f);

		if (!state->theta_valid) {
			state->theta = theta_target;
			state->theta_valid = 1u;
		} else {
			float theta_error;
			float theta_step;
			float theta_step_limit;

			theta_alpha = DIR_THETA_ALPHA_MIN + ((DIR_THETA_ALPHA_MAX - DIR_THETA_ALPHA_MIN) * state->confidence_smoothed);
			theta_error = wrap_angle_pi(theta_target - state->theta);
			theta_step = theta_error * theta_alpha;
			theta_step_limit = DIR_THETA_STEP_MIN
				+ ((DIR_THETA_STEP_MAX - DIR_THETA_STEP_MIN) * state->confidence_smoothed);
			theta_step = clampf(theta_step, -theta_step_limit, theta_step_limit);
			state->theta = wrap_angle_pi(state->theta + theta_step);
		}
	}

	*theta_out = state->theta;
	*conf_out = state->confidence_smoothed;
	state->s_prev = state->s_now;
}

void force_vector_context_reset(force_vector_context_t *context) {
	unsigned int index;

	for (index = 0u; index < 4u; index++) {
		context->dir_state.baseline_idle[index] = 0.0f;
		context->dir_state.baseline_for_touch[index] = 0.0f;
		context->dir_state.baseline_touch[index] = 0.0f;
	}

	context->dir_state.base_state = BASE_STATE_IDLE;
	context->dir_state.s_now = 0.0f;
	context->dir_state.s_prev = 0.0f;
	context->dir_state.s_peak = 0.0f;
	context->dir_state.vx_f = 0.0f;
	context->dir_state.vy_f = 0.0f;
	context->dir_state.consensus_x = 0.0f;
	context->dir_state.consensus_y = 0.0f;
	context->dir_state.dir_amp_envelope = 0.0f;
	context->dir_state.theta = 0.0f;
	context->dir_state.confidence = 0.0f;
	context->dir_state.confidence_smoothed = 0.0f;
	context->dir_state.recover_stable_cnt = 0u;
	context->dir_state.touched = 0u;
	context->dir_state.theta_valid = 0u;
	context->dir_state.force_observer_valid = 0u;
	context->dir_state.force_window_cnt = 0u;
	clear4(context->dir_state.observed_delta);
	clear4(context->dir_state.creep_offset);
	clear4(context->dir_state.force_delta);
	context->initialized = false;
	context->touch_detection_enabled = true;
	context->contact_active = false;
	context->theta = 0.0f;
	context->tangential_force = 0.0f;
	context->confidence = 0.0f;
	clear_force_vector_debug(&context->debug);
}

void force_vector_context_set_touch_detection_enabled(force_vector_context_t *context, bool enabled) {
	context->touch_detection_enabled = enabled;
}

void force_vector_context_force_recovery(force_vector_context_t *context) {
	if (!context->initialized) {
		return;
	}

	enter_rebound_recovery(&context->dir_state);
	context->contact_active = false;
	context->theta = 0.0f;
	context->tangential_force = 0.0f;
	context->confidence = 0.0f;
	clear_force_vector_debug(&context->debug);
}

void force_vector_context_confirm_recontact(force_vector_context_t *context) {
	if (!context->initialized) {
		return;
	}

	copy4(context->dir_state.baseline_for_touch, context->dir_state.baseline_idle);
	enter_touch_active(&context->dir_state);
	context->contact_active = true;
}

void force_vector_context_finish_recovery(force_vector_context_t *context) {
	if (!context->initialized) {
		return;
	}

	copy4(context->dir_state.baseline_for_touch, context->dir_state.baseline_idle);
	copy4(context->dir_state.baseline_touch, context->dir_state.baseline_idle);
	context->dir_state.base_state = BASE_STATE_IDLE;
	context->dir_state.s_now = 0.0f;
	context->dir_state.s_prev = 0.0f;
	context->dir_state.s_peak = 0.0f;
	context->dir_state.vx_f = 0.0f;
	context->dir_state.vy_f = 0.0f;
	context->dir_state.consensus_x = 0.0f;
	context->dir_state.consensus_y = 0.0f;
	context->dir_state.dir_amp_envelope = 0.0f;
	context->dir_state.theta = 0.0f;
	context->dir_state.confidence = 0.0f;
	context->dir_state.confidence_smoothed = 0.0f;
	context->dir_state.recover_stable_cnt = 0u;
	context->dir_state.touched = 0u;
	context->dir_state.theta_valid = 0u;
	context->dir_state.force_observer_valid = 0u;
	context->dir_state.force_window_cnt = 0u;
	clear4(context->dir_state.observed_delta);
	clear4(context->dir_state.creep_offset);
	clear4(context->dir_state.force_delta);
	context->contact_active = false;
	context->theta = 0.0f;
	context->tangential_force = 0.0f;
	context->confidence = 0.0f;
	clear_force_vector_debug(&context->debug);
}

void force_vector_context_get_idle_baseline(const force_vector_context_t *context, float baseline[4]) {
	copy4(baseline, context->dir_state.baseline_idle);
}

void force_vector_context_cal(force_vector_context_t *context, const float raw[4], const float noise[4]) {
	static const float inv_k[4] = {
		FV_CHANNEL_K_00,
		FV_CHANNEL_K_01,
		FV_CHANNEL_K_10,
		FV_CHANNEL_K_11,
	};
	static const float sensor_x[4] = {-1.0f, -1.0f, 1.0f, 1.0f};
	static const float sensor_y[4] = {1.0f, -1.0f, 1.0f, -1.0f};
	dir_4ch_debug_t debug_state;
	float theta;
	float confidence;
	float force_x;
	float force_y;
	float touch_weight_sum = 0.0f;
	float cop_floor;
	float cop_denom;
	bool output_active;
	unsigned int index;

	initialize_direction_state_if_needed(&context->dir_state, &context->initialized, raw);
	calc_direction_4ch(&context->dir_state, raw, inv_k, noise,
		context->touch_detection_enabled, &theta, &confidence, &debug_state);

	output_active = (context->dir_state.base_state == BASE_STATE_TOUCH_ACTIVE);
	force_x = SHEAR_KX * context->dir_state.vx_f;
	force_y = SHEAR_KY * context->dir_state.vy_f;
	context->tangential_force = output_active ? vector_magnitude(force_x, force_y) : 0.0f;
	context->theta = (context->tangential_force > 0.0f) ? theta : 0.0f;
	context->confidence = output_active ? confidence : 0.0f;
	context->contact_active = output_active;

	for (index = 0u; index < 4u; index++) {
		float weight = (debug_state.d_soft[index] > 0.0f) ? debug_state.d_soft[index] : 0.0f;

		touch_weight_sum += weight;
		debug_state.cop_x += weight * sensor_x[index];
		debug_state.cop_y += weight * sensor_y[index];
	}

	if (touch_weight_sum > 0.0f) {
		cop_floor = clampf(debug_state.avg_noise * DIR_COP_DEN_SCALE, DIR_COP_DEN_MIN, DIR_COP_DEN_MAX);
		cop_denom = (touch_weight_sum > cop_floor) ? touch_weight_sum : cop_floor;
		debug_state.cop_x /= cop_denom;
		debug_state.cop_y /= cop_denom;
	} else {
		debug_state.cop_x = 0.0f;
		debug_state.cop_y = 0.0f;
	}

	context->debug.theta = output_active ? context->theta : 0.0f;
	context->debug.tangential_force = context->tangential_force;
	context->debug.confidence = context->confidence;
	context->debug.normal_force = output_active ? debug_state.q_out_sum : 0.0f;
	context->debug.cop_x = output_active ? debug_state.cop_x : 0.0f;
	context->debug.cop_y = output_active ? debug_state.cop_y : 0.0f;
	context->debug.diff_x = output_active ? debug_state.vx : 0.0f;
	context->debug.diff_y = output_active ? debug_state.vy : 0.0f;
	context->debug.force_x = output_active ? force_x : 0.0f;
	context->debug.force_y = output_active ? force_y : 0.0f;
	context->debug.p00 = output_active ? debug_state.q_out[0] : 0.0f;
	context->debug.p01 = output_active ? debug_state.q_out[1] : 0.0f;
	context->debug.p10 = output_active ? debug_state.q_out[2] : 0.0f;
	context->debug.p11 = output_active ? debug_state.q_out[3] : 0.0f;
}

void force_vector_reset(void) {
	force_vector_context_reset(&g_default_context);
	g_force_vector_contact_active = false;
	g_theta = 0.0f;
	g_tangential_force = 0.0f;
	g_confidence = 0.0f;
	clear_force_vector_debug(&g_force_vector_debug);
}

void force_vector_cal(const float raw[4], const float noise[4]) {
	force_vector_context_cal(&g_default_context, raw, noise);
	g_force_vector_contact_active = g_default_context.contact_active;
	g_theta = g_default_context.theta;
	g_tangential_force = g_default_context.tangential_force;
	g_confidence = g_default_context.confidence;
	g_force_vector_debug = g_default_context.debug;
}
