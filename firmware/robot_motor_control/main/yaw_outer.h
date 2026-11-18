
#pragma once

// Robot_Delivery: relative gyro-heading PID.
// REP-103: +Z = CCW (robot quay trái).
// Yaw là góc tương đối tích phân từ gyro,
// không phải hướng tuyệt đối trên bản đồ.
// PID vòng ngoài chỉ hiệu chỉnh angular.z.
// PID tốc độ hai bánh chịu trách nhiệm điều khiển PWM.

#include <math.h>
#include <stdint.h>

namespace YawOuter {

inline float clamp(float v, float a, float b) {
    return v < a ? a : (v > b ? b : v);
}

inline float wrap(float a) {
    return atan2f(sinf(a), cosf(a));
}

// Chuyển correction angular.z sang RPM hai bánh.
struct WheelRpmDelta {
    float left = 0;
    float right = 0;
};

inline WheelRpmDelta wheelRpmDelta(
    float delta_omega,
    float track,
    float radius_left,
    float radius_right
) {
    if (!isfinite(delta_omega) ||
        !isfinite(track) ||
        !isfinite(radius_left) ||
        !isfinite(radius_right) ||
        track <= 0 ||
        radius_left <= 0 ||
        radius_right <= 0) {
        return {};
    }

    constexpr float rad_to_rpm =
        60.0f / (2.0f * 3.14159265358979323846f);

    return {
        -0.5f * delta_omega * track *
            rad_to_rpm / radius_left,

         0.5f * delta_omega * track *
            rad_to_rpm / radius_right
    };
}

// Giữ nguyên cấu trúc Result để tương thích debug.
struct Result {
    float omega_out = 0;
    float correction = 0;
    float heading_err = 0;
    float gyro_z = 0;

    float yaw_rad = 0;
    float heading_ref_rad = 0;
    float rate_corr = 0;
    float heading_corr = 0;

    float requested_w = 0;
    float path_corr = 0;
    float cross_track_m = 0;
    float along_track_m = 0;

    float p_term = 0;
    float i_term = 0;
    float d_term = 0;

    bool active = false;
    bool saturated = false;
    bool slew_limited = false;

    float x_ref_m = 0;
    float y_ref_m = 0;
    float x_est_m = 0;
    float y_est_m = 0;

    float curvature_inv_m = 0;
    float radius_m = 0;

    // 0 = IDLE
    // 1 = Straight
    // 2 = Turn
    // 3 = IMU / control fallback
    int mode = 0;
};

class Controller {

    float yaw_ = 0;
    float heading_ref_ = 0;
    float gyro_ = 0;
    float last_corr_ = 0;
    float integral_ = 0;

    bool correction_active_ = false;

    float x_ref_ = 0;
    float y_ref_ = 0;
    float x_est_ = 0;
    float y_est_ = 0;
    float last_est_yaw_ = 0;

    uint64_t last_stamp_us_ = 0;
    uint32_t last_seq_ = 0;

    bool observed_ = false;
    bool motion_active_ = false;
    bool reacquire_ = false;

    int mode_ = 0;

public:

    // ==========================================
    // 1. THÔNG SỐ PID YAW VÒNG NGOÀI
    // ==========================================

    static constexpr float KP_HEADING = 0.50f;
    static constexpr float KP_RATE = 0.10f;

    struct Gains {
        float kp = KP_HEADING;
        float ki = 0.06f;
        float kd = KP_RATE;
    };

    // ==========================================
    // 2. NGƯỠNG KÍCH HOẠT PID
    // ==========================================

    // Sai số >1.2 độ: bật correction.
    static constexpr float HEADING_TRIGGER_RAD =
        0.0209439510239f;

    // Sai số <1.0 độ: tắt correction.
    static constexpr float HEADING_RELEASE_RAD =
        0.0174532925199f;

    // ==========================================
    // 3. GIỚI HẠN ĐIỀU KHIỂN
    // ==========================================

    static constexpr float INTEGRAL_TERM_LIMIT_RADPS =
        0.045f;

    static constexpr float KP_CROSSTRACK =
        0.40f;

    static constexpr float CROSSTRACK_LIMIT_RADPS =
        0.030f;

    static constexpr float CORRECTION_LIMIT_RADPS =
        0.140f;

    static constexpr float CORRECTION_SLEW_RADPS2 =
        0.70f;

    // Không tích phân qua khoảng trống IMU >120ms.
    static constexpr float MAX_IMU_SAMPLE_DT_S =
        0.12f;

    // Ngưỡng nhận biết lệnh dừng.
    static constexpr float COMMAND_STOP_EPS =
        0.001f;

    Gains gains_{};

    // ==========================================
    // 4. THAY ĐỔI PID
    // ==========================================

    void setGains(Gains g) {

        if (!isfinite(g.kp) ||
            !isfinite(g.ki) ||
            !isfinite(g.kd) ||
            g.kp < 0 ||
            g.ki < 0 ||
            g.kd < 0) {
            return;
        }

        gains_ = g;

        integral_ = 0;
        last_corr_ = 0;
        correction_active_ = false;
    }

    Gains gains() const {
        return gains_;
    }

    void reset() {
        *this = Controller{};
    }

    void inactive() {
        motion_active_ = false;
        last_corr_ = 0;
        integral_ = 0;
        correction_active_ = false;
        mode_ = 0;
    }

    void lostSample() {

        reacquire_ = true;
        inactive();

        // Ngăn tích phân qua mẫu IMU bị mất.
        observed_ = false;
        last_stamp_us_ = 0;
        last_seq_ = 0;
        gyro_ = 0;

        // Không tự đặt yaw về 0.
    }

    float yawRadians() const {
        return yaw_;
    }

    float targetRadians() const {
        return heading_ref_;
    }

    float gyroRadps() const {
        return gyro_;
    }

    int mode() const {
        return mode_;
    }

    bool observed() const {
        return observed_;
    }

    // ==========================================
    // 5. ĐỌC IMU VÀ TÍCH PHÂN YAW
    // ==========================================

    bool observe(
        float gz,
        uint64_t stamp_us,
        uint32_t seq,
        bool valid
    ) {

        if (!valid ||
            !isfinite(gz) ||
            stamp_us == 0) {

            lostSample();
            return false;
        }

        if (observed_) {

            // Mẫu cũ lặp lại: không tích phân lần hai.
            if (seq == last_seq_ &&
                stamp_us == last_stamp_us_) {
                return true;
            }

            // Hỗ trợ sequence uint32 quay vòng.
            const uint32_t seq_step =
                seq - last_seq_;

            // Dữ liệu sai thứ tự hoặc timestamp lỗi.
            if (stamp_us <= last_stamp_us_ ||
                seq_step == 0 ||
                seq_step >= 0x80000000U) {

                lostSample();

                // Đồng bộ lại từ mẫu hiện tại,
                // không cộng thêm góc yaw.
                gyro_ = gz;
                last_stamp_us_ = stamp_us;
                last_seq_ = seq;
                observed_ = true;

                return false;
            }

            const float sample_dt =
                float(stamp_us - last_stamp_us_) *
                1e-6f;

            if (sample_dt > 0 &&
                sample_dt <= MAX_IMU_SAMPLE_DT_S) {

                // Tích phân hình thang.
                yaw_ += 0.5f *
                    (gyro_ + gz) *
                    sample_dt;

            } else {

                // Mất mẫu quá lâu:
                // bỏ qua góc quay trong khoảng mất mẫu.
                reacquire_ = true;
                inactive();
            }
        }

        gyro_ = gz;
        last_stamp_us_ = stamp_us;
        last_seq_ = seq;
        observed_ = true;

        return true;
    }

    // ==========================================
    // 6. ƯỚC LƯỢNG QUÃNG ĐƯỜNG
    // ==========================================

    void observeTravel(float ds_m) {

        if (!motion_active_ ||
            !observed_ ||
            !isfinite(ds_m) ||
            fabsf(ds_m) > 0.10f) {
            return;
        }

        const float mid =
            0.5f * (last_est_yaw_ + yaw_);

        x_est_ += ds_m * cosf(mid);
        y_est_ += ds_m * sinf(mid);

        last_est_yaw_ = yaw_;
    }

    // ==========================================
    // 7. PID YAW VÒNG NGOÀI
    // ==========================================

    Result update(
        float v_cmd,
        float w_cmd,
        float dt,
        bool imu_ready,
        bool local_crosstrack_enabled = false
    ) {

        Result out;

        out.requested_w =
            isfinite(w_cmd) ? w_cmd : 0;

        out.omega_out = out.requested_w;

        out.gyro_z = gyro_;
        out.yaw_rad = yaw_;
        out.heading_ref_rad = heading_ref_;

        // ======================================
        // A. CHẶN CORRECTION KHI IDLE
        // ======================================

        if (isfinite(v_cmd) &&
            isfinite(w_cmd) &&
            fabsf(v_cmd) < COMMAND_STOP_EPS &&
            fabsf(w_cmd) < COMMAND_STOP_EPS) {

            inactive();

            // Chốt hướng mới khi đứng yên.
            heading_ref_ = yaw_;

            out.omega_out = 0;
            out.requested_w = 0;
            out.heading_ref_rad = heading_ref_;
            out.mode = 0;

            return out;
        }

        // ======================================
        // B. KIỂM TRA IMU VÀ ĐIỀU KIỆN
        // ======================================

        if (!imu_ready ||
            !observed_ ||
            !isfinite(v_cmd) ||
            !isfinite(w_cmd) ||
            !isfinite(dt) ||
            dt <= 0 ||
            dt > 0.08f) {

            reacquire_ = true;
            inactive();

            out.mode = 3;

            // Giữ lệnh /cmd_vel hợp lệ cho cơ chế
            // fallback của robot_drive.cpp.
            return out;
        }

        // ======================================
        // C. CHỐT TARGET KHI BẮT ĐẦU CHẠY
        // ======================================

        if (!motion_active_ || reacquire_) {

            heading_ref_ = yaw_;

            last_corr_ = 0;
            integral_ = 0;
            correction_active_ = false;

            x_ref_ = 0;
            y_ref_ = 0;
            x_est_ = 0;
            y_est_ = 0;

            last_est_yaw_ = yaw_;

            motion_active_ = true;
            reacquire_ = false;
        }

        const bool straight =
            fabsf(w_cmd) < 0.001f &&
            fabsf(v_cmd) > 0.001f;

        // ======================================
        // D. CẬP NHẬT HEADING TARGET
        // ======================================

        const float new_heading =
            heading_ref_ + w_cmd * dt;

        if (fabsf(w_cmd) > 1.0e-5f) {

            x_ref_ +=
                v_cmd / w_cmd *
                (sinf(new_heading) -
                 sinf(heading_ref_));

            y_ref_ -=
                v_cmd / w_cmd *
                (cosf(new_heading) -
                 cosf(heading_ref_));

        } else {

            x_ref_ +=
                v_cmd * dt * cosf(heading_ref_);

            y_ref_ +=
                v_cmd * dt * sinf(heading_ref_);
        }

        heading_ref_ = new_heading;

        mode_ = straight ? 1 : 2;

        // ======================================
        // E. TÍNH SAI SỐ YAW
        // ======================================

        const float err =
            wrap(heading_ref_ - yaw_);

        const float rate_err =
            w_cmd - gyro_;

        // ======================================
        // F. KÍCH HOẠT PID VỚI HYSTERESIS
        // ======================================

        if (!correction_active_) {

            if (fabsf(err) >
                HEADING_TRIGGER_RAD) {

                correction_active_ = true;
            }

        } else if (fabsf(err) <
                   HEADING_RELEASE_RAD) {

            correction_active_ = false;
        }

        const bool active =
            correction_active_;

        // ======================================
        // G. TÍNH SAI LỆCH QUỸ ĐẠO CỤC BỘ
        // ======================================

        const float dx =
            x_est_ - x_ref_;

        const float dy =
            y_est_ - y_ref_;

        const float cte =
            -sinf(heading_ref_) * dx +
             cosf(heading_ref_) * dy;

        const float along =
             cosf(heading_ref_) * dx +
             sinf(heading_ref_) * dy;

        float path_corr = 0;

        // Mặc định OFF, không thay Nav2.
        if (active &&
            local_crosstrack_enabled &&
            fabsf(v_cmd) > 0.035f) {

            path_corr = clamp(
                -KP_CROSSTRACK *
                cte *
                copysignf(1.0f, v_cmd),

                -CROSSTRACK_LIMIT_RADPS,
                 CROSSTRACK_LIMIT_RADPS
            );
        }

        // ======================================
        // H. TÍNH PID
        // ======================================

        float p = 0;
        float i = 0;
        float d = 0;
        float corr = 0;
        float unclipped = 0;

        if (active) {

            // P: theo sai số góc.
            p = gains_.kp * err;

            // D: phản hồi vận tốc gyro.
            d = gains_.kd * rate_err;

            // ==================================
            // I: CHỐNG TÍCH LŨY QUÁ MỨC
            // ==================================

            if (gains_.ki > 0) {

                const float integral_bound =
                    INTEGRAL_TERM_LIMIT_RADPS /
                    gains_.ki;

                const float candidate = clamp(
                    integral_ + err * dt,
                    -integral_bound,
                     integral_bound
                );

                const float trial =
                    p +
                    gains_.ki * candidate +
                    d +
                    path_corr;

                const float trial_bounded = clamp(
                    trial,
                    -CORRECTION_LIMIT_RADPS,
                     CORRECTION_LIMIT_RADPS
                );

                const float max_step =
                    CORRECTION_SLEW_RADPS2 * dt;

                // Chặn tích phân khi đang
                // đẩy correction bão hòa thêm.
                const bool pushing_amplitude =
                    (trial > CORRECTION_LIMIT_RADPS &&
                     err > 0) ||

                    (trial < -CORRECTION_LIMIT_RADPS &&
                     err < 0);

                // Chặn tích phân khi đầu ra
                // còn đang bị giới hạn slew.
                const bool pushing_slew =
                    (trial_bounded >
                     last_corr_ + max_step &&
                     err > 0) ||

                    (trial_bounded <
                     last_corr_ - max_step &&
                     err < 0);

                if (!pushing_amplitude &&
                    !pushing_slew) {

                    integral_ = candidate;
                }

            } else {

                integral_ = 0;
            }

            i = gains_.ki * integral_;

            // ==================================
            // TỔNG CORRECTION
            // ==================================

            unclipped =
                p + i + d + path_corr;

            const float bounded = clamp(
                unclipped,
                -CORRECTION_LIMIT_RADPS,
                 CORRECTION_LIMIT_RADPS
            );

            // Giới hạn tốc độ thay đổi correction.
            corr = clamp(
                bounded,

                last_corr_ -
                CORRECTION_SLEW_RADPS2 * dt,

                last_corr_ +
                CORRECTION_SLEW_RADPS2 * dt
            );

            out.slew_limited =
                fabsf(corr - bounded) > 1e-7f;

            last_corr_ = corr;

        } else {

            // Khi sai số <1 độ:
            // tắt correction ngay.
            integral_ = 0;
            last_corr_ = 0;
        }

        // ======================================
        // I. XUẤT KẾT QUẢ VÀ DEBUG
        // ======================================

        out.active = active;

        out.saturated =
            active &&
            fabsf(unclipped) >
            CORRECTION_LIMIT_RADPS;

        out.p_term = p;
        out.i_term = i;
        out.d_term = d;

        out.omega_out = w_cmd + corr;
        out.correction = corr;

        out.heading_err = err;
        out.heading_ref_rad = heading_ref_;

        out.yaw_rad = yaw_;
        out.gyro_z = gyro_;

        out.heading_corr = p;
        out.rate_corr = d;
        out.path_corr = path_corr;

        out.x_ref_m = x_ref_;
        out.y_ref_m = y_ref_;

        out.x_est_m = x_est_;
        out.y_est_m = y_est_;

        out.cross_track_m = cte;
        out.along_track_m = along;

        out.curvature_inv_m =
            fabsf(v_cmd) > 0.015f ?
            w_cmd / v_cmd : 0;

        out.radius_m =
            fabsf(w_cmd) > 0.018f ?
            v_cmd / w_cmd : 0;

        out.mode = mode_;

        return out;
    }
};

} // namespace YawOuter
