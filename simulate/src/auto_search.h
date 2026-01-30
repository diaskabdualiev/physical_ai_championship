#pragma once

#include <opencv2/opencv.hpp>
#include <mujoco/mujoco.h>
#include <random>
#include <atomic>
#include <iostream>

enum class SearchState { SEARCHING, APPROACHING };

enum class BootPhase { IDLE, LOWERING, FIXSTAND_WARMUP, FIXSTAND_PRESS, FIXSTAND_HOLD, VELOCITY_PRESS, VELOCITY_HOLD, BAND_OFF_WAIT, READY };

class AutoSearch
{
public:
    AutoSearch() : rng_(std::random_device{}()) {}

    // Process an RGB image and return virtual joystick commands
    // Returns: {ly, rx} where ly=forward speed, rx=yaw rate
    void process(const unsigned char* rgb, const float* depth_buf, int width, int height,
                 float znear, float zfar, bool show_window)
    {
        cv::Mat img(height, width, CV_8UC3, const_cast<unsigned char*>(rgb));
        cv::Mat bgr;
        cv::cvtColor(img, bgr, cv::COLOR_RGB2BGR);
        cv::flip(bgr, bgr, 0); // MuJoCo renders upside down

        cv::Mat hsv;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

        // Red color detection in HSV (two ranges for red hue wrap)
        cv::Mat mask1, mask2, mask;
        cv::inRange(hsv, cv::Scalar(0, 80, 80), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(170, 80, 80), cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;

        // Morphological cleanup
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double max_area = 0;
        int best_idx = -1;
        for (int i = 0; i < (int)contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                best_idx = i;
            }
        }

        float center_x = width / 2.0f;
        const double min_detect_area = 100.0;

        if (best_idx >= 0 && max_area > min_detect_area) {
            cv::Rect bbox = cv::boundingRect(contours[best_idx]);
            float obj_cx = bbox.x + bbox.width / 2.0f;
            float offset = (obj_cx - center_x) / center_x; // -1..1

            const float center_threshold = 0.15f; // cube is centered when |offset| < this

            // Get depth at cube's bounding box center (flipped Y because MuJoCo renders upside down)
            int cube_px = bbox.x + bbox.width / 2;
            int cube_py = bbox.y + bbox.height / 2;
            // bbox is in flipped image coords, convert back to raw depth buffer coords
            int depth_x = cube_px;
            int depth_y = (height - 1) - cube_py;
            int depth_idx = depth_y * width + depth_x;
            float raw_depth = depth_buf[depth_idx];
            // Convert to linear distance in meters (along optical axis)
            float cube_depth = znear / (1.0f - raw_depth * (1.0f - znear / zfar));

            // Convert depth-buffer distance to horizontal ground distance
            // Camera xyaxes="0 -1 0 1.5 0 1" → optical axis = -Z_cam = (1, 0, -1.5) in body frame
            // Pitch from horizontal: atan2(1.5, 1) ≈ 56.3° downward
            // For pixel (cube_px, cube_py) compute ray direction in body frame
            constexpr float fovy_rad = 75.0f * M_PI / 180.0f;
            float fy = height / (2.0f * std::tan(fovy_rad / 2.0f));
            float fx = fy; // square pixels

            // Pixel offset from image center (in flipped image)
            float px_offset = (cube_px - width / 2.0f) / fx;
            float py_offset = -(cube_py - height / 2.0f) / fy; // negative: image Y is flipped vs camera Y

            // Ray in camera frame: (px_offset, py_offset, -1) — camera looks along -Z
            // Transform to body frame using camera rotation matrix:
            // cam_X = (0, -1, 0), cam_Y = (1.5, 0, 1)/norm, cam_Z = (-1, 0, 1.5)/norm
            // where norm = sqrt(1 + 1.5^2) = sqrt(3.25)
            constexpr float inv_norm = 1.0f / 1.80277564f; // 1/sqrt(3.25)
            float ray_bx = /* cam_Yx * py_offset + cam_Zx * (-1) */
                           (1.5f * inv_norm) * py_offset - (-inv_norm) * 1.0f;  // wait let me redo this

            // Camera axes in body frame (normalized):
            // X_b = (0, -1, 0)
            // Y_b = (1.5, 0, 1) / sqrt(3.25)
            // Z_b = X_b × Y_b_raw = (-1, 0, 1.5) / sqrt(3.25)
            // Ray_body = X_b * px_offset + Y_b * py_offset + Z_b * (-1)
            //          = (0, -px_offset, 0) + (1.5*py_offset, 0, py_offset)/norm + (1, 0, -1.5)/norm
            float rbx = (1.5f * py_offset + 1.0f) * inv_norm;
            float rby = -px_offset;
            float rbz = (1.0f * py_offset - 1.5f) * inv_norm;

            // depth buffer gives distance along optical axis (-Z_cam)
            // 3D point in body frame = ray_body * cube_depth (since ray_cam.z = -1, depth scales linearly)
            float pt_x = rbx * cube_depth;
            float pt_y = rby * cube_depth;
            // Horizontal distance from robot body to cube
            float horizontal_dist = std::sqrt(pt_x * pt_x + pt_y * pt_y);

            // Stop if close enough (horizontal ground distance ≤ 0.5m)
            if (horizontal_dist <= 0.5f && cube_depth > 0.01f) {
                state_ = SearchState::APPROACHING;
                rx_ = 0.0f;
                ly_ = 0.0f;
                stopped_ = true;
                std::cout << "[AutoSearch] STOPPED — horizontal: " << horizontal_dist
                          << "m, depth: " << cube_depth << "m" << std::endl;
            } else if (std::fabs(offset) > center_threshold) {
                // Centering: rotate toward cube, don't move forward
                state_ = SearchState::SEARCHING;
                rx_ = (offset > 0) ? 1.0f : -1.0f;
                ly_ = 0.0f;
                stopped_ = false;
            } else {
                // Centered: go forward at max speed
                state_ = SearchState::APPROACHING;
                rx_ = 0.0f;
                ly_ = 0.7f;
                stopped_ = false;
            }

            if (show_window) {
                cv::rectangle(bgr, bbox, cv::Scalar(0, 255, 0), 2);
                std::string label = stopped_ ? "STOPPED" :
                    (std::fabs(offset) > center_threshold ? "CENTERING" : "APPROACHING");
                cv::Scalar color = stopped_ ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 0);
                cv::putText(bgr, label, cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
                cv::putText(bgr, "depth: " + std::to_string(cube_depth).substr(0,4) + "m",
                           cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);
                cv::putText(bgr, "horiz: " + std::to_string(horizontal_dist).substr(0,4) + "m",
                           cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,0), 1);
            }
        } else {
            state_ = SearchState::SEARCHING;
            rx_ = 1.0f;  // Rotate right full stick
            ly_ = 0.0f;

            if (show_window) {
                cv::putText(bgr, "SEARCHING", cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }
        }

        if (show_window) {
            cv::imshow("Head Camera", bgr);
            cv::waitKey(1);
        }
    }

    float getLy() const { return ly_; }
    float getRx() const { return rx_; }
    SearchState getState() const { return state_; }
    bool isStopped() const { return stopped_; }

    // Respawn cube to random position within room bounds, avoiding robot
    void respawnCube(mjModel* m, mjData* d, int cube_body_id)
    {
        int cube_jnt_id = m->body_jntadr[cube_body_id];
        if (cube_jnt_id < 0) return;
        int cube_qadr = m->jnt_qposadr[cube_jnt_id];

        double robot_x = d->qpos[0];
        double robot_y = d->qpos[1];

        std::uniform_real_distribution<double> dist_xy(-1.5, 1.5);

        double cx, cy;
        int attempts = 0;
        do {
            cx = dist_xy(rng_);
            cy = dist_xy(rng_);
            attempts++;
        } while (std::sqrt((cx - robot_x) * (cx - robot_x) +
                           (cy - robot_y) * (cy - robot_y)) < 0.8 && attempts < 50);

        // Set cube position (free joint: 3 pos + 4 quat)
        d->qpos[cube_qadr + 0] = cx;
        d->qpos[cube_qadr + 1] = cy;
        d->qpos[cube_qadr + 2] = 0.15; // on the floor
        d->qpos[cube_qadr + 3] = 1.0;  // quat w
        d->qpos[cube_qadr + 4] = 0.0;
        d->qpos[cube_qadr + 5] = 0.0;
        d->qpos[cube_qadr + 6] = 0.0;

        // Zero velocities
        int cube_dofadr = m->jnt_dofadr[cube_jnt_id];
        for (int i = 0; i < 6; i++) {
            d->qvel[cube_dofadr + i] = 0.0;
        }

        std::cout << "[AutoSearch] Cube respawned to (" << cx << ", " << cy << ")" << std::endl;
    }

    std::atomic<bool> enabled{false};

    // Boot sequence state
    BootPhase boot_phase_ = BootPhase::IDLE;

    struct ButtonCmd {
        bool lt = false;
        bool up = false;
        bool rb = false;
        bool x = false;
    };

    // Called from bridge at 1kHz with simulation time
    // Returns button overrides for the boot sequence
    ButtonCmd updateBoot(double sim_time)
    {
        ButtonCmd cmd;

        switch (boot_phase_) {
        case BootPhase::IDLE:
            // 1. Start lowering elastic band at 4s
            if (sim_time >= 4.0) {
                boot_phase_ = BootPhase::LOWERING;
                std::cout << "[AutoLaunch] Lowering elastic band..." << std::endl;
            }
            break;

        case BootPhase::LOWERING:
            // Elastic band lowering handled in main.cc physics loop
            // Transition to FIXSTAND_PRESS set externally when length reached
            break;

        case BootPhase::FIXSTAND_WARMUP:
            // 2a. Send LT only for 100ms to let Axis smoothing accumulate past 0.5 threshold
            // Axis smooth=0.03: need ~23 frames at 1kHz = 23ms, using 100ms for safety
            cmd.lt = true;
            cmd.up = false;
            if (fixstand_warmup_start_ < 0) fixstand_warmup_start_ = sim_time;
            if (sim_time - fixstand_warmup_start_ >= 0.1) {
                boot_phase_ = BootPhase::FIXSTAND_PRESS;
            }
            break;

        case BootPhase::FIXSTAND_PRESS:
            // 2b. Now add Up while LT is warm → on_pressed triggers
            cmd.lt = true;
            cmd.up = true;
            if (fixstand_press_start_ < 0) fixstand_press_start_ = sim_time;
            if (sim_time - fixstand_press_start_ >= 0.5) {
                boot_phase_ = BootPhase::FIXSTAND_HOLD;
            }
            break;

        case BootPhase::FIXSTAND_HOLD:
            // Wait 2s for robot to stand
            if (fixstand_hold_start_ < 0) fixstand_hold_start_ = sim_time;
            if (sim_time - fixstand_hold_start_ >= 2.0) {
                boot_phase_ = BootPhase::VELOCITY_PRESS;
                std::cout << "[AutoLaunch] Sending Velocity mode (RB+X)..." << std::endl;
            }
            break;

        case BootPhase::VELOCITY_PRESS:
            // 3. Send Velocity: RB first for 50ms, then add X for on_pressed edge
            cmd.rb = true;
            if (velocity_press_start_ < 0) velocity_press_start_ = sim_time;
            if (sim_time - velocity_press_start_ >= 0.05) {
                cmd.x = true; // X.on_pressed triggers after RB is already held
            }
            if (sim_time - velocity_press_start_ >= 0.5) {
                boot_phase_ = BootPhase::VELOCITY_HOLD;
            }
            break;

        case BootPhase::VELOCITY_HOLD:
            // 4. Wait 3s for stabilization
            if (velocity_hold_start_ < 0) velocity_hold_start_ = sim_time;
            if (sim_time - velocity_hold_start_ >= 3.0) {
                boot_phase_ = BootPhase::BAND_OFF_WAIT;
                std::cout << "[AutoLaunch] Disabling elastic band..." << std::endl;
            }
            break;

        case BootPhase::BAND_OFF_WAIT:
            // 5. Elastic band disabled externally, wait 1s then AutoSearch
            if (band_off_start_ < 0) band_off_start_ = sim_time;
            if (sim_time - band_off_start_ >= 1.0) {
                enabled.store(true);
                boot_phase_ = BootPhase::READY;
                std::cout << "[AutoLaunch] AutoSearch ENABLED" << std::endl;
            }
            break;

        case BootPhase::READY:
            break;
        }

        return cmd;
    }

    bool isBooting() const { return boot_phase_ != BootPhase::IDLE && boot_phase_ != BootPhase::READY; }
    bool isBootReady() const { return boot_phase_ == BootPhase::READY; }

private:
    double fixstand_warmup_start_ = -1;
    double fixstand_press_start_ = -1;
    double fixstand_hold_start_ = -1;
    double velocity_press_start_ = -1;
    double velocity_hold_start_ = -1;
    double band_off_start_ = -1;
    SearchState state_ = SearchState::SEARCHING;
    float ly_ = 0.0f;
    float rx_ = 0.0f;
    bool stopped_ = false;
    std::mt19937 rng_;
};
