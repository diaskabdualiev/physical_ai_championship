#pragma once

#include <opencv2/opencv.hpp>
#include <mujoco/mujoco.h>
#include <random>
#include <atomic>
#include <iostream>

enum class SearchState { SEARCHING, APPROACHING };

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
            // Convert to linear distance in meters
            float cube_depth = znear / (1.0f - raw_depth * (1.0f - znear / zfar));

            // Stop if close enough (depth-based, acts as LiDAR)
            if (cube_depth <= 0.9f && cube_depth > 0.01f) {
                state_ = SearchState::APPROACHING;
                rx_ = 0.0f;
                ly_ = 0.0f;
                stopped_ = true;
                std::cout << "[AutoSearch] STOPPED at distance: " << cube_depth << "m" << std::endl;
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

private:
    SearchState state_ = SearchState::SEARCHING;
    float ly_ = 0.0f;
    float rx_ = 0.0f;
    bool stopped_ = false;
    std::mt19937 rng_;
};
