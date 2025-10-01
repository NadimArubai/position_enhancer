#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "position_enhancer_interfaces/srv/enhance_position.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <memory>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;
using gtsam::symbol_shorthand::X; // for robot poses
using gtsam::symbol_shorthand::L; // for landmarks/objects

class PositionSolverNode : public rclcpp::Node {
public:
    PositionSolverNode() : Node("position_solver") {
        service_ = this->create_service<position_enhancer_interfaces::srv::EnhancePosition>(
            "enhance_position",
            std::bind(&PositionSolverNode::handle_service, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Parameters for noise models
        this->declare_parameter("default_range_noise", 0.1);
        this->declare_parameter("default_bearing_noise", 0.05);
        this->declare_parameter("optimization_iterations", 100);
        this->declare_parameter("optimization_tolerance", 1e-5);
        
        default_range_noise_ = this->get_parameter("default_range_noise").as_double();
        default_bearing_noise_ = this->get_parameter("default_bearing_noise").as_double();
        optimization_iterations_ = this->get_parameter("optimization_iterations").as_int();
        optimization_tolerance_ = this->get_parameter("optimization_tolerance").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Position Solver service started (stateless mode)");
    }

private:
    rclcpp::Service<position_enhancer_interfaces::srv::EnhancePosition>::SharedPtr service_;
    
    double default_range_noise_;
    double default_bearing_noise_;
    int optimization_iterations_;
    double optimization_tolerance_;
    
    void handle_service(
        const std::shared_ptr<position_enhancer_interfaces::srv::EnhancePosition::Request> request,
        std::shared_ptr<position_enhancer_interfaces::srv::EnhancePosition::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Called");
        try {
            // Validate input
            if (!validate_request(request, response)) {
                return;
            }
            
            // Create and solve the optimization problem
            auto result = solve_position(request);
            
            // Extract and return the solution
            extract_solution(result, response);
            
            response->success = true;
            response->message = "Position enhanced successfully";
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service error: %s", e.what());
            response->success = false;
            response->message = std::string("Error: ") + e.what();
        }
    }
    
    bool validate_request(
        const std::shared_ptr<position_enhancer_interfaces::srv::EnhancePosition::Request> request,
        std::shared_ptr<position_enhancer_interfaces::srv::EnhancePosition::Response> response) {
        
        size_t n_observations = request->robot_poses.size();
        
        if (n_observations < 1) {
            response->success = false;
            response->message = "No observations provided";
            return false;
        }
        
        if (request->ranges.size() != n_observations ||
            request->bearings.size() != n_observations ||
            request->range_uncertainties.size() != n_observations ||
            request->bearing_uncertainties.size() != n_observations) {
            
            response->success = false;
            response->message = "Mismatched array sizes in request";
            return false;
        }
        
        if (n_observations < 2) {
            RCLCPP_WARN(this->get_logger(), "Only one observation provided - triangulation not possible");
        }
        
        return true;
    }
    
    gtsam::Pose2 ros_pose_to_gtsam(const geometry_msgs::msg::Pose& ros_pose) {
        // Extract yaw from quaternion
        tf2::Quaternion q(
            ros_pose.orientation.x,
            ros_pose.orientation.y,
            ros_pose.orientation.z,
            ros_pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        return gtsam::Pose2(ros_pose.position.x, ros_pose.position.y, yaw);
    }
    
    gtsam::Point2 calculate_initial_estimate(
        const std::vector<gtsam::Pose2>& robot_poses,
        const std::vector<double>& ranges,
        const std::vector<double>& bearings) {
        
        // Simple triangulation: average of all observations converted to global frame
        gtsam::Point2 sum_position(0, 0);
        int valid_count = 0;
        
        for (size_t i = 0; i < robot_poses.size(); ++i) {
            try {
                gtsam::Point2 local_object(
                    ranges[i] * cos(bearings[i]),
                    ranges[i] * sin(bearings[i])
                );
                gtsam::Point2 global_object = robot_poses[i].transformFrom(local_object);
                sum_position += global_object;
                valid_count++;
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to calculate initial estimate for observation %zu", i);
            }
        }
        
        if (valid_count == 0) {
            throw std::runtime_error("No valid observations for initial estimate");
        }
        
        return sum_position / valid_count;
    }
    
    std::pair<gtsam::Values, gtsam::NonlinearFactorGraph> solve_position(
        const std::shared_ptr<position_enhancer_interfaces::srv::EnhancePosition::Request> request) {
        
        size_t n_observations = request->robot_poses.size();
        
        // Convert ROS poses to GTSAM poses
        std::vector<gtsam::Pose2> robot_poses;
        for (const auto& ros_pose : request->robot_poses) {
            robot_poses.push_back(ros_pose_to_gtsam(ros_pose));
        }
        
        // Calculate initial estimate
        gtsam::Point2 initial_object_pos = calculate_initial_estimate(
            robot_poses, request->ranges, request->bearings);
        
        // Create factor graph and values
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial_estimate;
        
        // Add object initial estimate
        initial_estimate.insert(L(0), initial_object_pos);
        
        // Add factors for each observation
        for (size_t i = 0; i < n_observations; ++i) {
            double range_unc = request->range_uncertainties[i] > 0 ? 
                request->range_uncertainties[i] : default_range_noise_;
            double bearing_unc = request->bearing_uncertainties[i] > 0 ? 
                request->bearing_uncertainties[i] : default_bearing_noise_;
            
            auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(2) << bearing_unc, range_unc).finished());
            
            gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2> factor(
                X(i), L(0), 
                gtsam::Rot2::fromAngle(request->bearings[i]), 
                request->ranges[i], 
                noise_model
            );
            
            graph.add(factor);
            
            // Add robot pose as fixed (known) value
            initial_estimate.insert(X(i), robot_poses[i]);
        }
        
        // Add weak prior on object position to help optimization
        auto weak_prior = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(2) << 10.0, 10.0).finished());
        graph.add(gtsam::PriorFactor<gtsam::Point2>(L(0), initial_object_pos, weak_prior));
        
        // Optimize
        gtsam::LevenbergMarquardtParams params;
        params.setMaxIterations(optimization_iterations_);
        params.setRelativeErrorTol(optimization_tolerance_);
        params.setAbsoluteErrorTol(optimization_tolerance_);
        
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
        gtsam::Values result = optimizer.optimize();
        
        return {result, graph};
    }
    
    void extract_solution(
        const std::pair<gtsam::Values, gtsam::NonlinearFactorGraph>& solution,
        std::shared_ptr<position_enhancer_interfaces::srv::EnhancePosition::Response> response) {
        
        const auto& [result, graph] = solution;
        
        try {
            // Extract optimized object position
            gtsam::Point2 object_pos = result.at<gtsam::Point2>(L(0));
            
            // Compute marginal covariance
            gtsam::Marginals marginals(graph, result);
            gtsam::Matrix covariance = marginals.marginalCovariance(L(0));
            
            // Fill response
            response->enhanced_pose.position.x = object_pos.x();
            response->enhanced_pose.position.y = object_pos.y();
            response->enhanced_pose.position.z = 0.0;
            
            // Set orientation to identity (for point objects)
            response->enhanced_pose.orientation.x = 0.0;
            response->enhanced_pose.orientation.y = 0.0;
            response->enhanced_pose.orientation.z = 0.0;
            response->enhanced_pose.orientation.w = 1.0;
            
            response->pose_uncertainty_x = std::sqrt(covariance(0, 0));
            response->pose_uncertainty_y = std::sqrt(covariance(1, 1));
            response->pose_uncertainty_theta = 0.0;
            
            RCLCPP_DEBUG(this->get_logger(), 
                       "Solved position: (%.3f, %.3f) ± (%.3f, %.3f)", 
                       object_pos.x(), object_pos.y(),
                       std::sqrt(covariance(0, 0)), std::sqrt(covariance(1, 1)));
            
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("Failed to extract solution: ") + e.what());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionSolverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
