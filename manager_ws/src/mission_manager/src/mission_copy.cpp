
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <vector>
#include <algorithm>

class MissionManager : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    struct Cell
    {
        int x;
        int y;
    };

    struct Candidate
    {
        int index;
        int bfs_distance;
        double x;
        double y;
    };

    struct FailedGoal
    {
        double x;
        double y;
    };

    MissionManager()
        : Node("mission_goal")
    {
        // Permissive BFS clearance. Nav2 must still use the real footprint.
        robot_radius_ = declare_parameter<double>("robot_radius", 0.15);

        minimum_goal_distance_ =
            declare_parameter<double>("minimum_goal_distance", 0.30);

        failed_goal_exclusion_radius_ =
            declare_parameter<double>("failed_goal_exclusion_radius", 0.30);

        occupied_threshold_ =
            declare_parameter<int>("occupied_threshold", 65);

        goal_tolerance_ =
            declare_parameter<double>("goal_tolerance", 0.50);

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            rclcpp::QoS(1).transient_local().reliable(),
            std::bind(
                &MissionManager::MapCallback,
                this,
                std::placeholders::_1
            )
        );

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(
                *tf_buffer_,
                this
            );

        action_client_ =
            rclcpp_action::create_client<NavigateToPose>(
                this,
                "navigate_to_pose"
            );

        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MissionManager::MainLoop, this)
        );

        RCLCPP_INFO(
            get_logger(),
            "BFS mission manager started. Clearance: %.2f m",
            robot_radius_
        );
    }

private:
    // ------------------------------------------------------------
    // ROS
    // ------------------------------------------------------------

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ------------------------------------------------------------
    // STATE
    // ------------------------------------------------------------

    nav_msgs::msg::OccupancyGrid map_;

    bool map_received_ = false;
    bool goal_active_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    double current_goal_x_ = 0.0;
    double current_goal_y_ = 0.0;

    double robot_radius_;
    double minimum_goal_distance_;
    double failed_goal_exclusion_radius_;
    double goal_tolerance_;

    int occupied_threshold_;

    std::vector<FailedGoal> failed_goals_;

    // Successfully visited goals are excluded to prevent
    // repeatedly navigating between the same locations.
    std::vector<FailedGoal> visited_goals_;

    // ------------------------------------------------------------
    // MAP
    // ------------------------------------------------------------

    void MapCallback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = *msg;
        map_received_ = true;
    }

    bool IsInsideMap(int x, int y) const
    {
        return (
            x >= 0 &&
            y >= 0 &&
            x < static_cast<int>(map_.info.width) &&
            y < static_cast<int>(map_.info.height)
        );
    }

    int CellIndex(int x, int y) const
    {
        return y * static_cast<int>(map_.info.width) + x;
    }

    double MapYaw() const
    {
        const auto &q = map_.info.origin.orientation;

        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        );
    }

    bool WorldToMap(
        double wx,
        double wy,
        int &mx,
        int &my) const
    {
        const double yaw = MapYaw();

        const double dx = wx - map_.info.origin.position.x;
        const double dy = wy - map_.info.origin.position.y;

        const double local_x =
            std::cos(yaw) * dx + std::sin(yaw) * dy;

        const double local_y =
            -std::sin(yaw) * dx + std::cos(yaw) * dy;

        mx = static_cast<int>(
            std::floor(local_x / map_.info.resolution)
        );

        my = static_cast<int>(
            std::floor(local_y / map_.info.resolution)
        );

        return IsInsideMap(mx, my);
    }

    void MapToWorld(
        int mx,
        int my,
        double &wx,
        double &wy) const
    {
        const double yaw = MapYaw();

        const double local_x =
            (mx + 0.5) * map_.info.resolution;

        const double local_y =
            (my + 0.5) * map_.info.resolution;

        wx =
            map_.info.origin.position.x +
            std::cos(yaw) * local_x -
            std::sin(yaw) * local_y;

        wy =
            map_.info.origin.position.y +
            std::sin(yaw) * local_x +
            std::cos(yaw) * local_y;
    }

    double Distance(
        double x1,
        double y1,
        double x2,
        double y2) const
    {
        return std::hypot(x1 - x2, y1 - y2);
    }

    // ------------------------------------------------------------
    // BUILD TRAVERSABILITY GRID
    // ------------------------------------------------------------
    //
    // 0 = blocked
    // 1 = traversable
    //
    // Unknown cells are blocked.
    //
    // Instead of checking the footprint repeatedly during BFS,
    // construct the traversability grid once.
    // ------------------------------------------------------------

    std::vector<uint8_t> BuildTraversabilityGrid() const
    {
        const int width = static_cast<int>(map_.info.width);
        const int height = static_cast<int>(map_.info.height);

        const int count = width * height;

        std::vector<uint8_t> traversable(count, 0);

        if (count == 0 || map_.info.resolution <= 0.0)
        {
            return traversable;
        }

        const int radius_cells = static_cast<int>(
            std::ceil(robot_radius_ / map_.info.resolution)
        );

        std::vector<Cell> footprint_offsets;

        for (int dy = -radius_cells; dy <= radius_cells; ++dy)
        {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx)
            {
                const double distance = std::hypot(
                    dx * map_.info.resolution,
                    dy * map_.info.resolution
                );

                if (distance <= robot_radius_ + 1e-9)
                {
                    footprint_offsets.push_back({dx, dy});
                }
            }
        }

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                bool valid = true;

                for (const auto &offset : footprint_offsets)
                {
                    const int nx = x + offset.x;
                    const int ny = y + offset.y;

                    if (!IsInsideMap(nx, ny))
                    {
                        valid = false;
                        break;
                    }

                    const int value =
                        map_.data[CellIndex(nx, ny)];

                    if (value < 0 || value >= occupied_threshold_)
                    {
                        valid = false;
                        break;
                    }
                }

                if (valid)
                {
                    traversable[CellIndex(x, y)] = 1;
                }
            }
        }

        return traversable;
    }

    // ------------------------------------------------------------
    // BFS
    // ------------------------------------------------------------
    //
    // BFS returns reachable cells with their BFS distances.
    //
    // 8-connected movement is used, but diagonal corner cutting
    // is forbidden.
    // ------------------------------------------------------------

    std::vector<Candidate> BFS(
        int start_x,
        int start_y,
        const std::vector<uint8_t> &traversable) const
    {
        std::vector<Candidate> candidates;

        if (!IsInsideMap(start_x, start_y))
        {
            return candidates;
        }

        const int start_index = CellIndex(start_x, start_y);

        if (!traversable[start_index])
        {
            RCLCPP_WARN(
                get_logger(),
                "Robot's current cell is blocked after footprint clearance."
            );

            return candidates;
        }

        const int count =
            static_cast<int>(map_.info.width * map_.info.height);

        std::vector<int> distances(count, -1);

        std::queue<Cell> queue;

        distances[start_index] = 0;
        queue.push({start_x, start_y});

        const int dx[8] = {
            1, -1, 0, 0,
            1, 1, -1, -1
        };

        const int dy[8] = {
            0, 0, 1, -1,
            1, -1, 1, -1
        };

        while (!queue.empty())
        {
            const Cell current = queue.front();
            queue.pop();

            const int current_index =
                CellIndex(current.x, current.y);

            double wx;
            double wy;

            MapToWorld(current.x, current.y, wx, wy);

            candidates.push_back({
                current_index,
                distances[current_index],
                wx,
                wy
            });

            for (int i = 0; i < 8; ++i)
            {
                const int nx = current.x + dx[i];
                const int ny = current.y + dy[i];

                if (!IsInsideMap(nx, ny))
                {
                    continue;
                }

                const int next_index = CellIndex(nx, ny);

                if (!traversable[next_index])
                {
                    continue;
                }

                if (distances[next_index] != -1)
                {
                    continue;
                }

                // Do not move diagonally through obstacle corners.
                if (dx[i] != 0 && dy[i] != 0)
                {
                    const int adjacent_x =
                        CellIndex(current.x + dx[i], current.y);

                    const int adjacent_y =
                        CellIndex(current.x, current.y + dy[i]);

                    if (
                        !traversable[adjacent_x] ||
                        !traversable[adjacent_y]
                    )
                    {
                        continue;
                    }
                }

                distances[next_index] =
                    distances[current_index] + 1;

                queue.push({nx, ny});
            }
        }

        return candidates;
    }

    // ------------------------------------------------------------
    // EXCLUDED GOALS
    // ------------------------------------------------------------

    bool IsNearGoalList(
        double x,
        double y,
        const std::vector<FailedGoal> &goals,
        double radius) const
    {
        for (const auto &goal : goals)
        {
            if (Distance(x, y, goal.x, goal.y) < radius)
            {
                return true;
            }
        }

        return false;
    }

    // ------------------------------------------------------------
    // SELECT FARTHEST REACHABLE GOAL
    // ------------------------------------------------------------

    bool SelectGoal(
        const std::vector<Candidate> &candidates,
        double &goal_x,
        double &goal_y) const
    {
        bool found = false;

        int best_bfs_distance = -1;

        for (const auto &candidate : candidates)
        {
            const double distance = Distance(
                robot_x_,
                robot_y_,
                candidate.x,
                candidate.y
            );

            if (distance < minimum_goal_distance_)
            {
                continue;
            }

            if (IsNearGoalList(
                    candidate.x,
                    candidate.y,
                    failed_goals_,
                    failed_goal_exclusion_radius_))
            {
                continue;
            }

            if (IsNearGoalList(
                    candidate.x,
                    candidate.y,
                    visited_goals_,
                    goal_tolerance_))
            {
                continue;
            }

            if (candidate.bfs_distance > best_bfs_distance)
            {
                best_bfs_distance = candidate.bfs_distance;

                goal_x = candidate.x;
                goal_y = candidate.y;

                found = true;
            }
        }

        return found;
    }

    // ------------------------------------------------------------
    // ROBOT POSE
    // ------------------------------------------------------------

    bool GetRobotPose()
    {
        try
        {
            const auto transform = tf_buffer_->lookupTransform(
                "map",
                "base_link",
                tf2::TimePointZero
            );

            robot_x_ = transform.transform.translation.x;
            robot_y_ = transform.transform.translation.y;

            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(
                get_logger(),
                "TF unavailable: %s",
                ex.what()
            );

            return false;
        }
    }

    // ------------------------------------------------------------
    // RECORD FAILED GOAL
    // ------------------------------------------------------------

    void RecordFailedGoal()
    {
        failed_goals_.push_back({
            current_goal_x_,
            current_goal_y_
        });

        RCLCPP_WARN(
            get_logger(),
            "Excluding failed goal (%.2f, %.2f)",
            current_goal_x_,
            current_goal_y_
        );
    }

    // ------------------------------------------------------------
    // NAV2 GOAL RESPONSE
    // ------------------------------------------------------------

    void GoalResponse(const GoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_WARN(
                get_logger(),
                "Nav2 rejected goal."
            );

            RecordFailedGoal();

            goal_active_ = false;
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Nav2 accepted goal."
        );
    }

    // ------------------------------------------------------------
    // NAV2 RESULT
    // ------------------------------------------------------------

    void ResultResponse(const GoalHandle::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(
                get_logger(),
                "Reached goal (%.2f, %.2f)",
                current_goal_x_,
                current_goal_y_
            );

            visited_goals_.push_back({
                current_goal_x_,
                current_goal_y_
            });
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "Navigation failed with result code %d",
                static_cast<int>(result.code)
            );

            RecordFailedGoal();
        }

        goal_active_ = false;
    }

    // ------------------------------------------------------------
    // SEND NAV2 GOAL
    // ------------------------------------------------------------

    void SendGoal(double x, double y)
    {
        if (!action_client_->action_server_is_ready())
        {
            RCLCPP_WARN(
                get_logger(),
                "Nav2 action server is not ready."
            );

            return;
        }

        NavigateToPose::Goal goal;

        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = get_clock()->now();

        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.position.z = 0.0;

        // Keep the rover's current orientation at the goal.
        // This avoids forcing it to rotate to yaw=0 every time.
        goal.pose.pose.orientation.w = 1.0;

        current_goal_x_ = x;
        current_goal_y_ = y;

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

        options.goal_response_callback =
            std::bind(
                &MissionManager::GoalResponse,
                this,
                std::placeholders::_1
            );

        options.result_callback =
            std::bind(
                &MissionManager::ResultResponse,
                this,
                std::placeholders::_1
            );

        goal_active_ = true;

        RCLCPP_INFO(
            get_logger(),
            "Sending BFS goal: x=%.2f y=%.2f",
            x,
            y
        );

        action_client_->async_send_goal(goal, options);
    }

    // ------------------------------------------------------------
    // MAIN LOOP
    // ------------------------------------------------------------

    void MainLoop()
    {
        if (!map_received_ || goal_active_)
        {
            return;
        }

        if (!action_client_->action_server_is_ready())
        {
            return;
        }

        if (!GetRobotPose())
        {
            return;
        }

        int robot_cell_x;
        int robot_cell_y;

        if (!WorldToMap(
                robot_x_,
                robot_y_,
                robot_cell_x,
                robot_cell_y))
        {
            RCLCPP_WARN(
                get_logger(),
                "Robot is outside the map."
            );

            return;
        }

        // Build footprint-aware traversability grid.
        const auto traversable = BuildTraversabilityGrid();

        // Run BFS.
        const auto candidates = BFS(
            robot_cell_x,
            robot_cell_y,
            traversable
        );

        RCLCPP_INFO(
            get_logger(),
            "BFS found %zu reachable cells.",
            candidates.size()
        );

        if (candidates.empty())
        {
            return;
        }

        double goal_x;
        double goal_y;

        if (!SelectGoal(
                candidates,
                goal_x,
                goal_y))
        {
            RCLCPP_WARN(
                get_logger(),
                "No suitable reachable goal remains."
            );

            return;
        }

        SendGoal(goal_x, goal_y);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MissionManager>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
