#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

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

    enum class RecoveryState
    {
        NORMAL,
        ROTATING,
        WAITING_FOR_MAP
    };

    using SteadyClock = std::chrono::steady_clock;

    MissionManager()
        : Node("mission_goal")
    {
        // --------------------------------------------------------
        // BFS / footprint parameters
        // --------------------------------------------------------

        robot_radius_ =
            declare_parameter<double>(
                "robot_radius",
                0.46824  //0.4245
            );

        minimum_goal_distance_ =
            declare_parameter<double>(
                "minimum_goal_distance",
                0.20
            );

        failed_goal_exclusion_radius_ =
            declare_parameter<double>(
                "failed_goal_exclusion_radius",
                0.30
            );

        /*
         * How far around a successfully visited goal future
         * goals are forbidden.
         */
        visited_goal_exclusion_radius_ =
            declare_parameter<double>(
                "visited_goal_exclusion_radius",
                0.5
            );

        /*
         * Kept separately from the visited-goal exclusion radius.
         */
        goal_tolerance_ =
            declare_parameter<double>(
                "goal_tolerance",
                0.50
            );

        occupied_threshold_ =
            declare_parameter<int>(
                "occupied_threshold",
                65
            );

        // --------------------------------------------------------
        // Unknown-area exploration parameters
        // --------------------------------------------------------

        unknown_goal_min_distance_ =
            declare_parameter<double>(
                "unknown_goal_min_distance",
                0.50
            );

        unknown_goal_max_distance_ =
            declare_parameter<double>(
                "unknown_goal_max_distance",
                8.0
            );

        unknown_search_step_ =
            declare_parameter<int>(
                "unknown_search_step",
                1
            );

        // --------------------------------------------------------
        // Controlled rotation parameters
        // --------------------------------------------------------

        recovery_rotation_degrees_ =
            declare_parameter<double>(
                "recovery_rotation_degrees",
                30.0
            );

        recovery_rotation_speed_ =
            declare_parameter<double>(
                "recovery_rotation_speed",
                0.15
            );

        recovery_wait_after_rotation_ =
            declare_parameter<double>(
                "recovery_wait_after_rotation",
                1.5
            );

        // --------------------------------------------------------
        // ROS interfaces
        // --------------------------------------------------------

        map_sub_ =
            create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map",
                rclcpp::QoS(1)
                    .transient_local()
                    .reliable(),
                std::bind(
                    &MissionManager::MapCallback,
                    this,
                    std::placeholders::_1
                )
            );

        cmd_vel_pub_ =
            create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10
            );

        exploration_active_pub_ =
            create_publisher<std_msgs::msg::Bool>(
                "/exploration_active",
                10
            );

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(
                get_clock()
            );

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

        cancel_nav_sub_ =
            create_subscription<std_msgs::msg::Bool>(
                "/cancel_nav",
                10,
                std::bind(
                    &MissionManager::CancelNavCallback,
                    this,
                    std::placeholders::_1
                )
            );

        executing_xyz_sub_ =
            create_subscription<std_msgs::msg::Bool>(
                "executing_xyz",
                10,
                std::bind(
                    &MissionManager::ExecutingPickCallback,
                    this,
                    std::placeholders::_1
                )
            );

        resume_exploration_sub_ = 
            create_subscription<std_msgs::msg::Bool>(
                "resume_exploration",
                10,
                std::bind(
                    &MissionManager::ResumeExplorationCallback,
                    this,
                    std::placeholders::_1
                )
            );

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(
                    &MissionManager::MainLoop,
                    this
                )
            );

        RCLCPP_INFO(
            get_logger(),
            "Mission manager started."
        );

        RCLCPP_INFO(
            get_logger(),
            "Robot radius: %.3f m",
            robot_radius_
        );

        RCLCPP_INFO(
            get_logger(),
            "Visited-goal exclusion radius: %.2f m",
            visited_goal_exclusion_radius_
        );

        RCLCPP_INFO(
            get_logger(),
            "Recovery rotation: %.1f degrees at %.2f rad/s",
            recovery_rotation_degrees_,
            recovery_rotation_speed_
        );

        RCLCPP_INFO(
            get_logger(),
            "Unknown-area goals may be located anywhere in the map."
        );
    }

private:

    // ============================================================
    // ROS
    // ============================================================

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
        map_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        exploration_active_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr
        action_client_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        cancel_nav_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        executing_xyz_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        resume_exploration_sub_;

    GoalHandle::SharedPtr goal_handle_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener>
        tf_listener_;

    // ============================================================
    // STATE
    // ============================================================

    nav_msgs::msg::OccupancyGrid map_;

    bool map_received_ = false;
    bool goal_active_ = false;
    bool navigation_interrupted_ = false;
    bool cancellation_requested_ = false;
    bool arm_executing_ = false;
    bool exploration_active_ = true;
    bool resume_exploration_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    double current_goal_x_ = 0.0;
    double current_goal_y_ = 0.0;

    double robot_radius_;

    double minimum_goal_distance_;

    double failed_goal_exclusion_radius_;

    double visited_goal_exclusion_radius_;

    double goal_tolerance_;

    int occupied_threshold_;

    /*
     * Goals that Nav2 failed to reach.
     */
    std::vector<FailedGoal> failed_goals_;

    /*
     * Goals that Nav2 successfully reached.
     *
     * These are permanently remembered for the lifetime
     * of this mission-manager process.
     */
    std::vector<FailedGoal> visited_goals_;

    // ============================================================
    // UNKNOWN EXPLORATION PARAMETERS
    // ============================================================

    double unknown_goal_min_distance_;

    double unknown_goal_max_distance_;

    int unknown_search_step_;

    // ============================================================
    // RECOVERY
    // ============================================================

    RecoveryState recovery_state_ =
        RecoveryState::NORMAL;

    double recovery_rotation_degrees_;

    double recovery_rotation_speed_;

    double recovery_wait_after_rotation_;

    SteadyClock::time_point
        recovery_rotation_start_;

    SteadyClock::time_point
        recovery_wait_start_;

    bool recovery_active_ = false;

    bool rotation_command_sent_ = false;

    // ============================================================
    // MAP
    // ============================================================

    void PublishExplorationActive(bool active)
    {
        bool changed = exploration_active_ != active;
        exploration_active_ = active;

        std_msgs::msg::Bool msg;
        msg.data = active;
        exploration_active_pub_->publish(msg);

        if (changed) {
            RCLCPP_INFO(
                get_logger(),
                "Exploration active: %s",
                active ? "true" : "false"
            );
        }
    }

    void CancelNavCallback(
        const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data) {
            return;
        }

        if (navigation_interrupted_) {
            return;
        }

        PublishExplorationActive(false);
        navigation_interrupted_ = true;
        cancellation_requested_ = true;

        RCLCPP_INFO(
            get_logger(),
            "Navigation interruption requested. Exploration will pause."
        );

        if (goal_handle_) {
            RCLCPP_INFO(
                get_logger(),
                "Cancelling the active Nav2 goal."
            );
            action_client_->async_cancel_goal(goal_handle_);
        } else if (!goal_active_) {
            RCLCPP_INFO(
                get_logger(),
                "No active Nav2 goal to cancel. Exploration is paused."
            );
        }
        // If the goal request is still awaiting acceptance, GoalResponse()
        // will cancel it as soon as its goal handle arrives.
    }

    void ExecutingPickCallback(
        const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool was_executing = arm_executing_;
        arm_executing_ = msg->data;

        if (navigation_interrupted_ &&
            was_executing &&
            !arm_executing_)
        {
            RCLCPP_INFO(
               get_logger(),
               "Arm operation finished. Resuming exploration."
            );

            navigation_interrupted_ = false;
            cancellation_requested_ = false;
            PublishExplorationActive(true);
        }
    }

    void ResumeExplorationCallback(
        const std_msgs::msg::Bool::SharedPtr msg)
    {
        resume_exploration_ = msg->data;
        if(resume_exploration_)
        {
            RCLCPP_INFO(
                get_logger(),
                "Been centered for too long. Resuming exploration."
            ); 
        
            navigation_interrupted_ = false;
            cancellation_requested_ = false;
            PublishExplorationActive(true); 
        }
    }      

    void MapCallback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = *msg;
        map_received_ = true;
    }

    bool IsInsideMap(
        int x,
        int y) const
    {
        return (
            x >= 0 &&
            y >= 0 &&
            x < static_cast<int>(
                map_.info.width
            ) &&
            y < static_cast<int>(
                map_.info.height
            )
        );
    }

    int CellIndex(
        int x,
        int y) const
    {
        return (
            y *
            static_cast<int>(
                map_.info.width
            ) +
            x
        );
    }

    double MapYaw() const
    {
        const auto &q =
            map_.info.origin.orientation;

        return std::atan2(
            2.0 *
            (
                q.w * q.z +
                q.x * q.y
            ),
            1.0 -
            2.0 *
            (
                q.y * q.y +
                q.z * q.z
            )
        );
    }

    bool WorldToMap(
        double wx,
        double wy,
        int &mx,
        int &my) const
    {
        if (map_.info.resolution <= 0.0)
        {
            return false;
        }

        const double yaw = MapYaw();

        const double dx =
            wx -
            map_.info.origin.position.x;

        const double dy =
            wy -
            map_.info.origin.position.y;

        const double local_x =
            std::cos(yaw) * dx +
            std::sin(yaw) * dy;

        const double local_y =
            -std::sin(yaw) * dx +
            std::cos(yaw) * dy;

        mx =
            static_cast<int>(
                std::floor(
                    local_x /
                    map_.info.resolution
                )
            );

        my =
            static_cast<int>(
                std::floor(
                    local_y /
                    map_.info.resolution
                )
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
            (mx + 0.5) *
            map_.info.resolution;

        const double local_y =
            (my + 0.5) *
            map_.info.resolution;

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
        return std::hypot(
            x1 - x2,
            y1 - y2
        );
    }

    // ============================================================
    // ROBOT POSE
    // ============================================================

    bool GetRobotPose()
    {
        try
        {
            const auto transform =
                tf_buffer_->lookupTransform(
                    "map",
                    "base_link",
                    tf2::TimePointZero
                );

            robot_x_ =
                transform.transform.translation.x;

            robot_y_ =
                transform.transform.translation.y;

            return true;
        }
        catch (
            const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "TF unavailable: %s",
                ex.what()
            );

            return false;
        }
    }

    // ============================================================
    // TRAVERSABILITY
    // ============================================================

    std::vector<uint8_t>
    BuildTraversabilityGrid() const
    {
        const int width =
            static_cast<int>(
                map_.info.width
            );

        const int height =
            static_cast<int>(
                map_.info.height
            );

        const int count =
            width * height;

        std::vector<uint8_t>
            traversable(
                count,
                0
            );

        if (
            count == 0 ||
            map_.info.resolution <= 0.0
        )
        {
            return traversable;
        }

        const int radius_cells =
            static_cast<int>(
                std::ceil(
                    robot_radius_ /
                    map_.info.resolution
                )
            );

        std::vector<Cell>
            footprint_offsets;

        for (
            int dy = -radius_cells;
            dy <= radius_cells;
            ++dy
        )
        {
            for (
                int dx = -radius_cells;
                dx <= radius_cells;
                ++dx
            )
            {
                const double distance =
                    std::hypot(
                        dx *
                            map_.info.resolution,
                        dy *
                            map_.info.resolution
                    );

                if (
                    distance <=
                    robot_radius_ +
                    1e-9
                )
                {
                    footprint_offsets.push_back(
                        {
                            dx,
                            dy
                        }
                    );
                }
            }
        }

        for (
            int y = 0;
            y < height;
            ++y
        )
        {
            for (
                int x = 0;
                x < width;
                ++x
            )
            {
                bool valid = true;

                for (
                    const auto &offset :
                    footprint_offsets
                )
                {
                    const int nx =
                        x + offset.x;

                    const int ny =
                        y + offset.y;

                    if (
                        !IsInsideMap(
                            nx,
                            ny
                        )
                    )
                    {
                        valid = false;
                        break;
                    }

                    const int value =
                        map_.data[
                            CellIndex(
                                nx,
                                ny
                            )
                        ];

                    if (
                        value < 0 ||
                        value >=
                            occupied_threshold_
                    )
                    {
                        valid = false;
                        break;
                    }
                }

                if (valid)
                {
                    traversable[
                        CellIndex(
                            x,
                            y
                        )
                    ] = 1;
                }
            }
        }

        return traversable;
    }

    // ============================================================
    // BFS
    // ============================================================

    std::vector<Candidate>
    BFS(
        int start_x,
        int start_y,
        const std::vector<uint8_t>
            &traversable) const
    {
        std::vector<Candidate>
            candidates;

        if (
            !IsInsideMap(
                start_x,
                start_y
            )
        )
        {
            return candidates;
        }

        const int start_index =
            CellIndex(
                start_x,
                start_y
            );

        /*
         * Always allow the robot's current cell as the BFS seed.
         */
        std::vector<uint8_t>
            usable =
                traversable;

        usable[start_index] = 1;

        const int count =
            static_cast<int>(
                map_.info.width *
                map_.info.height
            );

        std::vector<int>
            distances(
                count,
                -1
            );

        std::queue<Cell>
            queue;

        distances[start_index] = 0;

        queue.push(
            {
                start_x,
                start_y
            }
        );

        const int dx[8] =
        {
            1,
            -1,
            0,
            0,
            1,
            1,
            -1,
            -1
        };

        const int dy[8] =
        {
            0,
            0,
            1,
            -1,
            1,
            -1,
            1,
            -1
        };

        while (!queue.empty())
        {
            const Cell current =
                queue.front();

            queue.pop();

            const int current_index =
                CellIndex(
                    current.x,
                    current.y
                );

            double wx;
            double wy;

            MapToWorld(
                current.x,
                current.y,
                wx,
                wy
            );

            candidates.push_back(
                {
                    current_index,
                    distances[
                        current_index
                    ],
                    wx,
                    wy
                }
            );

            for (int i = 0; i < 8; ++i)
            {
                const int nx =
                    current.x + dx[i];

                const int ny =
                    current.y + dy[i];

                if (
                    !IsInsideMap(
                        nx,
                        ny
                    )
                )
                {
                    continue;
                }

                const int next_index =
                    CellIndex(
                        nx,
                        ny
                    );

                if (!usable[next_index])
                {
                    continue;
                }

                if (
                    distances[
                        next_index
                    ] != -1
                )
                {
                    continue;
                }

                /*
                 * Prevent diagonal corner cutting.
                 */
                if (
                    dx[i] != 0 &&
                    dy[i] != 0
                )
                {
                    const int side_x =
                        CellIndex(
                            current.x +
                                dx[i],
                            current.y
                        );

                    const int side_y =
                        CellIndex(
                            current.x,
                            current.y +
                                dy[i]
                        );

                    if (
                        !usable[side_x] ||
                        !usable[side_y]
                    )
                    {
                        continue;
                    }
                }

                distances[next_index] =
                    distances[
                        current_index
                    ] + 1;

                queue.push(
                    {
                        nx,
                        ny
                    }
                );
            }
        }

        return candidates;
    }

    // ============================================================
    // GOAL LIST HELPER
    // ============================================================

    bool IsNearGoalList(
        double x,
        double y,
        const std::vector<FailedGoal>
            &goals,
        double radius) const
    {
        for (const auto &goal : goals)
        {
            if (
                Distance(
                    x,
                    y,
                    goal.x,
                    goal.y
                ) < radius
            )
            {
                return true;
            }
        }

        return false;
    }

    // ============================================================
    // SELECT BFS GOAL
    // ============================================================

    bool SelectBFSGoal(
        const std::vector<Candidate>
            &candidates,
        double &goal_x,
        double &goal_y) const
    {
        bool found = false;

        int best_bfs_distance = -1;

        for (const auto &candidate :
             candidates)
        {
            const double distance =
                Distance(
                    robot_x_,
                    robot_y_,
                    candidate.x,
                    candidate.y
                );

            if (
                distance <
                minimum_goal_distance_
            )
            {
                continue;
            }

            /*
             * Exclude previously failed goals.
             */
            if (
                IsNearGoalList(
                    candidate.x,
                    candidate.y,
                    failed_goals_,
                    failed_goal_exclusion_radius_
                )
            )
            {
                continue;
            }

            /*
             * IMPORTANT:
             *
             * Exclude the whole area around every successfully
             * visited goal.
             */
            if (
                IsNearGoalList(
                    candidate.x,
                    candidate.y,
                    visited_goals_,
                    visited_goal_exclusion_radius_
                )
            )
            {
                continue;
            }

            if (
                candidate.bfs_distance >
                best_bfs_distance
            )
            {
                best_bfs_distance =
                    candidate.bfs_distance;

                goal_x = candidate.x;
                goal_y = candidate.y;

                found = true;
            }
        }

        return found;
    }

    // ============================================================
    // UNKNOWN AREA GOAL
    // ============================================================
    //
    // Unknown cells may be ANYWHERE in the map.
    //
    // We search the entire map.
    //
    // The navigation goal itself must still be a known,
    // footprint-safe cell.
    // ============================================================

    bool FindUnknownAreaGoal(
        const std::vector<uint8_t>
            &traversable,
        double &goal_x,
        double &goal_y)
    {
        const int width =
            static_cast<int>(
                map_.info.width
            );

        const int height =
            static_cast<int>(
                map_.info.height
            );

        if (
            width <= 0 ||
            height <= 0
        )
        {
            return false;
        }

        std::vector<Cell>
            unknown_cells;

        const int step =
            std::max(
                1,
                unknown_search_step_
            );

        /*
         * Search the ENTIRE map for unknown cells.
         */
        for (
            int y = 0;
            y < height;
            y += step
        )
        {
            for (
                int x = 0;
                x < width;
                x += step
            )
            {
                const int index =
                    CellIndex(
                        x,
                        y
                    );

                if (
                    map_.data[index] < 0
                )
                {
                    unknown_cells.push_back(
                        {
                            x,
                            y
                        }
                    );
                }
            }
        }

        if (unknown_cells.empty())
        {
            return false;
        }

        bool found = false;

        double best_score =
            std::numeric_limits<double>::max();

        const int search_radius_cells =
            static_cast<int>(
                std::ceil(
                    unknown_goal_max_distance_ /
                    map_.info.resolution
                )
            );

        for (
            const auto &unknown :
            unknown_cells
        )
        {
            for (
                int dy = -search_radius_cells;
                dy <= search_radius_cells;
                ++dy
            )
            {
                for (
                    int dx = -search_radius_cells;
                    dx <= search_radius_cells;
                    ++dx
                )
                {
                    const int gx =
                        unknown.x + dx;

                    const int gy =
                        unknown.y + dy;

                    if (
                        !IsInsideMap(
                            gx,
                            gy
                        )
                    )
                    {
                        continue;
                    }

                    const int goal_index =
                        CellIndex(
                            gx,
                            gy
                        );

                    /*
                     * Goal must be footprint-safe.
                     */
                    if (
                        !traversable[
                            goal_index
                        ]
                    )
                    {
                        continue;
                    }

                    const double
                        distance_to_unknown =
                            std::hypot(
                                dx *
                                    map_.info.resolution,
                                dy *
                                    map_.info.resolution
                            );

                    if (
                        distance_to_unknown <
                        unknown_goal_min_distance_
                    )
                    {
                        continue;
                    }

                    if (
                        distance_to_unknown >
                        unknown_goal_max_distance_
                    )
                    {
                        continue;
                    }

                    double wx;
                    double wy;

                    MapToWorld(
                        gx,
                        gy,
                        wx,
                        wy
                    );

                    /*
                     * Don't select a previously failed location.
                     */
                    if (
                        IsNearGoalList(
                            wx,
                            wy,
                            failed_goals_,
                            failed_goal_exclusion_radius_
                        )
                    )
                    {
                        continue;
                    }

                    /*
                     * IMPORTANT:
                     *
                     * Don't select an unknown-area goal inside
                     * an already visited area.
                     */
                    if (
                        IsNearGoalList(
                            wx,
                            wy,
                            visited_goals_,
                            visited_goal_exclusion_radius_
                        )
                    )
                    {
                        continue;
                    }

                    const double
                        distance_from_robot =
                            Distance(
                                robot_x_,
                                robot_y_,
                                wx,
                                wy
                            );

                    /*
                     * Prefer closer unknown-associated goals.
                     *
                     * The visited-goal exclusion above is what
                     * prevents repeatedly selecting the same area.
                     */
                    const double score =
                        distance_from_robot;

                    if (
                        score < best_score
                    )
                    {
                        best_score = score;

                        goal_x = wx;
                        goal_y = wy;

                        found = true;
                    }
                }
            }
        }

        if (found)
        {
            RCLCPP_INFO(
                get_logger(),
                "Found unknown-area goal: "
                "x=%.2f y=%.2f",
                goal_x,
                goal_y
            );
        }

        return found;
    }

    // ============================================================
    // SEND NAV2 GOAL
    // ============================================================

    void SendGoal(
        double x,
        double y,
        const char *reason)
    {
        if (navigation_interrupted_) {
            return;
        }

        if (
            !action_client_
                ->action_server_is_ready()
        )
        {
            RCLCPP_WARN(
                get_logger(),
                "Nav2 action server is not ready."
            );

            return;
        }

        NavigateToPose::Goal goal;

        goal.pose.header.frame_id =
            "map";

        goal.pose.header.stamp =
            get_clock()->now();

        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.position.z = 0.0;

        goal.pose.pose.orientation.x = 0.0;
        goal.pose.pose.orientation.y = 0.0;
        goal.pose.pose.orientation.z = 0.0;
        goal.pose.pose.orientation.w = 1.0;

        current_goal_x_ = x;
        current_goal_y_ = y;

        rclcpp_action::Client<
            NavigateToPose
        >::SendGoalOptions options;

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
            "Sending %s goal: x=%.2f y=%.2f",
            reason,
            x,
            y
        );

        action_client_->async_send_goal(
            goal,
            options
        );
    }

    // ============================================================
    // NAV2 GOAL RESPONSE
    // ============================================================

    void GoalResponse(
        const GoalHandle::SharedPtr
            &goal_handle)
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

        goal_handle_ = goal_handle;

        RCLCPP_INFO(
            get_logger(),
            "Nav2 accepted goal."
        );

        if (navigation_interrupted_ && cancellation_requested_) {
            RCLCPP_INFO(
                get_logger(),
                "Navigation was interrupted before goal acceptance; cancelling now."
            );
            action_client_->async_cancel_goal(goal_handle_);
        }
    }

    // ============================================================
    // NAV2 RESULT
    // ============================================================

    void ResultResponse(
        const GoalHandle::WrappedResult
            &result)
    {
        if (
            result.code ==
            rclcpp_action::ResultCode::SUCCEEDED
        )
        {
            RCLCPP_INFO(
                get_logger(),
                "Reached goal (%.2f, %.2f)",
                current_goal_x_,
                current_goal_y_
            );

            /*
             * Permanently remember this goal for this mission.
             *
             * Future BFS and unknown-area candidates within
             * visited_goal_exclusion_radius_ will be rejected.
             */
            visited_goals_.push_back(
                {
                    current_goal_x_,
                    current_goal_y_
                }
            );

            RCLCPP_INFO(
                get_logger(),
                "Visited-goal exclusion area recorded "
                "around (%.2f, %.2f), radius %.2f m.",
                current_goal_x_,
                current_goal_y_,
                visited_goal_exclusion_radius_
            );
        }
        else if (
            result.code == rclcpp_action::ResultCode::CANCELED &&
            cancellation_requested_
        )
        {
            RCLCPP_INFO(
                get_logger(),
                "Nav2 goal cancelled because garbage was detected."
            );
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "Navigation failed with result code %d",
                static_cast<int>(
                    result.code
                )
            );

            RecordFailedGoal();
        }

        goal_active_ = false;
        goal_handle_.reset();
        cancellation_requested_ = false;
    }

    // ============================================================
    // RECORD FAILED GOAL
    // ============================================================

    void RecordFailedGoal()
    {
        failed_goals_.push_back(
            {
                current_goal_x_,
                current_goal_y_
            }
        );

        RCLCPP_WARN(
            get_logger(),
            "Excluding failed goal (%.2f, %.2f)",
            current_goal_x_,
            current_goal_y_
        );
    }

    // ============================================================
    // STOP ROBOT
    // ============================================================

    void StopRobot()
    {
        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;

        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;

        cmd_vel_pub_->publish(cmd);
    }

    // ============================================================
    // START CONTROLLED ROTATION
    // ============================================================

    void StartControlledRotation()
    {
        if (
            recovery_state_ ==
            RecoveryState::ROTATING
        )
        {
            return;
        }

        recovery_state_ =
            RecoveryState::ROTATING;

        recovery_rotation_start_ =
            SteadyClock::now();

        rotation_command_sent_ = false;

        RCLCPP_WARN(
            get_logger(),
            "No usable goal found. "
            "Starting controlled %.1f degree rotation.",
            recovery_rotation_degrees_
        );
    }

    // ============================================================
    // CONTROLLED ROTATION
    // ============================================================

    void HandleRotation()
    {
        const auto now =
            SteadyClock::now();

        const double
            rotation_radians =
                std::abs(
                    recovery_rotation_degrees_
                ) *
                M_PI /
                180.0;

        const double
            duration_seconds =
                rotation_radians /
                std::max(
                    0.01,
                    std::abs(
                        recovery_rotation_speed_
                    )
                );

        const double elapsed =
            std::chrono::duration<double>(
                now -
                recovery_rotation_start_
            ).count();

        if (!rotation_command_sent_)
        {
            geometry_msgs::msg::Twist cmd;

            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;

            cmd.angular.x = 0.0;
            cmd.angular.y = 0.0;

            cmd.angular.z =
                recovery_rotation_speed_;

            cmd_vel_pub_->publish(cmd);

            rotation_command_sent_ = true;

            RCLCPP_INFO(
                get_logger(),
                "Rotating %.1f degrees "
                "(%.2f rad/s, %.2f s).",
                recovery_rotation_degrees_,
                recovery_rotation_speed_,
                duration_seconds
            );
        }
        else
        {
            geometry_msgs::msg::Twist cmd;

            cmd.angular.z =
                recovery_rotation_speed_;

            cmd_vel_pub_->publish(cmd);
        }

        if (
            elapsed >=
            duration_seconds
        )
        {
            StopRobot();

            recovery_wait_start_ =
                SteadyClock::now();

            recovery_state_ =
                RecoveryState::WAITING_FOR_MAP;

            RCLCPP_INFO(
                get_logger(),
                "Rotation finished. "
                "Waiting %.1f s for SLAM/map update.",
                recovery_wait_after_rotation_
            );
        }
    }

    // ============================================================
    // CHECK GOALS DURING RECOVERY
    // ============================================================

    bool TryFindRecoveryGoal()
    {
        if (!map_received_)
        {
            return false;
        }

        if (!GetRobotPose())
        {
            return false;
        }

        int robot_cell_x;
        int robot_cell_y;

        if (
            !WorldToMap(
                robot_x_,
                robot_y_,
                robot_cell_x,
                robot_cell_y
            )
        )
        {
            return false;
        }

        const auto traversable =
            BuildTraversabilityGrid();

        const auto candidates =
            BFS(
                robot_cell_x,
                robot_cell_y,
                traversable
            );

        double goal_x;
        double goal_y;

        /*
         * First priority: BFS.
         */
        if (
            SelectBFSGoal(
                candidates,
                goal_x,
                goal_y
            )
        )
        {
            RCLCPP_INFO(
                get_logger(),
                "Recovery found new BFS goal."
            );

            recovery_state_ =
                RecoveryState::NORMAL;

            recovery_active_ = false;

            SendGoal(
                goal_x,
                goal_y,
                "BFS"
            );

            return true;
        }

        /*
         * Second priority: unknown area anywhere in map.
         */
        if (
            FindUnknownAreaGoal(
                traversable,
                goal_x,
                goal_y
            )
        )
        {
            RCLCPP_INFO(
                get_logger(),
                "Recovery found new unknown-area goal."
            );

            recovery_state_ =
                RecoveryState::NORMAL;

            recovery_active_ = false;

            SendGoal(
                goal_x,
                goal_y,
                "unknown-area"
            );

            return true;
        }

        return false;
    }

    // ============================================================
    // WAIT FOR MAP
    // ============================================================

    void HandleWaitingForMap()
    {
        const auto now =
            SteadyClock::now();

        const double elapsed =
            std::chrono::duration<double>(
                now -
                recovery_wait_start_
            ).count();

        if (
            elapsed <
            recovery_wait_after_rotation_
        )
        {
            return;
        }

        if (TryFindRecoveryGoal())
        {
            return;
        }

        RCLCPP_WARN(
            get_logger(),
            "No new BFS goal and no new unknown-area goal. "
            "Continuing controlled rotation."
        );

        StartControlledRotation();
    }

    // ============================================================
    // RECOVERY LOOP
    // ============================================================

    void RecoveryLoop()
    {
        if (goal_active_)
        {
            StopRobot();
            return;
        }

        if (
            recovery_state_ ==
            RecoveryState::ROTATING
        )
        {
            HandleRotation();
            return;
        }

        if (
            recovery_state_ ==
            RecoveryState::WAITING_FOR_MAP
        )
        {
            StopRobot();
            HandleWaitingForMap();
            return;
        }

        StartControlledRotation();
    }

    // ============================================================
    // MAIN LOOP
    // ============================================================

    void MainLoop()
    {
        // Keep the control node synchronized with the current owner of /cmd_vel.
        PublishExplorationActive(exploration_active_);

        // Stay paused after a detection-triggered cancellation. The control
        // node can take over /cmd_vel without this node sending another goal.
        if (navigation_interrupted_) {
            return;
        }

        /*
         * Navigation is currently active.
         */
        if (goal_active_)
        {
            if (
                recovery_state_ !=
                RecoveryState::NORMAL
            )
            {
                StopRobot();

                recovery_state_ =
                    RecoveryState::NORMAL;

                recovery_active_ = false;
            }

            return;
        }

        if (!map_received_)
        {
            return;
        }

        if (
            !action_client_
                ->action_server_is_ready()
        )
        {
            return;
        }

        /*
         * --------------------------------------------------------
         * RECOVERY
         * --------------------------------------------------------
         */

        if (
            recovery_state_ !=
            RecoveryState::NORMAL
        )
        {
            RecoveryLoop();
            return;
        }

        /*
         * --------------------------------------------------------
         * NORMAL BFS
         * --------------------------------------------------------
         */

        if (!GetRobotPose())
        {
            return;
        }

        int robot_cell_x;
        int robot_cell_y;

        if (
            !WorldToMap(
                robot_x_,
                robot_y_,
                robot_cell_x,
                robot_cell_y
            )
        )
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Robot is outside the map."
            );

            return;
        }

        const auto traversable =
            BuildTraversabilityGrid();

        const auto candidates =
            BFS(
                robot_cell_x,
                robot_cell_y,
                traversable
            );

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "BFS found %zu reachable cells. "
            "Visited goals: %zu.",
            candidates.size(),
            visited_goals_.size()
        );

        double goal_x;
        double goal_y;

        /*
         * FIRST:
         *
         * Try a new BFS goal.
         */
        if (
            SelectBFSGoal(
                candidates,
                goal_x,
                goal_y
            )
        )
        {
            SendGoal(
                goal_x,
                goal_y,
                "BFS"
            );

            return;
        }

        /*
         * SECOND:
         *
         * Search entire map for unknown-area goal.
         */
        if (
            FindUnknownAreaGoal(
                traversable,
                goal_x,
                goal_y
            )
        )
        {
            SendGoal(
                goal_x,
                goal_y,
                "unknown-area"
            );

            return;
        }

        /*
         * NOTHING LEFT:
         *
         * Start indefinite controlled rotation.
         */
        RCLCPP_WARN(
            get_logger(),
            "No unused BFS goal and no unused "
            "unknown-area goal. "
            "Entering indefinite controlled rotation."
        );

        recovery_active_ = true;

        StartControlledRotation();
    }
};


// ================================================================
// MAIN
// ================================================================

int main(
    int argc,
    char **argv)
{
    rclcpp::init(
        argc,
        argv
    );

    auto node =
        std::make_shared<MissionManager>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
