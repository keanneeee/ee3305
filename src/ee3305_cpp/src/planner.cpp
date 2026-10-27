#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee3305
{
    // define the Nodes
    struct DijkstraNode
    {
        DijkstraNode *parent;
        double f;
        double g;
        double h;
        int r;
        int c;
        bool expanded;

        // constructor
        DijkstraNode(int c, int r)
            : parent(nullptr), f(INFINITY), g(INFINITY), h(INFINITY), r(r), c(c), expanded(false)
        {
        }
    };

    // Define the Open list and comparators using std::priority_queue
    template <typename T> // T must be pointer type
    struct OpenListComparator
    {
        bool operator()(const T &l, const T &r) const { return l->f > r->f; };
    };

    template <typename T> // T must be pointer type
    using OpenList = std::priority_queue<T, std::deque<T>, OpenListComparator<T>>;

    class Planner : public rclcpp::Node
    {

    private:
        // Node Instance Properties =============================================================

        // Handles
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_request_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_global_costmap_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Data from Subscriptions
        std::vector<int8_t> costmap_; // read only; written only by subscriber(s)
        double costmap_resolution_;   // read only; written only by subscriber(s)
        double costmap_origin_x_;     // read only; written only by subscriber(s)
        double costmap_origin_y_;     // read only; written only by subscriber(s)
        int costmap_rows_;            // read only; written only by subscriber(s)
        int costmap_cols_;            // read only; written only by subscriber(s)
        double rbt_x_;                // read only; written only by subscriber(s)
        double rbt_y_;                // read only; written only by subscriber(s)
        double goal_x_;               // read only; written only by subscriber(s)
        double goal_y_;               // read only; written only by subscriber(s)

        // Parameters
        double max_access_cost_; // read only; written only in constructor

        // Other Instance Variables
        bool has_new_request_;
        bool received_map_;

    public:
        explicit Planner(std::string node_name = "planner")
            : rclcpp::Node(node_name)
        {
            // Node Constructor =============================================================

            // Declare Parameters
            this->declare_parameter<int>("max_access_cost", 100);

            // Get Parameters
            this->max_access_cost_ = this->get_parameter("max_access_cost").get_value<int>();

            // Handles: Subscribers
            // Global costmap subscriber
            auto qos_profile_latch = rclcpp::ServicesQoS();
            qos_profile_latch.durability(rclcpp::DurabilityPolicy::TransientLocal); // latching
            this->sub_global_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "global_costmap",
                qos_profile_latch, // latching qos. Only one message is published, and at the start of launch.
                std::bind(&Planner::callbackSubGlobalCostmap_, this, std::placeholders::_1));

            // Path request subscriber
            this->sub_path_request_ = this->create_subscription<nav_msgs::msg::Path>(
                "path_request",
                10,
                std::bind(&Planner::callbackSubPathRequest_, this, std::placeholders::_1));

            // Handles: Publishers
            // Path publisher
            this->pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
                "path",
                10);

            // Timers:
            this->timer_ = this->create_timer(
                0.1s,
                std::bind(&Planner::callbackTimer_, this));

            // Instance Variables
            this->has_new_request_ = false;
            this->received_map_ = false;
        }

    private:
        // Callbacks =============================================================

        // Path request subscriber callback
        void callbackSubPathRequest_(nav_msgs::msg::Path::SharedPtr msg)
        {
            if (msg->poses.size() < 2)
            {
                RCLCPP_WARN(this->get_logger(), "Path request must contain start and goal poses.");
                return;
            }
            // write to rbt_x_, rbt_y_, goal_x_, goal_y_.
            // The behavior node writes the robot coordinates at [0] and the goal coordinates at [1].
            this->rbt_x_ = msg->poses[0].pose.position.x;
            this->rbt_y_ = msg->poses[0].pose.position.y;
            this->goal_x_ = msg->poses[1].pose.position.x;
            this->goal_y_ = msg->poses[1].pose.position.y;

            this->has_new_request_ = true;
        }

        // Global costmap subscriber callback
        // This is only run once because the costmap is only published once, at the start of the launch.
        void callbackSubGlobalCostmap_(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            // write to costmap_, costmap_resolution_, costmap_origin_x_, costmap_origin_y_, costmap_rows_, costmap_cols_
            this->costmap_ = msg->data;
            this->costmap_resolution_ = msg->info.resolution;
            this->costmap_origin_x_ = msg->info.origin.position.x;
            this->costmap_origin_y_ = msg->info.origin.position.y;
            this->costmap_rows_ = msg->info.height;
            this->costmap_cols_ = msg->info.width;

            this->received_map_ = true;
        }

        // runs the path planner at regular intervals as long as there is a new path request.
        void callbackTimer_()
        {
            if (!this->received_map_ || !this->has_new_request_)
                return; // silently return if no new request or map is not received.

            // run the path planner
            dijkstra_(this->rbt_x_, this->rbt_y_, this->goal_x_, this->goal_y_);

            // reset the request
            this->has_new_request_ = false;
        }

        // Publish the interpolated path for testing
        void publishInterpolatedPath(double start_x, double start_y, double goal_x, double goal_y)
        {
            nav_msgs::msg::Path msg_path;
            msg_path.header.stamp = this->get_clock()->now();
            msg_path.header.frame_id = "map";

            double dx = start_x - goal_x;
            double dy = start_y - goal_y;
            double distance = hypot(dx, dy);
            double steps = distance / 0.05;

            // Generate poses at every 0.05m
            for (double i = 0; i < steps; ++i)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = goal_x + dx * i / steps;
                pose.pose.position.y = goal_y + dy * i / steps;
                msg_path.poses.push_back(pose);
            }

            // Add the goal pose
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = goal_x;
            pose.pose.position.y = goal_y;
            msg_path.poses.push_back(pose);

            // Reverse the path (hint)
            std::reverse(msg_path.poses.begin(), msg_path.poses.end()); // this is a hint and is on purpose

            // Publish the path
            this->pub_path_->publish(msg_path);

            RCLCPP_INFO(this->get_logger(), "Publishing interpolated path between Start and Goal. Implement dijkstra_() instead.");
        }

        // Converts world coordinates to cell column and cell row.
        std::pair<int, int> XYToCR_(double x, double y)
        {
            int c = static_cast<int>(std::floor((x - costmap_origin_x_) / costmap_resolution_));
            int r = static_cast<int>(std::floor((y - costmap_origin_y_) / costmap_resolution_));
            return {c, r};
        }

        // Converts cell column and cell row to world coordinates.
        std::pair<double, double> CRToXY_(int c, int r)
        {
            double x = costmap_origin_x_ + (c + 0.5) * costmap_resolution_;
            double y = costmap_origin_y_ + (r + 0.5) * costmap_resolution_;

            return {x, y};
        }

        // Converts cell column and cell row to flattened array index.
        int CRToIndex_(int c, int r)
        {
            return r * costmap_cols_ + c;
        }

        // Returns true if the cell column and cell row is outside the costmap.
        bool outOfMap_(int c, int r)
        {
            return c < 0 || r < 0 || c >= costmap_cols_ || r >= costmap_rows_;
        }

        // Runs the path planning algorithm based on the world coordinates.
        void dijkstra_(double start_x, double start_y, double goal_x, double goal_y)
        {
            // Initializations ---------------------------------
            if (costmap_cols_ <= 0 || costmap_rows_ <= 0 || costmap_resolution_ <= 0.0)
                return;

            std::vector<DijkstraNode> nodes;
            nodes.reserve(static_cast<size_t>(costmap_rows_) * costmap_cols_);
            for (int r = 0; r < costmap_rows_; ++r)
                for (int c = 0; c < costmap_cols_; ++c)
                    nodes.emplace_back(c, r);

            auto [rbt_c, rbt_r] = this->XYToCR_(start_x, start_y);
            auto [goal_c, goal_r] = this->XYToCR_(goal_x, goal_y);
            if (outOfMap_(rbt_c, rbt_r) || outOfMap_(goal_c, goal_r))
            {
                RCLCPP_WARN(this->get_logger(), "Start or goal is outside the costmap.");
                return;
            }
            int rbt_idx = this->CRToIndex_(rbt_c, rbt_r);
            int goal_idx = this->CRToIndex_(goal_c, goal_r);
            DijkstraNode *start_node = &nodes[rbt_idx];
            start_node->g = 0.0;
            start_node->f = 0.0;

            // open list
            OpenList<DijkstraNode *> open_list;
            open_list.push(start_node);

            // Expansion Loop ---------------------------------------------
            while (!open_list.empty())
            {
                // Poll the cheapest node
                DijkstraNode *node = open_list.top();
                open_list.pop();

                if (node->expanded)
                    continue;
                node->expanded = true;

                // Return path if reached goal
                if (node->c == goal_c && node->r == goal_r)
                {
                    nav_msgs::msg::Path msg_path;
                    msg_path.header.stamp = this->get_clock()->now();
                    msg_path.header.frame_id = "map";

                    std::vector<geometry_msgs::msg::PoseStamped> reverse_path;
                    for (DijkstraNode *current = node; current != nullptr; current = current->parent)
                    {
                        auto [x, y] = CRToXY_(current->c, current->r);
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header = msg_path.header;
                        pose.pose.position.x = x;
                        pose.pose.position.y = y;
                        pose.pose.orientation.w = 1.0;
                        reverse_path.push_back(pose);
                    }
                    std::reverse(reverse_path.begin(), reverse_path.end());
                    msg_path.poses = std::move(reverse_path);

                    // publish path
                    this->pub_path_->publish(msg_path);

                    RCLCPP_INFO(this->get_logger(),
                                "Path Found from Rbt @ (%7.3f, %7.3f) to Goal @ (%7.3f, %7.3f)!",
                                start_x, start_y, goal_x, goal_y);

                    return;
                }

                // Neighbor Loop --------------------------------------------------
                for (auto [dc, dr] : std::vector<std::pair<int, int>>{{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}})
                {
                    int nb_c = node->c + dc;
                    int nb_r = node->r + dr;

                    if (outOfMap_(nb_c, nb_r))
                        continue;
                    int nb_idx = CRToIndex_(nb_c, nb_r);
                    if (costmap_[nb_idx] < 0 || costmap_[nb_idx] > max_access_cost_)
                        continue;

                    // Extract pointer to neighboring node
                    DijkstraNode *nb_node = &nodes[nb_idx];

                    if (nb_node->expanded)
                        continue;

                    // Ignore if the cell cost exceeds max_access_cost_ (to avoid passing through obstacles)

                    const double distance = (dc != 0 && dr != 0) ? std::sqrt(2.0) : 1.0;
                    const double new_g = node->g + distance * (static_cast<double>(costmap_[nb_idx]) + 1.0);
                    if (new_g < nb_node->g)
                    {
                        nb_node->parent = node;
                        nb_node->g = new_g;
                        nb_node->f = new_g;
                        open_list.push(nb_node);
                    }
                }
            }
            nav_msgs::msg::Path empty_path;
            empty_path.header.stamp = this->get_clock()->now();
            empty_path.header.frame_id = "map";
            this->pub_path_->publish(empty_path);
            RCLCPP_WARN(this->get_logger(), "No Path Found!");
        }
    };
}

// Main Boiler Plate =============================================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ee3305::Planner>());
    rclcpp::shutdown();
    return 0;
}
