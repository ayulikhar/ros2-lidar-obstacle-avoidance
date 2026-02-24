#include <memory>
#include <vector>
#include <queue>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std;

struct AStarNode {
    int x, y;
    int g, h;
    AStarNode* parent;
    int f() const { return g + h; }
};

struct Compare {
    bool operator()(AStarNode* a, AStarNode* b) {
        return a->f() > b->f();
    }
};

class AStarPlanner : public rclcpp::Node
{
public:
    AStarPlanner() : Node("astar_planner")
    {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

        compute_and_publish_path();
    }

private:

    int heuristic(int x1, int y1, int x2, int y2) {
        return abs(x1 - x2) + abs(y1 - y2);
    }

    bool isValid(int x, int y, vector<vector<int>>& grid) {
        return x >= 0 && y >= 0 &&
               x < (int)grid.size() &&
               y < (int)grid[0].size() &&
               grid[x][y] == 0;
    }

    void compute_and_publish_path()
    {
        vector<vector<int>> grid = {
            {0,0,0,0,1},
            {1,1,0,1,0},
            {0,0,0,0,0}
        };

        pair<int,int> start = {0,0};
        pair<int,int> goal = {2,4};

        priority_queue<AStarNode*, vector<AStarNode*>, Compare> openList;
        vector<vector<bool>> closed(grid.size(), vector<bool>(grid[0].size(), false));

        AStarNode* startNode = new AStarNode{
            start.first,
            start.second,
            0,
            heuristic(start.first, start.second, goal.first, goal.second),
            nullptr
        };

        openList.push(startNode);

        int dx[4] = {1, -1, 0, 0};
        int dy[4] = {0, 0, 1, -1};

        AStarNode* goalNode = nullptr;

        while (!openList.empty()) {
            AStarNode* current = openList.top();
            openList.pop();

            if (current->x == goal.first &&
                current->y == goal.second)
            {
                goalNode = current;
                break;
            }

            closed[current->x][current->y] = true;

            for (int i = 0; i < 4; i++) {
                int newX = current->x + dx[i];
                int newY = current->y + dy[i];

                if (isValid(newX, newY, grid) &&
                    !closed[newX][newY])
                {
                    AStarNode* neighbor = new AStarNode{
                        newX,
                        newY,
                        current->g + 1,
                        heuristic(newX, newY, goal.first, goal.second),
                        current
                    };

                    openList.push(neighbor);
                }
            }
        }

        if (goalNode == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "No Path Found.");
            return;
        }

        // Convert path to ROS message
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";

        float scale = 0.5;  // grid cell size
        float offset_x = -1.0;
        float offset_y = -1.0;

        AStarNode* current = goalNode;
        while (current != nullptr) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";

            pose.pose.position.x = offset_x + current->x * scale;
            pose.pose.position.y = offset_y + current->y * scale;
            pose.pose.orientation.w = 1.0;

            path_msg.poses.push_back(pose);
            current = current->parent;
        }

        reverse(path_msg.poses.begin(), path_msg.poses.end());

        path_pub_->publish(path_msg);

        RCLCPP_INFO(this->get_logger(), "Path Published.");
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarPlanner>());
    rclcpp::shutdown();
    return 0;
}
