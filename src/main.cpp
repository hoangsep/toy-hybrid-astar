
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>
#include "gif.h"

const double PI = 3.14159265358979323846;
const int GRID_SIZE = 100; // 100m x 100m grid
const double VEHICLE_LENGTH = 3.32;
const int HEADING_BINS = 72;
const double HEADING_BIN_SIZE = 5 * (PI / 180); // 5 degrees in radians
const double STEP_SIZE = 1.0;
const double DIRECTION_CHANGE_COST = 10.0;
const double REVERSE_COST = 15.0;

const int SCALE = 10; // Scaling factor to make the display larger

// Add this to store frames
std::vector<cv::Mat> frames;

struct State
{
    double x;
    double y;
    int heading_bin;
    int direction; // 1 for forward, -1 for reverse
    bool operator==(const State& other) const
    {
        return static_cast<int>(x) == static_cast<int>(other.x) &&
               static_cast<int>(y) == static_cast<int>(other.y) &&
               heading_bin == other.heading_bin && direction == other.direction;
    }
};

namespace std
{
template <> struct hash<State>
{
    std::size_t operator()(const State& state) const
    {
        return std::hash<int>()(static_cast<int>(state.x)) ^
               std::hash<int>()(static_cast<int>(state.y)) ^ std::hash<int>()(state.heading_bin) ^
               std::hash<int>()(state.direction);
    }
};
} // namespace std

struct Node
{
    State state;
    double cost;
    double heuristic;
    Node* parent;
    bool operator>(const Node& other) const
    {
        return cost + heuristic > other.cost + other.heuristic;
    }
};

// simplified Dijkstra node for heuristic computation
struct DijkstraNode
{
    int x;
    int y;
    double cost;

    bool operator>(const DijkstraNode& other) const { return cost > other.cost; }
};

double euclideanDistance(const State& a, const State& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

int binHeading(double theta)
{
    int bin = static_cast<int>(std::round(theta / HEADING_BIN_SIZE)) % HEADING_BINS;
    return bin < 0 ? bin + HEADING_BINS : bin;
}

bool isValidState(const State& state, const std::vector<std::vector<int>>& grid)
{
    const auto x = static_cast<int>(std::round(state.x));
    const auto y = static_cast<int>(std::round(state.y));
    return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE && grid[y][x] == 0;
}

std::vector<State> getNeighbors(const State& current)
{
    std::vector<State> neighbors;
    double current_theta = current.heading_bin * HEADING_BIN_SIZE;

    std::vector<int> steering_angles = {0,   5,  -5,  10, -10, 15, -15, 20,
                                        -20, 25, -25, 30, -30, 35, -35};
    std::vector<int> directions = {1, -1};

    for (int direction : directions)
    {
        for (int steering_angle : steering_angles)
        {
            double steering_rad = steering_angle * (PI / 180);
            double next_x = current.x + direction * STEP_SIZE * std::cos(current_theta);
            double next_y = current.y + direction * STEP_SIZE * std::sin(current_theta);
            double next_theta =
                current_theta + direction * (STEP_SIZE / VEHICLE_LENGTH) * std::tan(steering_rad);
            next_theta = std::fmod(next_theta + 2 * PI, 2 * PI); // normalize angle
            neighbors.push_back({next_x, next_y, binHeading(next_theta), direction});
        }
    }

    return neighbors;
}

double calculateCost(const State& current, const State& neighbor)
{
    double cost = STEP_SIZE;

    cost += DIRECTION_CHANGE_COST * abs(current.direction - neighbor.direction);

    if (neighbor.direction == -1)
    {
        cost += REVERSE_COST;
    }

    return cost;
}

std::vector<State> reconstructPath(Node* node)
{
    std::vector<State> path;
    while (node)
    {
        path.push_back(node->state);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void initializeDisplay(const std::vector<std::vector<int>>& grid, cv::Mat& display)
{

    for (size_t row = 0; row < GRID_SIZE; ++row)
    {
        for (size_t col = 0; col < GRID_SIZE; ++col)
        {
            // draw grid lines
            cv::Point top_left(col * SCALE, row * SCALE);
            cv::Point bottom_right((col + 1) * SCALE, (row + 1) * SCALE);
            cv::rectangle(display, top_left, bottom_right, cv::Scalar(0, 0, 0), 1);
        }
    }
}

void updateDisplay(const std::vector<std::vector<int>>& grid, const cv::Mat& display,
                   const std::vector<State>& path)
{
    // Draw path
    for (size_t i = 1; i < path.size(); ++i)
    {
        cv::line(display, cv::Point(path[i - 1].x * SCALE, path[i - 1].y * SCALE),
                 cv::Point(path[i].x * SCALE, path[i].y * SCALE), cv::Scalar(0, 0, 255), 2);
    }

    // Display start and goal points
    if (!path.empty())
    {
        cv::circle(display, cv::Point(path.front().x * SCALE, path.front().y * SCALE), 3,
                   cv::Scalar(0, 255, 0), cv::FILLED);
        cv::circle(display, cv::Point(path.back().x * SCALE, path.back().y * SCALE), 3,
                   cv::Scalar(255, 0, 0), cv::FILLED);
    }

    cv::imshow("Hybrid A* Path", display);
    cv::waitKey(1); // Wait for a short time to visualize the progress
}

void saveGif(const std::string& filename, const std::vector<cv::Mat>& frames, int delay)
{
    GifWriter gif;
    GifBegin(&gif, filename.c_str(), frames[0].cols, frames[0].rows, delay);

    for (const auto& frame : frames)
    {
        std::vector<uint8_t> image(frame.rows * frame.cols * 4);
        for (int y = 0; y < frame.rows; ++y)
        {
            for (int x = 0; x < frame.cols; ++x)
            {
                cv::Vec3b color = frame.at<cv::Vec3b>(y, x);
                image[4 * (y * frame.cols + x) + 0] = color[0];
                image[4 * (y * frame.cols + x) + 1] = color[1];
                image[4 * (y * frame.cols + x) + 2] = color[2];
                image[4 * (y * frame.cols + x) + 3] = 255;
            }
        }
        GifWriteFrame(&gif, image.data(), frame.cols, frame.rows, delay);
    }

    GifEnd(&gif);
}

// Simplified Reed-Shepp path generation (for demonstration)
std::vector<State> computeFakeReedSheppPath(const State& start, const State& goal)
{
    std::vector<State> path;
    double dx = goal.x - start.x;
    double dy = goal.y - start.y;
    double dtheta = (goal.heading_bin - start.heading_bin) * HEADING_BIN_SIZE;
    int steps = std::max(std::abs(dx), std::abs(dy)) / STEP_SIZE;
    for (int i = 0; i <= steps; ++i)
    {
        double t = static_cast<double>(i) / steps;
        double x = start.x + t * dx;
        double y = start.y + t * dy;
        double theta = start.heading_bin * HEADING_BIN_SIZE + t * dtheta;
        path.push_back({x, y, binHeading(theta), 1});
    }
    return path;
}

bool checkCollision(const std::vector<State>& path, const std::vector<std::vector<int>>& grid)
{
    for (const auto& state : path)
    {
        if (!isValidState(state, grid))
        {
            return true;
        }
    }
    return false;
}

std::vector<std::vector<double>> computeDijkstraHeuristic(const State& goal,
                                                          const std::vector<std::vector<int>>& grid)
{
    std::vector<std::pair<int, int>> directions = {{-1, 0},  {1, 0},  {0, -1}, {0, 1},
                                                   {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
    std::vector<std::vector<double>> heuristic_map(
        GRID_SIZE, std::vector<double>(GRID_SIZE, std::numeric_limits<double>::infinity()));
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>>
        open_set;

    heuristic_map[static_cast<int>(goal.y)][static_cast<int>(goal.x)] = 0;
    open_set.push({static_cast<int>(goal.x), static_cast<int>(goal.y), 0});

    while (!open_set.empty())
    {
        DijkstraNode current = open_set.top();
        open_set.pop();

        for (const auto& direction : directions)
        {
            const auto nx = current.x + direction.first;
            const auto ny = current.y + direction.second;
            if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE && grid[nx][ny] == 0)
            {
                const auto new_cost = current.cost + 1;
                if (new_cost < heuristic_map[ny][nx])
                {
                    heuristic_map[ny][nx] = new_cost;
                    open_set.push({nx, ny, new_cost});
                }
            }
        }
    }

    return heuristic_map;
}

void colorHeuristicMap(const std::vector<std::vector<double>>& heuristic_map, cv::Mat& display)
{
    // find the maximum cost in the heuristic map to scale the colors later
    double max_heuristic_cost = std::numeric_limits<double>::min();
    for (const auto& row : heuristic_map)
    {
        auto row_max_it =
            std::max_element(row.begin(), row.end(),
                             [](int a, int b)
                             {
                                 return (a == std::numeric_limits<int>::max() ? true : a < b) &&
                                        b != std::numeric_limits<int>::max();
                             });

        if (row_max_it != row.end() && *row_max_it != std::numeric_limits<int>::max())
        {
            max_heuristic_cost = std::max(max_heuristic_cost, *row_max_it);
        }
    }

    // Color the cells on display based on the value of heuristic_map
    for (size_t x = 0; x < GRID_SIZE; ++x)
    {
        for (size_t y = 0; y < GRID_SIZE; ++y)
        {
            double cost = heuristic_map[x][y];
            if (cost == std::numeric_limits<double>::infinity())
            {
                // Cell is unreachable
                cv::rectangle(display, cv::Point(x * SCALE, y * SCALE),
                              cv::Point((x + 1) * SCALE, (y + 1) * SCALE), cv::Scalar(0, 0, 0),
                              cv::FILLED);
                continue;
            }
            else
            {
                // color the cell based on the cost
                const auto green = static_cast<int>(255 * (1 - cost / max_heuristic_cost));
                const auto red = static_cast<int>(255 * (cost / max_heuristic_cost));
                cv::rectangle(display, cv::Point(x * SCALE, y * SCALE),
                              cv::Point((x + 1) * SCALE, (y + 1) * SCALE),
                              cv::Scalar(0, green, red), cv::FILLED);
            }
        }
    }
}

std::vector<State> hybridAstar(const State& start, const State& goal,
                               const std::vector<std::vector<int>>& grid, const double goal_tol,
                               cv::Mat display)
{
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::unordered_set<State> closed_set;
    std::unordered_map<State, double> cost_so_far;

    const auto heuristic_map = computeDijkstraHeuristic(goal, grid);

    colorHeuristicMap(heuristic_map, display);

    open_set.push(
        {start, 0, heuristic_map[static_cast<int>(start.y)][static_cast<int>(start.x)], nullptr});
    cost_so_far[start] = 0;

    unsigned int frame_count{0};
    while (!open_set.empty())
    {
        Node current = open_set.top();
        open_set.pop();

        // Display the current node being searched
        cv::circle(display, cv::Point(current.state.x * SCALE, current.state.y * SCALE), 2,
                   cv::Scalar(255, 0, 0), cv::FILLED);
        cv::imshow("Hybrid A* Path", display);
        if (frame_count++ % 10 == 0)
        {
            frames.push_back(display.clone()); // Save the current frame
        }
        cv::waitKey(1); // Wait for a short time to visualize the progress

        if (std::hypot(current.state.x - goal.x, current.state.y - goal.y) < goal_tol)
        {
            return reconstructPath(&current);
        }

        if (closed_set.find(current.state) != closed_set.end())
        {
            continue;
        }

        std::vector<State> reed_shepp_path = computeFakeReedSheppPath(current.state, goal);
        if (!checkCollision(reed_shepp_path, grid))
        {
            if (!checkCollision(reed_shepp_path, grid))
            {
                auto final_path = reconstructPath(&current);
                final_path.insert(final_path.end(), reed_shepp_path.begin(), reed_shepp_path.end());
                return final_path;
            }
        }

        closed_set.insert(current.state);

        for (const State& neighbor : getNeighbors(current.state))
        {
            if (!isValidState(neighbor, grid))
            {
                continue;
            }

            const auto new_cost =
                cost_so_far[current.state] + calculateCost(current.state, neighbor);
            if (cost_so_far.find(neighbor) == cost_so_far.end() || new_cost < cost_so_far[neighbor])
            {
                cost_so_far[neighbor] = new_cost;
                open_set.push(
                    {neighbor, new_cost,
                     heuristic_map[static_cast<int>(neighbor.x)][static_cast<int>(neighbor.y)],
                     new Node(current)});
            }
        }
    }
    return {}; // No path found
}

int main()
{
    State start = {0, 0, binHeading(0)};
    State goal = {99, 99, binHeading(0)};

    std::vector<std::vector<int>> grid(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));

    // Add obstacle
    for (size_t row = 0; row < 40; ++row)
    {
        for (size_t col = 20; col < 30; ++col)
        {
            grid[row][col] = 1;
        }
    }

    cv::Mat display(GRID_SIZE * SCALE, GRID_SIZE * SCALE, CV_8UC3, cv::Scalar(255, 255, 255));
    initializeDisplay(grid, display);

    std::vector<State> path = hybridAstar(start, goal, grid, 1.0, display);

    if (!path.empty())
    {
        updateDisplay(grid, display, path);
        cv::waitKey(0); // Wait indefinitely to allow viewing the final path
    }
    else
    {
        std::cout << "No path found" << std::endl;
    }

    frames.push_back(display.clone()); // Save the final frame
    saveGif("path.gif", frames,
            10); // Save the frames as a GIF with a 10ms delay between frames

    return 0;
}
