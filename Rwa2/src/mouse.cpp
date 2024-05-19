#include "../include/mouse/mouse.h"
#include "../include/api/api.h"
#include "../include/util/util.h"
#include <string>
#include <iostream>
#include <stdio.h>
#include <stack>
#include <vector>
#include <algorithm>
#include <utility>

void rwa2::Mouse::display_walls()
{
    for (int x = 0; x < m_maze_width; x += 1)
    {
        for (int y = 0; y < m_maze_height; y += 1)
        {
            if (m_maze.at(x).at(y).is_wall(direction::NORTH))
            {
                //display this wall in the simulator
                API::setWall(x, y, 'n');
            }

            if (m_maze.at(x).at(y).is_wall(direction::EAST))
            {
                //display this wall in the simulator
                API::setWall(x, y, 'e');
            }

            if (m_maze.at(x).at(y).is_wall(direction::SOUTH))
            {
                //display this wall in the simulator
                API::setWall(x, y, 's');
            }

            if (m_maze.at(x).at(y).is_wall(direction::WEST))
            {
                //display this wall in the simulator
                API::setWall(x, y, 'w');
            }
            //display the number of walls surrounding the current node
            API::setText(x, y, std::to_string(m_maze.at(x).at(y).compute_number_of_walls()));
        }
    }
}

void rwa2::Mouse::log(const std::string &text)
{
    std::cerr << text << std::endl;
}

bool rwa2::Mouse::find(const std::vector<std::pair<int, int>> &list, const std::pair<int, int> &element)
{
    for (int i = 0; i < list.size(); i++)
    {
        if ((list.at(i).first == element.first) && (list.at(i).second == element.second))
        {
            return true;
        }
    }
    return false;
}

bool rwa2::Mouse::search_maze(std::pair<int, int> &n)
{
    std::vector<std::pair<int, int>> m_child_node; // holds the child node for one iteration
    m_child_node.clear();                          // Need to clear bc this is local

    // A vector that keeps track of the node being explored and the four nodes around it
    m_child_node.push_back(std::make_pair(n.first, n.second + 1));
    m_child_node.push_back(std::make_pair(n.first + 1, n.second));
    m_child_node.push_back(std::make_pair(n.first, n.second - 1));
    m_child_node.push_back(std::make_pair(n.first - 1, n.second));

    // Checking if current node is goal node
    if (n != m_g)
    {
        if (m_node_stack.empty())
        {
            m_node_stack.push(n);
        }
    }
    else
    {

        log("Solution found");
        return true; // Current node is the goal
    }

    // Check if node has been visited
    if (!find(m_visited_list, n))
    {
        m_visited_list.push_back(n);
    }

    // Checking for walls and if adjacent nodes are visited
    if (!m_maze.at(n.first).at(n.second).is_wall(direction::NORTH) && !find(m_visited_list, m_child_node.at(0)))
    {
        n.second += 1;        // Otherwise, move node up (north direction) for next node to be explored
        m_node_stack.push(n); // Add node that has been moved up to top of stack
    }
    else if (!m_maze.at(n.first).at(n.second).is_wall(direction::EAST) && !find(m_visited_list, m_child_node.at(1)))
    {
        n.first += 1;
        m_node_stack.push(n);
    }
    else if (!m_maze.at(n.first).at(n.second).is_wall(direction::SOUTH) && !find(m_visited_list, m_child_node.at(2)))
    {
        n.second -= 1;
        m_node_stack.push(n);
    }
    else if (!m_maze.at(n.first).at(n.second).is_wall(direction::WEST) && !find(m_visited_list, m_child_node.at(3)))
    {
        n.first -= 1;
        m_node_stack.push(n);
    }
    else
    {
        if (!m_node_stack.empty())
        {
            log("BACKTRACKING");
            m_node_stack.pop();
        }
        else
        {
            log("Path from start to goal node not found");
            return false;
        }
    }

    if (!m_node_stack.empty())
    {
        n = m_node_stack.top();
        search_maze(n);
    }
    else
    {
        log("Path from start to goal node not found");
        return false;
    }
}

void rwa2::Mouse::move_forward()
{
    //Move mouse forward
    log("Move Forward");
    API::moveForward();
}

void rwa2::Mouse::turn_left()
{
    //Turn mouse 90 CCW
    API::turnLeft();
}

void rwa2::Mouse::turn_right()
{
    //Move mouse 90 CW
    API::turnRight();
}

bool rwa2::Mouse::set_maze_walls(const std::pair<int, int> &path_node)
{
    std::vector<char> compass{'n', 'e', 's', 'w'}; // Obtains mouse direction from m_direction attribute
    std::vector<int> temp_dir{};
    std::vector<char> dir;

    if (m_direction == direction::NORTH)
    {
        dir = {compass.at(0), compass.at(3), compass.at(1)}; //'n', 'w', 'e'
        temp_dir.push_back(direction::WEST);
        temp_dir.push_back(direction::EAST);
    }
    else if (m_direction == direction::EAST)
    {
        dir = {compass.at(1), compass.at(0), compass.at(2)}; //'e', 'n', 's'
        temp_dir.push_back(direction::NORTH);
        temp_dir.push_back(direction::SOUTH);
    }
    else if (m_direction == direction::SOUTH)
    {
        dir = {compass.at(2), compass.at(1), compass.at(3)}; //'s', 'e', 'w'
        temp_dir.push_back(direction::EAST);
        temp_dir.push_back(direction::WEST);
    }
    else
    {
        dir = {compass.at(3), compass.at(2), compass.at(0)}; //'w', 's', 'n'
        temp_dir.push_back(direction::SOUTH);
        temp_dir.push_back(direction::NORTH);
    }

    // Check if wall is on the left. Draw wall in simulator and program.
    if (API::wallLeft())
    {
        API::setWall(path_node.first, path_node.second, dir.at(1));
        m_maze.at(path_node.first).at(path_node.second).set_wall(temp_dir.at(0), true);
    }
    // Check if wall is on the right. Draw wall in simulator and program.
    if (API::wallRight())
    {
        API::setWall(path_node.first, path_node.second, dir.at(2));
        m_maze.at(path_node.first).at(path_node.second).set_wall(temp_dir.at(1), true);
    }
    // Check if wall is in front. Draw wall in simulator and program.
    if (API::wallFront())
    {
        API::setWall(path_node.first, path_node.second, dir.at(0));
        if (!m_maze.at(path_node.first).at(path_node.second).is_wall(m_direction))
        { // Add condition to check if wall there bc still can have a wall and program crashes
            log("New Wall Found");
            m_maze.at(path_node.first).at(path_node.second).set_wall(m_direction, true);
            m_node_stack = std::stack<std::pair<int, int>>(); // Empty stack
            m_visited_list.clear();                           // Clear visited list
            return false;
        }
    }

    return true;
}

void rwa2::Mouse::turn_mouse(const std::pair<int, int> &nxt_step)
{

    // Mouse needs to turn to East or West
    if (nxt_step.first != 0)
    {
        //Turn East
        if (nxt_step.first == 1)
        {
            if (m_direction == direction::NORTH)
            {
                turn_right();
            }
            else if (m_direction == direction::SOUTH)
            {
                turn_left();
            }
            else if (m_direction == direction::WEST)
            {
                turn_left();
                turn_left();
            }
            m_direction = direction::EAST;
            log("Facing EAST");
        }
        //Turn West
        else
        {
            if (m_direction == direction::NORTH)
            {
                turn_left();
            }
            else if (m_direction == direction::SOUTH)
            {
                turn_right();
            }
            else if (m_direction == direction::EAST)
            {
                turn_right();
                turn_right();
            }
            m_direction = direction::WEST;
            log("Facing WEST");
        }
    }
    else
    {
        // Mouse needs to turn to North or South
        if (nxt_step.second != 0)
        {
            //Turn North

            if (nxt_step.second == 1)
            {
                if (m_direction == direction::EAST)
                {
                    turn_left();
                    log("TURN NORTH");
                }
                else if (m_direction == direction::WEST)
                {
                    turn_right();
                }
                else if (m_direction == direction::SOUTH)
                {
                    log("TURN AROUND");
                    turn_right();
                    turn_right();
                }
                m_direction = direction::NORTH;
                log("Facing NORTH");
            }
            //Turn South
            else
            {
                if (m_direction == direction::EAST)
                {
                    turn_right();
                }
                else if (m_direction == direction::WEST)
                {
                    turn_left();
                }
                else if (m_direction == direction::NORTH)
                {
                    turn_left();
                    turn_left();
                    log("TURN AROUND");
                }
                m_direction = direction::SOUTH;
                log("Facing SOUTH");
            }
        }
    }
}

void rwa2::Mouse::reverse_stack(std::stack<std::pair<int, int>> &stack)
{
    std::pair<int, int> item;
    std::stack<std::pair<int, int>> tmp_stack;

    while (!stack.empty())
    {
        item = stack.top();
        stack.pop();
        tmp_stack.push(item);
    }
    stack = tmp_stack;
}

void rwa2::Mouse::color_path(const std::stack<std::pair<int, int>> &path)
{
    //tmp_stack is used to color nodes in white in the maze
    auto tmp_stack = path;
    while (!tmp_stack.empty())
    {
        auto item = tmp_stack.top();
        API::setColor(item.first, item.second, 'w');
        tmp_stack.pop();
    }
    API::setColor(m_g.first, m_g.second, 'g'); // Set green otherwise Goal will show white
}
void rwa2::Mouse::draw_path()
{
    std::stack<std::pair<int, int>> final_path = m_node_stack;
    reverse_stack(final_path);
    color_path(final_path);
}

void rwa2::Mouse::follow_path()
{
    bool draw{true};
    log("Follow Path");

    std::stack<std::pair<int, int>> final_path = m_node_stack;
    reverse_stack(final_path);

    API::setColor(m_g.first, m_g.second, 'g'); // Color goal

    while (!final_path.empty())
    {
        std::pair<int, int> next_node{0, 0};
        m_current_node = final_path.top();

        // tmp_node is used to store the last not visited in case
        // m_current_node is emptied in the function set_maze_walls

        draw = set_maze_walls(m_current_node);

        final_path.pop();

        API::setColor(m_current_node.first, m_current_node.second, 'o'); // Color current node orange

        if (m_current_node == m_g)
        {
            log("Solution Found");
            break;
        }
        else if (!draw)
        { // if wall found
            log("Another front wall");
            final_path = std::stack<std::pair<int, int>>(); // clear path
            break;
        }

        // Calculate how to turn mouse
        if (!final_path.empty())
        { // Check bc final_path could be empty
            next_node.first = final_path.top().first - m_current_node.first;
            next_node.second = final_path.top().second - m_current_node.second;
            log("Current node: " + std::to_string(m_current_node.first) + " " + std::to_string(m_current_node.second));
            log("Next node: " + std::to_string(final_path.top().first) + " " + std::to_string(final_path.top().second));
            log("Direction: " + std::to_string(m_direction));
            log("Diff node: " + std::to_string(next_node.first) + " " + std::to_string(next_node.second));
            log("");

            turn_mouse(next_node);

            // Move one step forward
            // Check that there is no wall in front again because after turning, mouse may face a wall.
            if (!API::wallFront())
            {
                move_forward();
                log("Moved Forward. No Wall");
            }
            else
            {
                m_node_stack = std::stack<std::pair<int, int>>(); // Empty Stack
                m_visited_list.clear();                           // Clear visited list
                break;                                            // Search maze again
            }
        }
        else
        {
            break;
        }
    }
}

void rwa2::Mouse::program_flow()
{

    API::setText(m_g.first, m_g.second, "G2");
    // Clearing all tiles in maze simulation
    API::clearAllColor();

    while (search_maze(m_current_node))
    { // DFS
        try
        {
            API::clearAllColor(); // Clear all previous paths
            draw_path();
            follow_path();

            // Check if goal is reached
            if (m_current_node == m_g)
            {
                log("Goal Reached. Congrats!");
                API::setColor(m_g.first, m_g.second, 'g');
                break;
            }
        }
        catch (const char *msg)
        {
            log(msg);
            break;
        }
    }
}
