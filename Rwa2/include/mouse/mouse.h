/**
 * @file mouse.h
 * @author Hae Lee Kim, Yoseph Kebede, Mohammed Baaqeel
 * @brief The file contains the Mouse class
 * @version 0.1
 * @date 2021-11-12
 *
 * @copyright Copyright (c) 2021
 *
 */

/*! \mainpage Maze search algorithm
  *
  * This project consists of searching a path in a maze
  * and then task a mouse (robot) to follow the path.
  * - \subpage searchingPathPage "Searching a path"
  *
  * - \subpage followingPathPage "Following a path"
  *
  */

/*! \page searchingPathPage Searching a path
   *
   * The search algorithm used for searching a path in a maze relies on
   * the depth-first search (DFS) approach. This algorithm is implemented in rwa2::Mouse::search_maze()
   *
   */

/*! \page followingPathPage Following a path
    *
    * To follow a path generated by DFS, methods from the class API (api/api.h) must be used to interact
    * with the micromouse simulator.
    * - Methods of the API class are documented <a href="https://github.com/mackorone/mms#summary">here</a>.
    */

#ifndef MICROMOUSE_H
#define MICROMOUSE_H

#include "../node/node.h"
#include "../util/util.h"
#include <array>
#include <stack>
#include <string>
#include <vector>
#include <utility>

namespace rwa2
{
    /**
     * @brief This class is used to compute a path and execute the path.
     *
     */
    class Mouse
    {
    public:
        /**
         * @brief Construct a new MicroMouse object
         *
         * The robot is always at (0,0) and facing NORTH when the simulator starts
         * The goal node is initialized in the mouse ctor
         * The body of the ctor sets boundary walls in the internal maze
         */
        Mouse() : m_current_node{0, 0}, m_direction{direction::NORTH}, m_g{7, 8}
        {
            
            for (int x = 0; x < m_maze_width; x += 1)
            {
                for (int y = 0; y < m_maze_height; y += 1)
                {
                    m_maze.at(x).at(y).set_wall(direction::NORTH, (y == m_maze_height - 1));
                    m_maze.at(x).at(y).set_wall(direction::EAST, (x == m_maze_width - 1));
                    m_maze.at(x).at(y).set_wall(direction::SOUTH, (y == 0));
                    m_maze.at(x).at(y).set_wall(direction::WEST, (x == 0));
                }
            }
        }
        /**
         * @brief Print string onto API
         * 
         * @param text String being inputted to be displayed.
         */
        void log(const std::string &text);

        /**
         * @brief Visually set the walls in the simulator
         * 
         */
        void display_walls();

        /**
         * @brief Implement DFS to compute a path between current node and goal node in a maze
         * @param n current node for search, an ordered pair (x, y)
         *
         * @return true A path is found
         * @return false A path is not found
         */
        bool search_maze(std::pair<int, int> &n);

        /**
         * @brief Make the mouse move forward
         *
         */
        void move_forward();

        /**
         * @brief Make the mouse rotate 90 deg CCW
         *
         */
        void turn_left();

        /**
         * @brief Make the mouse rotate 90 deg CW
         *
         */
        void turn_right();

        /**
         * @brief Implement Algorithm 2 to move mouse in the simulator
         * 
         */
        void program_flow();

        /**
         * @brief Search for an element in a list, stack or vector
         * 
         * @param list the 1D stack or vector being searched
         * @param element the variable, ordered pair in search
         *
         * @return true if that element exists
         * @return false if the element does not exist
         *   
         */
        bool find(const std::vector<std::pair<int, int>> &list, const std::pair<int, int> &element);

        /**
         * @brief Draws wall on the simulation and stores 
         *    wall information from the simulation to the maze program 
         * 
         * @param path_node an ordered pair holding values of current nodes
         * @return true if the method finishes without mouse hitting a wall
         * @return false if new wall exists
         */
        bool set_maze_walls(const std::pair<int, int> &path_node);

        /**
         * @brief Turns the mouse to face the next tile it's going to step on 
         *  
         * @param nxt_step Ordered pair which is the difference of current node 
         *   with the next node in the final path
         * 
         */
        void turn_mouse(const std::pair<int, int> &nxt_step);

        /**
         * @brief Take found path and move mouse. If wall is detected, re-search maze and set appropriate walls.
         * 
         */
        void follow_path();

        /**
         * @brief Flip the order of (x,y) nodes in solution stack
         * 
         * @param stack 
         */
        void reverse_stack(std::stack<std::pair<int, int>> &stack);

        /**
         * @brief Draws color of the final path of the mouse on the 
         * simulation using reverse_stack and color path functions
         * 
         */
        void draw_path();

        /**
         * @brief color found path in white and goal node in green
         * 
         * @param path being inputted to color route accordingly 
         */
        void color_path(const std::stack<std::pair<int, int>> &path);

    private:
        /** private data member to store width of the maze */
        static const int m_maze_width{16};
        /** a private data member to store height of the maze */
        static const int m_maze_height{16};
        /** a private data member to store direction of the mouse*/
        int m_direction;
        /** 2D array maze object */
        std::array<std::array<Node, m_maze_width>, m_maze_height> m_maze;
        /** a stack member to store all nodes to be explored */
        std::stack<std::pair<int, int>> m_node_stack;
        /** a vector member to store all visited nodes */
        std::vector<std::pair<int, int>> m_visited_list;
        /** a pair variable to store current node of mouse */
        std::pair<int, int> m_current_node;
        /** a pair variable to store final goal node */
        std::pair<int, int> m_g;
    };
}
#endif
