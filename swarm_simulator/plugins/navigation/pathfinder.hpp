#pragma once
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <unordered_map>
#include <cmath>
#include <queue>
#include <list>
#include <map>
#include <algorithm>
#include <functional>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <iostream>


namespace simulator::plugin
{ 
    using namespace std;

    template<typename T, typename priority_t>
    struct PriorityQueue 
    {    
        typedef std::pair<priority_t, T> PQElement;
        struct CustomCompare
        {
            bool operator()(const PQElement& lhs, const PQElement& rhs)
            {
                if(lhs.first > rhs.first){return true;}
                else{
                    return false;
                }
            }
        };

        std::priority_queue<PQElement, std::vector<PQElement>,
        CustomCompare> elements;

        inline bool empty() const {
            return elements.empty();
        }

        inline void put(T item, priority_t priority) {
            elements.emplace(priority, item);
        }

        T get() {
            T best_item = elements.top().second;
            elements.pop();
            return best_item;
        }

    };


    class Point
    {
        public:
            Point( int a = 0, int b = 0 ) { x = a; y = b; }
            bool operator ==( const Point& o ) { return o.x == x && o.y == y; }
            Point operator +( const Point& o ) { return Point( o.x + x, o.y + y ); }
            int x, y;
    };

    class Node
    {
        public:
            bool operator == (const Node o ) { return pos == o.pos; }
            // bool operator == (const Point& o ) { return pos == o; }
            bool operator < (const Node& o ) { return f_cost + g_cost < o.f_cost + o.g_cost; }
            Point pos;
            // Node parent;
            double f_cost, g_cost;
    };

    class Map
    {
        public:
            int width;
            int height;

            vector<signed char> data;

            void importFromNavOccupancyGrid(nav_msgs::msg::OccupancyGrid::SharedPtr&);
            signed char operator()(int x, int y){ return data[y*width + x];}

    };

    struct NodeHasher
    {
        std::size_t operator () (const Node &key) const 
        {
            // The following line is a stright forward implementation. But it can be
            // hard to design a good hash function if KeyData is complex.

            //return (key.id << 32 | key.age); // suppose size_t is 64-bit and int is 32-bit

            // A commonly used way is to use boost
            std::size_t seed = 0;
            seed = (53 + key.pos.x) * 53 + key.pos.y;
            return seed;
        }
    };
    
    struct NodeEqual 
    {
        bool operator()(const Node& lhs, const Node& rhs) const
        {
            return lhs.pos.x == rhs.pos.x && lhs.pos.y == rhs.pos.y;
        }
    };

    class PathFinder
    {
        public:
            // PathFinder(void);
            // PathFinder(nav_msgs::msg::OccupancyGrid::SharedPtr& map);
            void init(nav_msgs::msg::OccupancyGrid::SharedPtr& map);

            double calcH(Point &p);
            bool isValid(Point &p);
            bool isExistPoint(Node &n, double f_cost);
            bool fillOpen(Node& n);
            bool search(Point& start, Point& end);
            double getPathAndCost(std::list<Point>& path);
            // void setStart(Point);
            // void setEnd(Point);

        private:
            Map m; 
            Point end, start;
            Point neighbours[8];
            PriorityQueue<Node, double> open;
            unordered_map<Node, double, NodeHasher, NodeEqual> cost_so_far;
            map<pair<int, int>, Node> parent_map;
    };

}