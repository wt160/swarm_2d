#include "pathfinder.hpp" 

namespace simulator::plugin
{

    // PathFinder::PathFinder(void)
    // {
    //     neighbours[0] = Point( -1, -1 ); neighbours[1] = Point(  1, -1 );
    //     neighbours[2] = Point( -1,  1 ); neighbours[3] = Point(  1,  1 );
    //     neighbours[4] = Point(  0, -1 ); neighbours[5] = Point( -1,  0 );
    //     neighbours[6] = Point(  0,  1 ); neighbours[7] = Point(  1,  0 );
    //     cost_so_far = {};
    // }

    // PathFinder::PathFinder(nav_msgs::msg::OccupancyGrid::SharedPtr& map)
    // {
    //     neighbours[0] = Point( -1, -1 ); neighbours[1] = Point(  1, -1 );
    //     neighbours[2] = Point( -1,  1 ); neighbours[3] = Point(  1,  1 );
    //     neighbours[4] = Point(  0, -1 ); neighbours[5] = Point( -1,  0 );
    //     neighbours[6] = Point(  0,  1 ); neighbours[7] = Point(  1,  0 );
    //     m.importFromNavOccupancyGrid(map);
    // }


    void PathFinder::init(nav_msgs::msg::OccupancyGrid::SharedPtr& map)
    {
        neighbours[0] = Point( -1, -1 ); neighbours[1] = Point(  1, -1 );
        neighbours[2] = Point( -1,  1 ); neighbours[3] = Point(  1,  1 );
        neighbours[4] = Point(  0, -1 ); neighbours[5] = Point( -1,  0 );
        neighbours[6] = Point(  0,  1 ); neighbours[7] = Point(  1,  0 );
        m.importFromNavOccupancyGrid(map);
    }

    double PathFinder::calcH(Point & p)
    {
        //diagonal distance heuristic
        double dx = std::fabs(end.x - p.x);
        double dy = std::fabs(end.y - p.y);
        return dx + dy + (1.414 - 2)*std::min(dx, dy);

    }

    bool PathFinder::isValid(Point & p)
    {
        return ( p.x >-1 && p.y > -1 && p.x < m.width && p.y < m.height );
    }

    bool PathFinder::isExistPoint(Node& p, double g_cost)
    {
        if(cost_so_far.find(p) == cost_so_far.end() || g_cost < cost_so_far[p]){
            return false;
        }else{
            return true;
        }

    }

    bool PathFinder::fillOpen(Node& n)
    {
        double stepCost, nc, dist;
        Point neighbour;
 
        for( int x = 0; x < 8; x++ ) {
            // one can make diagonals have different cost
            stepCost = x < 4 ? 1 : 1.414;
            neighbour = n.pos + neighbours[x];
            if( neighbour == end ){ 
                parent_map[make_pair(neighbour.x, neighbour.y)] = n;
                return true;
            }
 
            if( isValid( neighbour ) && m( neighbour.x, neighbour.y ) != 100 ) {
                nc = stepCost + n.g_cost;
                Node neigh;
                neigh.pos = neighbour;
                if( !isExistPoint( neigh, nc ) ) {        
                    neigh.g_cost = nc; 
                    neigh.f_cost = /*calcH( neighbour )*/ + neigh.g_cost;
                    parent_map[make_pair(neigh.pos.x, neigh.pos.y)] = n;
                    cost_so_far[neigh] = neigh.g_cost;
                    open.put(neigh, neigh.f_cost );
                }
            }
        }
        return false;
    }


    bool PathFinder::search(Point& s, Point& e)
    {
        Node null_n;
        Point null_n_point(-1, -1);
        null_n.pos = null_n_point;
        Node n; end = e; start = s;
        n.g_cost = 0.0; n.pos = s; 
        parent_map[make_pair(n.pos.x, n.pos.y)] = null_n;
        n.f_cost = n.g_cost + calcH( s );
        open.put( n, n.f_cost );
        cost_so_far[n] = n.g_cost;
        while( !open.empty() ) {
            Node n = open.get();
            std::cout<<"("<<n.pos.x<<","<<n.pos.y<<")"<<std::endl;
            // closed.push_back( n );
            if( fillOpen( n ) ) return true;
        }
        return false;
    }

    double PathFinder::getPathAndCost( std::list<Point>& path ) {
        path.push_front( end );
        double path_cost = 0.0;
        Point curr_pt = end;
        while(curr_pt.x != -1 && curr_pt.y != -1){
            path.push_front(curr_pt);
            std::cout<<"pushing "<<curr_pt.x<<","<<curr_pt.y<<std::endl;
            Node parent_node = parent_map[make_pair(curr_pt.x, curr_pt.y)];
            Point parent_pt = parent_node.pos;
            if(parent_pt.x == curr_pt.x || parent_pt.y == curr_pt.y){
                path_cost += 1.0;
            }else{
                path_cost += 1.414;
            }
            curr_pt = parent_pt;
        }

        return path_cost;
    }

    void Map::importFromNavOccupancyGrid(nav_msgs::msg::OccupancyGrid::SharedPtr& imported_map)
    {
        data = imported_map->data;
        width = imported_map->info.width;
        height = imported_map->info.height;
    }


}
