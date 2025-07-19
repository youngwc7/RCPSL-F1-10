#include <ros/console.h>
#include <algorithm> 
#include <CGAL/centroid.h>
#include <geometry_msgs/PoseStamped.h>
#include "initial/geometry_path.h"

using namespace initial;

GeometryPath::GeometryPath(ros::NodeHandle& nh)
{
	ros::NodeHandle privateNodeHandle("~");
	/* padding parameter, default 0.0*/
	privateNodeHandle.param("car_width", margin, 0.0);
	
	mapSub = nh.subscribe("map", 1, &GeometryPath::MapCallback, this);
	pathPub = nh.advertise<nav_msgs::Path>("geometry_path", 1, true);
}

void GeometryPath::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	/* Create a binary map */
	int w = msg->info.width, h = msg->info.height;
	if (w == 0 || h == 0 || msg->data.empty())
	{
		ROS_INFO("NO DATA FROM OCCUPANCY_GRID! (w: %d, h: %d)", w, h);
		return;
	}
	ROS_INFO("MAPCALLBACK: %d, %d, res %f", w, h, msg->info.resolution);
	cv::Mat binMap = cv::Mat::zeros(h, w, CV_8UC1);
	
	if (!binMap.data)
	{
		ROS_INFO ("CV BIN_MAP FAILED!");
		return;
	}
	for (int y = 0; y < h; ++y) {
	    for (int x = 0; x < w; ++x) {
		int idx = y * w + x;
		binMap.at<uchar>(y,x) = (msg->data[idx] == 0 ? 255 : 0);
	    }
	}
	
	/* Find largest contour */
	std::vector<std::vector<cv::Point> > contours;
	int maxContourIndex = -1;
	
	if (!FindContours(contours, binMap, maxContourIndex)) return;
	
	// std::vector<cv::Point> simplified;
	// cv::approxPolyDP(maxContour, simplified, 2.0, true);
	// if (cv::contourArea(simplified) < 1e-4)
	// {
	// 	ROS_INFO("CONTOUR TINY!");;
	// 	return;
	// }
	
	/* Build polygon from contour */
	Poly2 outerPoly;
	std::vector<Poly2> holes;
	
	if (!BuildCGALPoly(outerPoly, holes, msg, contours, maxContourIndex)) return;
	
	/* create skeleton */
	// SCGAL_Ptr ss = CGAL::create_interior_straight_skeleton_2(poly.vertices_begin(), poly.vertices_end());
	// if (!ss) return;
	
	// BuildSkeleton(ss, vertices);
	// std::vector<Point2> vertices = ExtractLongestSkeletonPath(ss);
	
	
	ROS_INFO("OUTER POLY: %zu vertices. First: (%.2f, %.2f)", outerPoly.size(), outerPoly[0].x(), outerPoly[0].y());
	/* Generate path */
	std::vector<Point2> pathWaypoints;
	
	BuildCDTPath(pathWaypoints, outerPoly, holes);
	if (pathWaypoints.empty())
	{
		ROS_INFO("no path!");
		return;
	}
	
	/* Publish Path msg */
	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "map";
	
	for (auto &p : pathWaypoints)
	{
		geometry_msgs::PoseStamped poseStamped;
		poseStamped.header = path.header;
		poseStamped.pose.position.x = p.x();
		poseStamped.pose.position.y = p.y();
		poseStamped.pose.position.z = 0.0;
		
		poseStamped.pose.orientation.x = 0.0;
		poseStamped.pose.orientation.y = 0.0;
		poseStamped.pose.orientation.z = 0.0;
		poseStamped.pose.orientation.w = 1.0;
		path.poses.push_back(poseStamped);
	}
	
	pathPub.publish(path);
}

bool GeometryPath::FindContours(std::vector<std::vector<cv::Point> >& contours, const cv::Mat& binMap, int& maxContourIndex)
{
	if (binMap.empty())
	{
		ROS_INFO("empty bin map in maxContour");
		return false;
	}
	
	cv::findContours(binMap, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	if (contours.empty()) 
	{
		ROS_INFO ("NO CONTOURS!"); 
		return false;
	}
	
	auto maxContourIt = std::max_element(contours.begin(), contours.end(),
				[](const auto& a, const auto& b) 
				{ return std::fabs(cv::contourArea(a)) < std::fabs(cv::contourArea(b)); });
	
	maxContourIndex = static_cast<int>(maxContourIt - contours.begin());
	return true;
}
	
bool GeometryPath::BuildCGALPoly(Poly2& outer, std::vector<Poly2>& holes, const nav_msgs::OccupancyGrid::ConstPtr& msg, const std::vector<std::vector<cv::Point> >& contours, int maxContourIndex)
{
	double res = msg->info.resolution;
	double origX = msg->info.origin.position.x;
	double origY = msg->info.origin.position.y;
	
	outer.clear();
	holes.clear();
	
	int contourIndex = 0;
	
	for (const auto& contour : contours) 
	{
		// only allow triangle capable stuff
		if (contour.size() < 3) 
		{
			++contourIndex;
			continue;
		}
		
		Poly2 tmpPoly;
		Point2 prevPoint;
		bool firstIter = true;
		
		/* build polygon out of cv contour */
		for (const auto& point : contour)
		{
			double toWorldX = origX + (point.x + 0.5) * res;
			double toWorldY = origY + (point.y + 0.5) * res;
			Point2 currPoint(toWorldX, toWorldY);
			
			// only add non-duplicates
			if (firstIter || currPoint != prevPoint)
				tmpPoly.push_back(currPoint);
			
			prevPoint = currPoint;
			firstIter = false;
		}
		
		if (tmpPoly.size() < 3) 
		{
			++contourIndex;
			continue;
		}  
		if (*(tmpPoly.vertices_begin()) == *(--tmpPoly.vertices_end()))
		{
			tmpPoly.erase(--tmpPoly.vertices_end());
		}
		
		// convert to simple polygon
		if (!CGAL::is_simple_2(tmpPoly.vertices_begin(), tmpPoly.vertices_end()))
		{
			ROS_INFO("Not simple poly! Iteration %d", contourIndex);
			++contourIndex;
 			return false;
		}
	

		// outer polygon set to CCW
		if (contourIndex == maxContourIndex) 
		{
			if (!tmpPoly.is_counterclockwise_oriented())
			{
				tmpPoly.reverse_orientation();
			}
			outer = std::move(tmpPoly);
		}
		else
		{
			// holes set to CW
			if (tmpPoly.is_counterclockwise_oriented())
			{
				tmpPoly.reverse_orientation();				
			}
			holes.push_back(std::move(tmpPoly));
		}
		++contourIndex;
	}
	return true;
}

void GeometryPath::BuildCDTPath(std::vector<Point2>& path, const Poly2& outerPoly, const std::vector<Poly2>& holes)
{
	path.clear();
	if (outerPoly.size() < 3)
	{
		ROS_INFO("WARNING: INVALID OUTER WALL");
		return;
	}
	
	if (!CGAL::is_simple_2(outerPoly.vertices_begin(), outerPoly.vertices_end()))
	{
		ROS_INFO("skip, outer wall not simple");
		return;
	}
	
	/* DELAUNAY */
	CDT cdt;
	
	/* define outer wall constraint */
	cdt.insert_constraint(outerPoly.vertices_begin(), outerPoly.vertices_end(), true);
	
	/* define inner wall constraint */
	for (const auto& hole : holes)
	{
		if (hole.size() >= 3)
			cdt.insert_constraint(hole.vertices_begin(), hole.vertices_end(), true);
		else 
		{
			ROS_INFO("INVALID HOLE");
			return;
		}
		if (!CGAL::is_simple_2(hole.vertices_begin(), hole.vertices_end()))
		{
			ROS_INFO("non simple HOLE");
			return;
		}
			
	}
	
	// CGAL::mark_domain_in_triangulation(cdt);
	
	/* graph connecting traingle centers -> Each node (face) has a std::set of neighboring faces */
	std::map<CDT::Face_handle, std::set<CDT::Face_handle>> graph; 
	
	/* define triangle center points */
	std::map<CDT::Face_handle, Point2> centers;
	
	/* iterating through triangles */
	for (auto face = cdt.finite_faces_begin(); face != cdt.finite_faces_end(); ++face)
	{
		// vertices of triangle face 
		Point2 a = face->vertex(0)->point();
		Point2 b = face->vertex(1)->point();
		Point2 c = face->vertex(2)->point();
		
		Point2 center = CGAL::centroid(a, b, c);
		
		// test if point is within outer wall
		if (outerPoly.bounded_side(center) != CGAL::ON_BOUNDED_SIDE)
			continue;
		
		// test if point is outside all inner walls
		bool in_hole = false;
		for (auto const& hole : holes)
		{
			if (hole.bounded_side(center) != CGAL::ON_UNBOUNDED_SIDE)
			{
				in_hole = true;
				break;
			}
		}
		if (in_hole) continue;
		
		
		// successfully passed both conditions
		centers[face] = center; // add to the 'centers' dictionary
		
	}
	
	// Create graph of valid edges
	for (const auto& center : centers)
	{
		CDT::Face_handle face = center.first;
		for (int i = 0; i < 3; ++i)
		{
			auto neighbor = face->neighbor(i);
			if (centers.count(neighbor))
			{
				graph[face].insert(neighbor);
			}
		}
	}
	
	// Convert graph to path 
	for (const auto& g : graph)
	{
		auto face = g.first;
		const auto& neighbors = g.second;
		
		for (const auto& n : neighbors)
		{
			path.push_back(centers.at(face));
			path.push_back(centers.at(n));
		}
	}
}

// void GeometryPath::BuildSkeleton(SCGAL_Ptr& ss, std::set<Point2>& vertices)
// {
// 	for (auto h = ss->halfedges_begin(); h != ss->halfedges_end(); ++h)
//	{
//		if (h->is_bisector())
//		{
//			vertices.insert(h->vertex()->point());
//			vertices.insert(h->opposite()->vertex()->point());
//		}
//	}
// }

/*
 NO LONGER IN USE
static std::vector<Point2> ExtractLongestSkeletonPath(SCGAL_Ptr& ss)
{
    std::map<Point2, std::vector<Point2>> graph;
    std::set<Point2> vertices;    for (auto h = ss->halfedges_begin(); h != ss->halfedges_end(); ++h)
    {
        if (!h->is_bisector()) continue;        Point2 src = h->vertex()->point();
        Point2 dst = h->opposite()->vertex()->point();        graph[src].push_back(dst);
        graph[dst].push_back(src);        vertices.insert(src);
        vertices.insert(dst);
    }    if (vertices.empty())
        return {};    // First BFS to find farthest point from an arbitrary root
    std::map<Point2, int> dist;
    std::map<Point2, Point2> parent;
    std::set<Point2> visited;    Point2 start = *vertices.begin();
    std::queue<Point2> q;
    q.push(start);
    visited.insert(start);
    dist[start] = 0;    Point2 farthest = start;    while (!q.empty())
    {
        Point2 u = q.front(); q.pop();
        for (const auto& v : graph[u])
        {
            if (visited.count(v)) continue;
            visited.insert(v);
            parent[v] = u;
            dist[v] = dist[u] + 1;
            if (dist[v] > dist[farthest])
                farthest = v;
            q.push(v);
        }
    }    // Second BFS from farthest point to get the longest path
    parent.clear();
    dist.clear();
    visited.clear();    Point2 root = farthest;
    q.push(root);
    visited.insert(root);
    dist[root] = 0;    Point2 end = root;    while (!q.empty())
    {
        Point2 u = q.front(); q.pop();
        for (const auto& v : graph[u])
        {
            if (visited.count(v)) continue;
            visited.insert(v);
            parent[v] = u;
            dist[v] = dist[u] + 1;
            if (dist[v] > dist[end])
                end = v;
            q.push(v);
        }
    }    // Reconstruct path from end to root
    std::vector<Point2> path;
    for (Point2 p = end; p != root; p = parent[p])
        path.push_back(p);
    path.push_back(root);
    std::reverse(path.begin(), path.end());
    return path;
}

*/


