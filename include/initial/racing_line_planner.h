#ifndef RACING_LINE_PLANNER_H
#define RACING_LINE_PLANNER_H

#include <vector>
#include <cfloat>
#include <CGAL/Simple_cartesian.h>

/* TYPEDEFS */
using Kernel = CGAL::Simple_cartesian<double>;
using Point2 = Kernel::Point_2;

namespace initial {

std::vector<Point2> ComputeRacingLine(
			const std::vector<Point2>& outerPts,
			const std::vector<Point2>& innerPts,
			int N = 500,
			int M = 31,
			double lambda = 0.1
		);


}

#endif // RACING_LINE_PLANNER_H
