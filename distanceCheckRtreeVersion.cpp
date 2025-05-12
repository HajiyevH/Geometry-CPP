#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <vector>
#include <iostream>
#include <algorithm>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

struct sPoint2D {
    float x, y;
    sPoint2D(float xValue, float yValue) : x(xValue), y(yValue) {}
};

typedef bg::model::point<float, 2, bg::cs::cartesian> BoostPoint;
typedef bg::model::box<BoostPoint> BoostBox;
typedef std::pair<BoostBox, size_t> RTreeValue;

const float DISTANCE_THRESHOLD = 1.5f;

float pointToSegmentDistanceSquared(const sPoint2D& p, const sPoint2D& a, const sPoint2D& b)
{
    // find the values of AB segment
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float abSquare = dx * dx + dy * dy;

    // checking if A = B
    if (abSquare == 0.0f) 
        return ((p.x - a.x)*(p.x - a.x) + (p.y - a.y)*(p.y - a.y)); 

    // finding the projection and normalazing it between 0 and 1
    float s = ((p.x - a.x) * dx + (p.y - a.y) * dy) / abSquare;
    if (s < 0.0f) { s = 0.0f;} // if it is less than 0 this means the closest point is a -- projX = a.x + 0.0 * dx so a, same for y
    else if (s > 1.0f) { s = 1.0f; } // if it is less than 0 this means the closest point is b -- projX = a.x + 1.0 * dx so a.x + dx = b.x same for y

    //find the closest point to P in segment
    float projX = a.x + s * dx;
    float projY = a.y + s * dy;

    //find the distance between point and closest point to it in segment
    float distX = p.x - projX;
    float distY = p.y - projY;
    return distX * distX + distY * distY;
}

bool arePolylinesCloserThanThresholdRtree(
    const std::vector<sPoint2D>& polyline1,
    const std::vector<sPoint2D>& polyline2)
{
    bgi::rtree<RTreeValue, bgi::quadratic<16>> rtree;
    for (size_t i = 0; i + 1 < polyline2.size(); ++i) {
        if (polyline2[i].x == polyline2[i+1].x && polyline2[i].y == polyline2[i+1].y)
            continue; 
        BoostBox box(
            BoostPoint(std::min(polyline2[i].x, polyline2[i+1].x), std::min(polyline2[i].y, polyline2[i+1].y)),
            BoostPoint(std::max(polyline2[i].x, polyline2[i+1].x), std::max(polyline2[i].y, polyline2[i+1].y))
        );
        rtree.insert(std::make_pair(box, i));
    }

    float threshold2 = DISTANCE_THRESHOLD * DISTANCE_THRESHOLD;
    for (size_t i = 0; i + 1 < polyline1.size(); ++i) {
        float min_x = std::min(polyline1[i].x, polyline1[i+1].x) - DISTANCE_THRESHOLD;
        float min_y = std::min(polyline1[i].y, polyline1[i+1].y) - DISTANCE_THRESHOLD;
        float max_x = std::max(polyline1[i].x, polyline1[i+1].x) + DISTANCE_THRESHOLD;
        float max_y = std::max(polyline1[i].y, polyline1[i+1].y) + DISTANCE_THRESHOLD;
        BoostBox query_box(BoostPoint(min_x, min_y), BoostPoint(max_x, max_y));

        std::vector<RTreeValue> result_s;
        rtree.query(bgi::intersects(query_box), std::back_inserter(result_s));

        for (const auto& val : result_s) {
            size_t j = val.second;
            if (
                pointToSegmentDistanceSquared(polyline1[i], polyline2[j], polyline2[j+1]) < threshold2 || pointToSegmentDistanceSquared(polyline1[i+1], polyline2[j], polyline2[j+1]) < threshold2 ||
                pointToSegmentDistanceSquared(polyline2[j], polyline1[i], polyline1[i+1]) < threshold2 || pointToSegmentDistanceSquared(polyline2[j+1], polyline1[i], polyline1[i+1]) < threshold2
            ) {
                return true;
            }
        }
    }
    return false;
}

int main() {
    std::vector<sPoint2D> polyline1 { {2.0F, 3.0F}, {3.0F, 4.0F}, {2.0F, 6.0F} };
    std::vector<sPoint2D> polyline2 { {5.0F, 6.0F}, {5.0F, 4.0F}, {7.0F, 4.0F}, {7.0F, 2.0F} };
    std::cout << (arePolylinesCloserThanThresholdRtree(polyline1, polyline2) ? "Close" : "Not Close") << std::endl;
}

