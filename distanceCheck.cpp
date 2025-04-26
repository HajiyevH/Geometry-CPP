#include <iostream>
#include <vector>


const float DISTANCE_THRESHOLD = 1.5f;
struct sPoint2D
{
sPoint2D(float xValue, float yValue)
    {
    x = xValue;
    y = yValue;
    }
float x;
float y;
};


// Function to calculate distance between singular point and segment
float pointToSegmentDistance(const sPoint2D& p, const sPoint2D& a, const sPoint2D& b)
{
    // find the values of AB segment
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float abSquare = dx * dx + dy * dy;

    // checking if A = B
    if (abSquare == 0.0f) 
        return std::sqrt((p.x - a.x)*(p.x - a.x) + (p.y - a.y)*(p.y - a.y)); 

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
    return std::sqrt(distX * distX + distY * distY);
}

// Function to check if any points on one polyline are closer to any segments on the other polyline done in a simple and BruteForce way
bool arePolylinesCloserThanThresholdBrute(std::vector<sPoint2D>& polyline1,
                                          std::vector<sPoint2D>& polyline2)
{

    for(int p1 = 0; p1+1 < polyline1.size();p1++)
    {
        for(int p2 = 0; p2+1 < polyline2.size();p2++)
        {
            // checking if point on polyline1 is closer to segment on polyline2
            if((pointToSegmentDistance(polyline1[p1],polyline2[p2],polyline2[p2+1]) < DISTANCE_THRESHOLD || pointToSegmentDistance(polyline1[p1+1],polyline2[p2],polyline2[p2+1]) < DISTANCE_THRESHOLD)||
                // checking if point on polyline2 is closer to segment on polyline1
                (pointToSegmentDistance(polyline1[p2],polyline2[p1],polyline2[p1+1]) < DISTANCE_THRESHOLD || pointToSegmentDistance(polyline1[p2+1],polyline2[p1],polyline2[p1+1]) < DISTANCE_THRESHOLD))
                return true; // early exit if any pair is closer than threshold
        }
    }

    return false;

}

// Struct to hold our BoundingBoxes
struct BoundingBox {
    float minX, maxX, minY, maxY;
};

// Function to get boundingbox of two points
BoundingBox getBoundingBox(const sPoint2D& a, const sPoint2D& b) {
    BoundingBox box;
    box.minX = std::min(a.x, b.x);
    box.maxX = std::max(a.x, b.x);
    box.minY = std::min(a.y, b.y);
    box.maxY = std::max(a.y, b.y);
    return box;
}

int main() {
    std::vector<sPoint2D> polyline1 {
        {2.0F,3.0F}, {3.0F,4.0F}, {2.0F,6.0F}
    };
    std::vector<sPoint2D> polyline2 {
        {5.0F,6.0F}, {5.0F,4.0F}, {7.0F,4.0F}, {7.0F,2.0F}
    };

    bool close = arePolylinesCloserThanThresholdBrute(polyline1, polyline2);
    std::cout << (close ? "true\n" : "false\n");  
    return 0;
}