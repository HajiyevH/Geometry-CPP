#include <iostream>
#include <cmath>      
#include <algorithm>    
#include <vector>
#include <unordered_map>
#include <utility>    
#include <functional>   

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

struct hash_pair {
    size_t operator()(const std::pair<int, int>& p) const
    {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};
// Function to calculate distance between singular point and segment
float pointToSegmentDistanceSquared(const sPoint2D& p, const sPoint2D& a, const sPoint2D& b)
{
    // find the values of AB segment
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float abSquare = dx * dx + dy * dy;

    // checking if A = B
    if (abSquare == 0.0f) 
        return ((p.x - b.x)*(p.x - b.x) + (p.y - b.y)*(p.y - b.y)); 

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

// 2D cross product
float cross(const sPoint2D& a, const sPoint2D& b) {
    return a.x * b.y - a.y * b.x;
}

// this functions gets the orientation of three points in the space (i got this from this handbook https://github.com/vlecomte/cp-geo)
float orient(const sPoint2D& a, const sPoint2D& b, const sPoint2D& c) {
    return cross({b.x - a.x, b.y - a.y}, {c.x - a.x, c.y - a.y});
}

// checking if they intersect
bool segmentsProperlyIntersect(const sPoint2D& a, const sPoint2D& b, const sPoint2D& c, const sPoint2D& d) {
    float oa = orient(c, d, a);
    float ob = orient(c, d, b);
    float oc = orient(a, b, c);
    float od = orient(a, b, d);
    return (oa * ob < 0) && (oc * od < 0);
}

// =====================================================================================================================================

// Function to check if any points on one polyline are closer to any segments on the other polyline done in a simple and BruteForce way
bool arePolylinesCloserThanThresholdBrute(std::vector<sPoint2D>& polyline1,std::vector<sPoint2D>& polyline2)
{
    const float thresholdSquared = DISTANCE_THRESHOLD * DISTANCE_THRESHOLD;

    for(int p1 = 0; p1+1 < polyline1.size();p1++)
    {
        for(int p2 = 0; p2+1 < polyline2.size();p2++)
        {
            if (segmentsProperlyIntersect(polyline1[p1], polyline1[p1+1], polyline2[p2], polyline2[p2+1])) {
                return true;
            }
            // checking if point on polyline1 is closer to segment on polyline2
            if((pointToSegmentDistanceSquared(polyline1[p1],polyline2[p2],polyline2[p2+1]) < thresholdSquared || pointToSegmentDistanceSquared(polyline1[p1+1],polyline2[p2],polyline2[p2+1]) < thresholdSquared)||
                // checking if point on polyline2 is closer to segment on polyline1
                (pointToSegmentDistanceSquared(polyline2[p2],polyline1[p1],polyline1[p1+1]) < thresholdSquared || pointToSegmentDistanceSquared(polyline2[p2+1],polyline1[p1],polyline1[p1+1]) < thresholdSquared))
                return true; // early exit if any pair is closer than threshold
        }
    }
    return false;
}

// =====================================================================================================================================

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

// Function to check if they are closer than Threshold
bool boundingBoxesCloserThanThreshold(const BoundingBox& box1, const BoundingBox& box2) {
    // float dx = std::max(0.0f, std::max(box1.minX - box2.maxX, box2.minX - box1.maxX));
    // float dy = std::max(0.0f, std::max(box1.minY - box2.maxY, box2.minY - box1.maxY));
    // return (dx * dx + dy * dy) < (DISTANCE_THRESHOLD * DISTANCE_THRESHOLD);

    // Checking if expaded box (box1) is overlapping the box2 
    if (box1.maxX + DISTANCE_THRESHOLD < box2.minX || box1.minX - DISTANCE_THRESHOLD > box2.maxX ||
        box1.maxY + DISTANCE_THRESHOLD < box2.minY || box1.minY - DISTANCE_THRESHOLD > box2.maxY)
    {
        return false; // No overlap so not closer
    }
    return true;
}

// Function to check if any points on one polyline are closer to any segments on the other polyline done in a simple way with the help of bounding boxes
bool arePolylinesCloserThanThresholdBoundingBox(std::vector<sPoint2D>& polyline1,std::vector<sPoint2D>& polyline2)
{
    float thresholdSquared = DISTANCE_THRESHOLD * DISTANCE_THRESHOLD;

    for(int p1 = 0; p1+1 < polyline1.size();p1++)
    {
        BoundingBox box1 = getBoundingBox(polyline1[p1], polyline1[p1+1]);
        for(int p2 = 0; p2+1 < polyline2.size();p2++)
        {
            // creating the new boxes for the points we have
            BoundingBox box2 = getBoundingBox(polyline2[p2], polyline2[p2+1]);

            // only checking the distance if they are close enough
            if (boundingBoxesCloserThanThreshold(box1, box2))
                if (segmentsProperlyIntersect(polyline1[p1], polyline1[p1+1], polyline2[p2], polyline2[p2+1])) {
                    return true;
                }
                // checking if point on polyline1 is closer to segment on polyline2
                if((pointToSegmentDistanceSquared(polyline1[p1],polyline2[p2],polyline2[p2+1]) < thresholdSquared || pointToSegmentDistanceSquared(polyline1[p1+1],polyline2[p2],polyline2[p2+1]) < thresholdSquared)||
                    // checking if point on polyline2 is closer to segment on polyline1
                    (pointToSegmentDistanceSquared(polyline2[p2],polyline1[p1],polyline1[p1+1]) < thresholdSquared || pointToSegmentDistanceSquared(polyline2[p2+1],polyline1[p1],polyline1[p1+1]) < thresholdSquared))
                    return true; // early exit if any pair is closer than threshold
        }
    }
    return false;
}

// =====================================================================================================================================

//Function to check the distance between two neighbouring cells
bool checkCellPair(const std::vector<std::pair<int, bool>> seg1,const std::vector<std::pair<int, bool>> seg2,
    const std::vector<sPoint2D>& polyline1, const std::vector<sPoint2D>& polyline2,  const float thresholdSquared)
    {
        // iterating through all segments in the first cell
        for(const std::pair<int, bool> seg1Pair : seg1) {
            // iterating through all segments in the second cell
            for(const std::pair<int, bool> seg2Pair : seg2) {
                // skip if both segments are from the same polyline
                if(seg1Pair.second == seg2Pair.second) continue;
                
                //getting the data of the segments we need to check
                int seg1Idx = seg1Pair.first;
                int seg2Idx = seg2Pair.first;
                bool seg1IsPoly1 = seg1Pair.second; 
    
                const sPoint2D& p1a = seg1IsPoly1 ? polyline1[seg1Idx]   : polyline2[seg1Idx];
                const sPoint2D& p1b = seg1IsPoly1 ? polyline1[seg1Idx+1] : polyline2[seg1Idx+1];
                const sPoint2D& p2a = seg1IsPoly1 ? polyline2[seg2Idx]   : polyline1[seg2Idx];
                const sPoint2D& p2b = seg1IsPoly1 ? polyline2[seg2Idx+1] : polyline1[seg2Idx+1];

                BoundingBox box1 = getBoundingBox(p1a, p1b);
                BoundingBox box2 = getBoundingBox(p2a, p2b);
    
                if (boundingBoxesCloserThanThreshold(box1, box2)) {
                    if (segmentsProperlyIntersect(p1a, p1b, p2a, p2b)) {
                        return true;
                    }
                    if ((pointToSegmentDistanceSquared(p1a, p2a, p2b) < thresholdSquared || pointToSegmentDistanceSquared(p1b, p2a, p2b) < thresholdSquared) ||
                        (pointToSegmentDistanceSquared(p2a, p1a, p1b) < thresholdSquared || pointToSegmentDistanceSquared(p2b, p1a, p1b) < thresholdSquared))
                    {
                        return true; 
                    }
                }
            }
        }
        return false;
    }

// The final most efficent function to check if distance between two polylines is less than threshold
bool arePolylinesCloserThanThreshold(std::vector<sPoint2D>& polyline1, std::vector<sPoint2D>& polyline2)
{
    const float thresholdSquared = DISTANCE_THRESHOLD * DISTANCE_THRESHOLD;

    // find the bounds of the grid for spatial grid implementation
    float minX = polyline1[0].x,minY = polyline1[0].y,maxX=polyline1[0].x,maxY=polyline1[0].y;

    for(int i = 0; i < polyline1.size();i++)
    {
        minX = std::min(minX,polyline1[i].x);
        minY = std::min(minY,polyline1[i].y);
        maxX = std::max(maxX,polyline1[i].x);
        maxY = std::max(maxY,polyline1[i].y);
    }
    for(int i = 0; i < polyline2.size();i++)
    {
        minX = std::min(minX,polyline2[i].x);
        minY = std::min(minY,polyline2[i].y);
        maxX = std::max(maxX,polyline2[i].x);
        maxY = std::max(maxY,polyline2[i].y);
    }


    // adding a small padding in order to stop all the error that might occur
    minX -= DISTANCE_THRESHOLD; minY -= DISTANCE_THRESHOLD;
    maxX += DISTANCE_THRESHOLD; maxY += DISTANCE_THRESHOLD;

    // setting the one cellSize to Threshold
    const float cellSize = DISTANCE_THRESHOLD;

    // getting the height of the grid we will create
    const int gridHeight = std::ceil((maxY-minY)/cellSize);
    const int gridWidth = std::ceil((maxX-minX)/cellSize);

    // we are instantiating our grid as a map and in inside vector we will store all the points that fall into that cell
    // bool -> true = polyline1 false = polyline2 for stoping distance check between points in same polylines
    std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, bool>>, hash_pair> grid;

    // adding segments from polyline1 to the grid 
    for(int p = 0; p < polyline1.size() - 1; p++) {
        BoundingBox box = getBoundingBox(polyline1[p], polyline1[p+1]);
        
        // determinin which cells does the box (segment) falls into
        int scellX = std::floor((box.minX - minX) / cellSize),scellY = std::floor((box.minY - minY) / cellSize);
        int ecellX = std::floor((box.maxX - minX) / cellSize),ecellY = std::floor((box.maxY - minY) / cellSize);
        
        // popularting the grid
        for(int y = scellY; y <= ecellY; y++) {
            for(int x = scellX; x <= ecellX; x++) {
                std::pair<int, int> cellKey(x, y);
                grid[cellKey].push_back(std::make_pair(p, true)); // true bcz polyline1
            }
        }
    }
    
    // adding segments from polyline2 to the grid 
    for(int p = 0; p < polyline2.size() - 1; p++) {
        BoundingBox box = getBoundingBox(polyline2[p], polyline2[p+1]);
        
        // determinin which cells does the box (segment) falls into
        int scellX = std::floor((box.minX - minX) / cellSize),scellY = std::floor((box.minY - minY) / cellSize);
        int ecellX = std::floor((box.maxX - minX) / cellSize),ecellY = std::floor((box.maxY - minY) / cellSize);
        
        // popularting the grid
        for(int y = scellY; y <= ecellY; y++) {
            for(int x = scellX; x <= ecellX; x++) {
                std::pair<int, int> cellKey(x, y);
                grid[cellKey].push_back(std::make_pair(p, false)); //false bcz polyline2
            }
        }
    }
    
    for(int cY = 0;cY < gridHeight;cY++)
    {
        for(int cX = 0; cX < gridWidth;cX++)
        {

            std::pair<int, int> cellKey(cX, cY);
            auto it_currCell = grid.find(cellKey); // iterator for the map
            if (it_currCell == grid.end() || it_currCell->second.empty()) {
                continue; // skipping if cell not found in map or is empty
            }
            
            // getting the segments in the cell that we are checking
            const std::vector<std::pair<int, bool>>& currSeg = it_currCell->second;
                
            // checking the segments same cell
            if (checkCellPair(currSeg, currSeg, polyline1, polyline2, thresholdSquared)) {
                return true; // Found close segments
            }

            // checking these 4 sides of cell is enough and at the end all neigbours of all cells will be checked with eachother
            int neighborOffsets[][2] = {{-1,1},{1, 0}, {0, 1}, {1, 1}};

            for(const auto& offset : neighborOffsets)
            {
                int nX = cX + offset[0], nY = cY + offset[1];
                if (nX < gridWidth && nY < gridHeight && nX >= 0 && nY >= 0) {
                    std::pair<int, int> nCellKey(nX, nY);

                    auto currNeighbour = grid.find(nCellKey);
                    if (currNeighbour != grid.end() &&  !currNeighbour->second.empty()) {
                        // calling helper function to compare current cell with neighbor
                        if (checkCellPair(currSeg, currNeighbour->second, polyline1, polyline2, thresholdSquared)) {
                            return true; // Found close segments
                        }
                    }
                }
            }
        }
    }
    return false; //if nothin returned true before then the segments are far away
}


// AI generated test cases
int main(){
    // Example Polylines
    std::vector<sPoint2D> polyline1 {
        {0.0f, 0.0f}, {2.0f, 0.0f}, {2.0f, 2.0f}, {4.0f, 2.0f} // Polyline 1
    };

    std::vector<sPoint2D> polyline2_far {
        {5.0f, 5.0f}, {7.0f, 5.0f}, {7.0f, 7.0f} // Far away
    };

     std::vector<sPoint2D> polyline2_close {
        // This segment {0.5, 1.0} to {2.5, 1.0} should be close to {2.0, 0.0} to {2.0, 2.0}
        {0.5f, 1.0f}, {2.5f, 1.0f}, {3.0f, 3.0f}
    };

      std::vector<sPoint2D> polyline3_degenerate {
        {1.0f, -1.0f}, {1.0f, -1.0f} // Degenerate segment (a point) close to {0,0}-{2,0}
    };
     std::vector<sPoint2D> polyline4_single_segment {
        {1.0f, -0.5f}, {1.5f, -0.5f} // Segment near degenerate segment
    };


    std::cout << "Distance Threshold: " << DISTANCE_THRESHOLD << std::endl;
    std::cout << "Threshold Squared: " << DISTANCE_THRESHOLD * DISTANCE_THRESHOLD << std::endl << std::endl;

    std::cout << "--- Polyline 1 vs Polyline 2 (Far) ---" << std::endl;
    std::cout << "  Brute Force:      " << (arePolylinesCloserThanThresholdBrute(polyline1, polyline2_far) ? "Close" : "Not Close") << std::endl;
    std::cout << "  Bounding Box:     " << (arePolylinesCloserThanThresholdBoundingBox(polyline1, polyline2_far) ? "Close" : "Not Close") << std::endl;
    std::cout << "  Spatial Grid:     " << (arePolylinesCloserThanThreshold(polyline1, polyline2_far) ? "Close" : "Not Close") << std::endl; // Renamed grid function
    std::cout << std::endl;

    std::cout << "--- Polyline 1 vs Polyline 2 (Close) ---" << std::endl;
    std::cout << "  Brute Force:      " << (arePolylinesCloserThanThresholdBrute(polyline1, polyline2_close) ? "Close" : "Not Close") << std::endl;
    std::cout << "  Bounding Box:     " << (arePolylinesCloserThanThresholdBoundingBox(polyline1, polyline2_close) ? "Close" : "Not Close") << std::endl;
    std::cout << "  Spatial Grid:     " << (arePolylinesCloserThanThreshold(polyline1, polyline2_close) ? "Close" : "Not Close") << std::endl; // Renamed grid function
    std::cout << std::endl;

    std::cout << "--- Polyline 1 vs Polyline 3 (Degenerate Close) ---" << std::endl;
     std::cout << "  Brute Force:      " << (arePolylinesCloserThanThresholdBrute(polyline1, polyline3_degenerate) ? "Close" : "Not Close") << std::endl;
     std::cout << "  Bounding Box:     " << (arePolylinesCloserThanThresholdBoundingBox(polyline1, polyline3_degenerate) ? "Close" : "Not Close") << std::endl;
     std::cout << "  Spatial Grid:     " << (arePolylinesCloserThanThreshold(polyline1, polyline3_degenerate) ? "Close" : "Not Close") << std::endl; // Renamed grid function
     std::cout << std::endl;

     std::cout << "--- Polyline 4 (Segment) vs Polyline 3 (Degenerate Close) ---" << std::endl;
     std::cout << "  Brute Force:      " << (arePolylinesCloserThanThresholdBrute(polyline4_single_segment, polyline3_degenerate) ? "Close" : "Not Close") << std::endl;
     std::cout << "  Bounding Box:     " << (arePolylinesCloserThanThresholdBoundingBox(polyline4_single_segment, polyline3_degenerate) ? "Close" : "Not Close") << std::endl;
     std::cout << "  Spatial Grid:     " << (arePolylinesCloserThanThreshold(polyline4_single_segment, polyline3_degenerate) ? "Close" : "Not Close") << std::endl; // Renamed grid function
     std::cout << std::endl;
    std::vector<sPoint2D> polyline_intersect1 {
        {0.0f, 0.0f}, {2.0f, 2.0f}
    };
    std::vector<sPoint2D> polyline_intersect2 {
        {0.0f, 2.0f}, {2.0f, 0.0f}
    };

    std::cout << "--- Intersecting Polylines ---" << std::endl;
    std::cout << "  Brute Force:      " << (arePolylinesCloserThanThresholdBrute(polyline_intersect1, polyline_intersect2) ? "Close" : "Not Close") << std::endl;
    std::cout << "  Bounding Box:     " << (arePolylinesCloserThanThresholdBoundingBox(polyline_intersect1, polyline_intersect2) ? "Close" : "Not Close") << std::endl;
    std::cout << "  Spatial Grid:     " << (arePolylinesCloserThanThreshold(polyline_intersect1, polyline_intersect2) ? "Close" : "Not Close") << std::endl;
    std::cout << std::endl;

    return 0;
}
