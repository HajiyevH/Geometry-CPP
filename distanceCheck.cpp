#include <iostream>
#include <vector>

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

int main() {

}