#ifndef _8076_PURSUIT_POINT_HPP_
#define _8076_PURSUIT_POINT_HPP_
#include <vector>

class Point {
    private:
    double x, y;

    public:
    Point();
    Point(double ix, double iy);
    friend Point operator+(const Point &n1, const Point &n2);
    friend Point operator-(const Point &n1, const Point &n2);
    friend Point operator*(const Point &n, double num);
    friend Point operator/(const Point &n, double num);

    double getX();
    double getY();
    void setPoint(double ix, double iy);
    double magnitude();
    double dotProduct(Point n);
    std::vector<double> findLookAhead(Point start, Point end, double lookAheadDist);
    Point normalise();
};

#endif 