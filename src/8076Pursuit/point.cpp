#include "main.h"

Point::Point(): x{0}, y{0} {}

Point::Point(double ix, double iy): x{ix}, y{iy} {}

Point operator+(const Point &n1, const Point &n2) {
    return {n1.x + n2.x, n1.y + n2.y};
}

Point operator-(const Point &n1, const Point &n2) {
    return {n1.x - n2.x, n1.y - n2.y};
}

Point operator*(const Point &n, double num) {
    return {n.x*num, n.y*num};
}

Point operator/(const Point &n, double num) {
    return {n.x/num, n.y/num};
}

double Point::getX() {
    return x;
}

double Point::getY() {
    return y;
}

void Point::setPoint(double ix, double iy) {
    x = ix;
    y = iy;
}

double Point::magnitude() {
    return sqrt((x * x) + (y * y));
}

double Point::dotProduct(Point n) {
    return (x * n.x + y *n.y);
}

std::vector<double> Point::findLookAhead(Point start, Point end, double lookAheadDist) {
    std::vector<double> result(2, 0);
    Point d = end - start;
    Point f = start - *this;
    double a = d.dotProduct(d);
    double b = 2*(f.dotProduct(d));
    double c = f.dotProduct(f) - lookAheadDist*lookAheadDist;
    double disc = b*b - 4*a*c;
    if (disc < 0) {
        result[0] = 0;
        return result;
    } else {
        disc = sqrt(disc);
        double t1 = (-b-disc)/(2*a);
        double t2 = (-b+disc)/(2*a);
        if(t1 >= 0 && t1 <= 1){
            result[0] = 1; 
            result[1] = t1;
            return result;
        }
        if(t2 >= 0 && t2 <= 1){
            result[0] = 1;
            result[1] = t2;
            return result;
        }
        result[0] = 0;
        return result;
    }
}

Point Point::normalise() {
    double magnitude = this->magnitude();
    return {(this->x)/magnitude, (this->y)/magnitude};
}