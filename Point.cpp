#include <math.h>
#include <iostream>
class Point
{
   
   
    int  f ,g , h;
public:
     bool closed,opened;
      Point *parent;
    bool walkable;
    int x,y;
Point(int ix, int iy, bool w)
{
    parent = NULL;
    closed = false;
    opened=false;
    f = g = h = 0;
    walkable = w;
    x = ix;
    y = iy;
}

 Point*  getPosition()
{
    return new Point(x, y,1);
}

 Point*  getParent()
{
    return parent;
}

 void  setParent(Point *p)
{
    parent = p;
}

 int  getX()
{
    return x;
}

 int  getY()
{
    return y;
}



 int  getGScore(Point *p)
{
    return p->g + ((x == p->x || y == p->y) ? 10 : 14);
}

int  getHScore(Point *p)
{
    return (abs(p->x - x) + abs(p->y - y)) * 10;
}

int  getGScore()
{
    return g;
}

int  getHScore()
{
    return h;
}

int  getFScore()
{
    return f;
}

 void  computeScores(Point *end)
{
    g = getGScore(parent);
    h = getHScore(end);
    f = g + h;
}

bool  hasParent()
{
    return parent != NULL;
}

};