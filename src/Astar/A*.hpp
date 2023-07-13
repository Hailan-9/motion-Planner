#pragma once


/* A_star算法 hpp*/

#include <iostream>
#include <vector>
#include <list>

using namespace std;


/* 常量定义 */
const float kCost_straight = 10;
const float kconst_diagonal = 14;


struct Point
{
    float x;
    float y;
    float F, G, H;
    Point *parent;
    //构造函数
    Point(float _x, float _y) : x(_x), y(_y), F(0), G(0), H(0),\
    parent(NULL)
    {

    }
};


class Astar
{
    public:
        void InitAstar(vector<vector<int>> &_map);
        //isIgnoreCorner 是否可以绊住情况下对角线通过
        list<Point *> GetPath(Point& start_Point, Point& end_Point,bool isIgnoreCorner);

    private:


        //判断点是否在openlist中
        Point* isInlist(const list<Point*> &openList, const Point *curPoint) const;
        //得到G
        float getG(Point *parentPoint, Point *curPoint);
        //得到H
        float getH(Point *curPoint,Point *endPoint);
        //得到F
        float getF(Point *curPoint);

        //得到周边的节点
        vector<Point*> getSurroundPoint(const Point* curPoint,bool isIgnoreCorner ) const;
        //从开放列表中得到F值最小的点
        Point* getLeastFpoint();
        //搜索路径 因为searchPath函数需要修改openlist和closelist 所以不能是常量成员函数 也就是不能在最后+ const
        Point* searchPath(Point& startPoint, Point& endPoint,bool isIgnoreCorner);
        //判断是否可以到达
        bool isCanreach(const Point* curPoint,const Point* targetPoint, bool isIgnoreCorner) const;



    private:
        //地图 使用二维数组来表示
        vector< vector<int> > map1;
        list<Point *> openList;
        list<Point *> closeList;
};