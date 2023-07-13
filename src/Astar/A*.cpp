#include <iostream>
#include <math.h>
#include "A*.hpp"


void Astar::InitAstar(vector<vector<int>> &_map)
{
    map1 = _map;
}
//得到G
float Astar::getG(Point *parentPoint, Point *curPoint)
{
    float extraG;
    float parentG;
    extraG = fabs(curPoint->x - parentPoint->x) + fabs(curPoint->y - parentPoint->y);
    extraG = (extraG == 1) ? kCost_straight : kconst_diagonal;
    // parentG = curPoint->parent == NULL ? 0 : curPoint->parent->G;
    parentG = parentPoint->G;
    return extraG + parentG;
}




//得到H
float Astar::getH(Point *curPoint,Point *endPoint)
{
    float costH;
    costH = pow( (curPoint->x - endPoint->x),2) + pow( (curPoint->y - endPoint->y),2);
    return sqrt(costH);
}
//得到F
float Astar::getF(Point *curPoint)
{
    return curPoint->G + curPoint->H;
}


//判断点是否在openlist中
Point* Astar::isInlist(const list<Point*> &openList, const Point *curPoint) const
{
    for(auto p : openList)
    {
        if(p->x == curPoint->x && p->y == curPoint->y)
        {
            return p;
        }
    }
    return NULL;
}


//从开放列表中得到最小的F值所对应的点
Point* Astar::getLeastFpoint()
{
    if(!openList.empty())
    {
        auto resPoint = openList.front();
        for(auto p : openList)
        {
            if(p->F < resPoint->F )
            {
                resPoint = p;
            }
        }
        return resPoint;
    }
    return NULL;

}


Point* Astar::searchPath(Point& startPoint, Point& endPoint,bool isIgnoreCorner)
{
    //首先是起始点添加进openlist
    //openList.push_back(&startPoint);
    //置入起点，拷贝开辟一个节点，内外隔离
    openList.push_back(new Point(startPoint.x,startPoint.y) );
    Point* curPoint;
    do{

            //寻找开启列表中F值最低的点，称其为当前点
            curPoint = getLeastFpoint();
            openList.remove(curPoint);
            closeList.push_back(curPoint);
            auto surroundPoint = getSurroundPoint(curPoint,isIgnoreCorner);

            // int count = 0;
            // for (auto it = surroundPoint.begin(); it != surroundPoint.end(); ++it) {
            //     count++;
            // }
            // std::cout << "List capacity: " << count << std::endl;
            // cout<<"test4"<<endl;

            for(auto p :  surroundPoint)
            {
                if(!isInlist(openList,p))
                {
                    p->parent = curPoint;
                    p->G = getG(curPoint,p);
                    p->H = getH(p,&endPoint);
                    p->F = getF(p);

                    openList.push_back(p);
                }
                else
                {
                    if(getG(curPoint,p) < p->G)
                    {
                        p->parent = curPoint;
                        p->G = getG(curPoint,p);
                        // p->H = getH(p,endPoint);
                        p->F = getF(p);
                        cout<<"test"<<endl;
                    }   

                }

            }

        }while(!isInlist(openList,&endPoint));

    if(openList.empty())
    {
        return NULL;
    }
    else
    {
        endPoint.parent = curPoint;
    }
    return &endPoint;

}


list<Point*> Astar::GetPath(Point& start_Point, Point& end_Point,bool isIgnoreCorner)
{
    Point* result = searchPath(start_Point, end_Point,isIgnoreCorner);

    if(result)
    {
        cout<<"result: "<<result->x<<" "<<result->y<<endl;
    }
    else
    {
        cout <<"空指针"<<endl;
    }

    list<Point*> path;
    while(result)
    {
        path.push_front(result);
        result = result->parent;
    }
    //清空列表
    openList.clear();
    closeList.clear();

    return path;
}


//判断是否可以到达
bool Astar::isCanreach(const Point* curPoint, const Point* targetPoint,bool isIgnoreCorner) const
{
    if(targetPoint->x <0 || targetPoint->x > map1.size() - 1 ||
       targetPoint->y <0 || targetPoint->y > map1[0].size() -1 ||
       curPoint->x == targetPoint->x&&curPoint->y == targetPoint->y

       ||map1[targetPoint->x][targetPoint->y] ==1
       || isInlist(closeList,targetPoint)
       )
    {
        return false;
    }
    else
    {
        if(fabs(curPoint->x - targetPoint->x) + fabs(curPoint->y - targetPoint->y) ==1 )
        {
            return true;
        }
        else
        {
            //判断斜对角是否被绊住 应该有四个斜对角 四种case
            if(map1[curPoint->x][curPoint->y+1] ==1&&\
            ( (curPoint->x+1 == targetPoint->x&&curPoint->y+1 == targetPoint->y) 
            ||(curPoint->x-1 == targetPoint->x&&curPoint->y+1 == targetPoint->y) 
            ) )
            {
                return isIgnoreCorner;
            }
            else if(map1[curPoint->x][curPoint->y-1] ==1&&\
            ( (curPoint->x-1 == targetPoint->x&&curPoint->y-1 == targetPoint->y) 
            ||(curPoint->x+1 == targetPoint->x&&curPoint->y-1 == targetPoint->y) 
            ) )
            {
                return isIgnoreCorner;
            }
            else if(map1[curPoint->x-1][curPoint->y] ==1&&\
            ( (curPoint->x-1 == targetPoint->x&&curPoint->y+1 == targetPoint->y) 
            ||(curPoint->x-1 == targetPoint->x&&curPoint->y-1 == targetPoint->y) 
            ) )
            {
                return isIgnoreCorner;
            }
            else if(map1[curPoint->x+1][curPoint->y] ==1&&\
            ( (curPoint->x+1 == targetPoint->x&&curPoint->y+1 == targetPoint->y) 
            ||(curPoint->x+1 == targetPoint->x&&curPoint->y-1 == targetPoint->y) 
            ) )
            {
                return isIgnoreCorner;
            }
            else
            {
                return true;
            }

        }
    }
}
vector<Point*> Astar::getSurroundPoint(const Point* curPoint,bool isIgnoreCorner ) const
{
    vector<Point*> surroundpoints;
    for(int x = curPoint->x - 1; x<=curPoint->x + 1;x++)
    {
        for(int y = curPoint->y - 1; y<=curPoint->y + 1;y++)
        {
            if( isCanreach(curPoint,new Point(x,y), isIgnoreCorner) )
            {
                surroundpoints.push_back(new Point(x,y));
            }
        }
    }
    return surroundpoints;
}