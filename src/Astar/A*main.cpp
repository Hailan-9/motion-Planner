

#include <iostream>
#include <list>
#include <vector>
#include "A*.hpp"


using namespace std;




bool InPath(const int &row, const int &col, const std::list<Point *> &path) {
  for (const auto &p : path) {
    if (row == p->x && col == p->y) {
      return true;
    }
  }
  return false;
}


int main()
{
    //初始化地图 使用二维数组来代表地图 1障碍物 0可通行
    vector<vector<int>> map = 
   {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1},
    {1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1},
    {1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1},
    {1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};

    Astar astar;
    astar.InitAstar(map);

    //设置起点 终点  计算机中索引都是从0开始 0 1 2 3……
    Point start(1,1);
    Point end(6,10);
    //前端：A*路径寻由
    list<Point*> path = astar.GetPath(start,end,false);

    for(auto p : path)
    {
        cout<<"("<<p->x<<","<<p->y<<") ";
    }
    cout<<endl;
    cout<<"finish"<<endl;

    for (int row = 0; row < map.size(); ++row) {
    for (int col = 0; col < map[0].size(); ++col) {
        if (InPath(row, col, path)) {
        if (map[row][col] != 0) {
            std::cout << "e ";
        } else {
            std::cout << "* ";
        }
        } else {
        std::cout << map[row][col] << " ";
        }
    }
    std::cout << "\n";
    }
    return 0;


}