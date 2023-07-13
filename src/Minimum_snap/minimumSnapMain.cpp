/* */

#include <iostream>
#include "minimumsnap.hpp"
#include "cppTypes.h"
#include <fstream>
#include <string>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <sys/time.h>



using namespace std;
/* 文本输出 */
std::ofstream outfile;


/* 总时间 */
struct timeval startT, endT;
long long plan_total_time;


// int main()
// {

// 	// // 创建输出文件
// 	time_t currentTime;
// 	time(&currentTime);
// 	currentTime = currentTime + 8 * 3600; //	格林尼治标准时间+8个小时
// 	tm *t = gmtime(&currentTime);	
// 	string filename = "/home/zs/vscode C++ wenjian/2023motion_planning/record/data" + to_string(t->tm_mon + 1) + "-" +to_string(t->tm_mday) + "-" + to_string(t->tm_hour) + "-" + to_string(t->tm_min) + ".txt";
// 	outfile.open(filename.c_str());




//     gettimeofday(&startT, NULL);
//     MinimumSnap *minimumSnap_Test = new MinimumSnap();
//     //每维数据用一列
//     Mat3<double> = wayPoint;
//     wayPoint<<1.0, 0.0, 0.0,
//                 2.0, 0.0, 0.4,
//                 3.0, 0.0, 0.0;
//     double total_Time = 1.0;
//     minimumSnap_Test->SolveQp(waypoint,3,1.0);
//     //轨迹输出到文件
//     minimumSnap_Test->PublishTrajectory();
//     for(int i = 0; i < minimumSnap_Test->pos_List.size();i++)
//     {
//         outfile<<"pos: "<<minimumSnap_Test->pos_List[i][0]<<" "<<minimumSnap_Test->pos_List[i][1]
//         <<" "<<minimumSnap_Test->pos_List[i][2]<<endl;

//         outfile<<"vel: "<<minimumSnap_Test->vel_List[i][0]<<" "<<minimumSnap_Test->vel_List[i][1]
//         <<" "<<minimumSnap_Test->vel_List[i][2]<<endl;

//         outfile<<"pos: "<<minimumSnap_Test->acc_List[i][0]<<" "<<minimumSnap_Test->acc_List[i][1]
//         <<" "<<minimumSnap_Test->acc_List[i][2]<<endl;
//     }


//     gettimeofday(&endT, NULL);
//     plan_total_time = (endT.tv_sec - startT.tv_sec)*1000000 + (endT.tv_usec - endT.tv_usec);
//     cout<<"plan_Total_Time: "<<plan_total_time<<endl;

//     return 0;

// }


int main()
{
    cout<<"hel"<<endl;
    return 0;
}