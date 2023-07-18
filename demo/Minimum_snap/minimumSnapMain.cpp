
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


int main()
{

	// 创建输出文件
	time_t currentTime;
	time(&currentTime);
	currentTime = currentTime + 8 * 3600; //	格林尼治标准时间+8个小时
	tm *t = gmtime(&currentTime);	
	string filename = "/home/zs/2023motion_planning/demo/record/data" + to_string(t->tm_mon + 1) + "-" +to_string(t->tm_mday) + "-" + to_string(t->tm_hour) + "-" + to_string(t->tm_min) + ".txt";
	outfile.open(filename.c_str());




    gettimeofday(&startT, NULL);
    MinimumSnap *minimumSnap_Test = new MinimumSnap();
    const int order = 3;
    //每维数据用一行
    DMat<double> wayPoint(3,4);
    DMat<double> _start_end_State(3,2*order);
    // _start_end_State<<10.0, 0.0, 0.0,0.0,  30.0, 0.0, 0.0, 0.0,
    //                   0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0,0.0,
    //                   0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0,0.0;
    _start_end_State<<10.0, 0.0, 0.0,  30.0, 0.0, 0.0,
                      0.0, 0.0, 0.0,   0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0,   0.0, 0.0, 0.0;
    
    cout<<"++++++"<<endl;
    cout<<_start_end_State<<endl;
    cout<<"++++++"<<endl;



    wayPoint<<  10.0, 20.0,  25.0,  30.0,
                0.0,  5.0,   10,    0.0,
                0.0,  0.4,   0.4,   0.0;
    double total_Time = 1.0;
    
    minimumSnap_Test->SolveQp(wayPoint, _start_end_State, order, 1.0, 10, 5);

    //轨迹输出到文件
    minimumSnap_Test->PublishTrajectory();

    gettimeofday(&endT, NULL);
    plan_total_time = (endT.tv_sec - startT.tv_sec)*1000000 + (endT.tv_usec - startT.tv_usec);
    cout<<"plan_Total_Time: "<<plan_total_time/1000.0<<endl;

    for(int i = 0; i < minimumSnap_Test->pos_List.size();i++)
    {
        outfile<<"pos: "<<minimumSnap_Test->pos_List[i][0]<<" "<<minimumSnap_Test->pos_List[i][1]
        <<" "<<minimumSnap_Test->pos_List[i][2]<<endl;

        outfile<<"vel: "<<minimumSnap_Test->vel_List[i][0]<<" "<<minimumSnap_Test->vel_List[i][1]
        <<" "<<minimumSnap_Test->vel_List[i][2]<<endl;

        outfile<<"acc: "<<minimumSnap_Test->acc_List[i][0]<<" "<<minimumSnap_Test->acc_List[i][1]
        <<" "<<minimumSnap_Test->acc_List[i][2]<<endl;
    }




    return 0;

}


