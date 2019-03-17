#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <goal_regist_server/goalRegist.h>
#include <std_msgs/String.h>

///#include <move_base_msgs/MoveBaseAction.h>   ///raspiでは無理
///#include <actionlib/client/simple_action_client.h>   ///raspiでは無理
///typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;   ///raspiでは無理


////////////////////////////////////名前空間の宣言/////////////////////////////////
using namespace std;
////////////////////////////////////構造体定義/////////////////////////////////////
typedef struct goalPoint{

  float x;
  float y;
  string goalName;

}goalPoint;
////////////////////////////////////グローバル変数宣言//////////////////////////////
vector<goalPoint> registedGP;
///move_base_msgs::MoveBaseGoal goal;   ///raspiでは無理
/////////////////////////////////プロトタイプ宣言///////////////////////////////////////////////////////
bool registGoal(vector<goalPoint> &rgstGP,string goalname,float x,float y);   /*ゴール登録関数*/
bool getGoal(vector<goalPoint> &rgstGP,string goalname,goalPoint* output);    /*ゴール情報引き出し関数*/
////////////////////////////////Subscriberコールバック関数//////////////////////////////////////////////
void GoalRegistCB(const goal_regist_server::goalRegist msg){

  registGoal(registedGP ,msg.GoalName,msg.x,msg.y);

}
void Go2PointCB(const std_msgs::String::ConstPtr& msg){
  
    goalPoint goal;

    if(getGoal(registedGP,msg->data,&goal)){
       printf("%s,(x,y)=(%1.4f,%1.4f)\n",goal.goalName.c_str(),goal.x,goal.y);
    }

}
/////////////////////////////////////メイン処理/////////////////////////////////////////////////////////
int main(int argc, char** argv){

  vector<goalPoint>::const_iterator it;
//  goalPoint goal;   

  ros::init(argc, argv, "goal_regist_node");
  ros::NodeHandle n;
  
///  MoveBaseClient ac("move_base", true);   ///raspiでは無理
///  while(!ac.waitForServer(ros::Duration(5.0))){   ///raspiでは無理
///     ROS_INFO("Waiting for the move_base action server to come up");   ///raspiでは無理
///  }   ///raspiでは無理
  ROS_INFO("Goal Regist Server ");

  ros::Rate loop_rate(10);

////デバッグ用///
/*
  registGoal(registedGP,"Point_A",1.0000,0.45875);
  registGoal(registedGP,"Point_B",5.0000,0.75496);
  registGoal(registedGP,"Point_C",1500.0,5841.75);
  registGoal(registedGP,"Point_D",485.57,578.875);
  registGoal(registedGP,"Point_D",585.00,698.162);
  registGoal(registedGP,"Point_E",391.58,584.954);
  for(it = registedGP.begin(); it != registedGP.end(); ++it){
    printf("%s,(x,y)=(%1.4f,%1.4f)\n",it->goalName.c_str(),it->x,it->y);
  }

  if(getGoal(registedGP,"Point_F",&goal)){
     printf("%s,(x,y)=(%1.4f,%1.4f)\n",goal.goalName.c_str(),goal.x,goal.y);
  }
  if(getGoal(registedGP,"Point_A",&goal)){
     printf("%s,(x,y)=(%1.4f,%1.4f)\n",goal.goalName.c_str(),goal.x,goal.y);
  }
*/

///デバッグ用///
    ros::Subscriber sub1 = n.subscribe("GoalPoint", 100, GoalRegistCB);
    ros::Subscriber sub2 = n.subscribe("GoToPoint", 100, Go2PointCB);

  while(ros::ok()){
    
///    ac.waitForResult();   ///raspiでは無理
///    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)   ///raspiでは無理
///       ROS_INFO("Hooray, the base moved 1 meter forward");   ///raspiでは無理
///    else   ///raspiでは無理
///       ROS_INFO("The base failed to move forward 1 meter for some reason");   ///raspiでは無理

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
///////////////////////////////////関数詳細///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
///■ゴール登録関数                                                              //
///-----------------------------------------------------------------------------//
///■指定したゴール地点格納ベクタ(registGP)にゴール情報を登録する関数            //
///-----------------------------------------------------------------------------//
///■引数                                                                        //
///・第１引数：ゴール地点格納ベクタ                                             //
///・第２引数：ゴール名                                                         //
///・第３引数：ゴールX座標                                                      //
///・第４引数：ゴールY座標                                                      //
///■戻り値                                                                      //
///・成功１失敗 0                                                               //
//////////////////////////////////////////////////////////////////////////////////
bool registGoal(vector<goalPoint> &rgstGP,string goalname,float x,float y){

     goalPoint newgoal;
     vector<goalPoint>::const_iterator it;

     for(it = rgstGP.begin(); it != rgstGP.end() ; ++it){

       if(equal(goalname.begin(), goalname.end(), it->goalName.begin())){
       
         printf("The Goal [%s] is already registed \n",goalname.c_str());
         return 0;

       }

     }

     newgoal.goalName = goalname; newgoal.x = x; newgoal.y = y;
     printf("The Goal [%s] is registed successfully \n",goalname.c_str());
     rgstGP.push_back(newgoal);

}
//////////////////////////////////////////////////////////////////////////////////
///■ゴール情報引き出し関数                                                      //
///-----------------------------------------------------------------------------//
///■ゴール地点格納ベクタ(rgstGP)から指定ゴール(goalname)を検索し、              //
/// 該当するものがあれば、output引数から出力する                                //
///-----------------------------------------------------------------------------//
///■引数                                                                        //
///・第１引数：ゴール地点格納ベクタ                                             //
///・第２引数：ゴール名                                                         //
///・第３引数：ゴールデータ                                                     //
///■戻り値                                                                      //
///・成功1　失敗0                                                               //
//////////////////////////////////////////////////////////////////////////////////
bool getGoal(vector<goalPoint> &rgstGP,string goalname,goalPoint* output){

     vector<goalPoint>::const_iterator it;

     for(it = rgstGP.begin(); it != rgstGP.end() ; ++it){
       if(equal(goalname.begin(), goalname.end(), it->goalName.begin())){

          output->goalName = it->goalName;
          output->x = it->x ;
          output->y = it->y ;

          printf("Goal [%s] is found \n", goalname.c_str());
          return 1;

       }     

     }

     printf("Goal [%s] is not found \n",goalname.c_str());
     return 0;

}

