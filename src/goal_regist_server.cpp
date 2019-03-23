#include <ros/ros.h>
#include <vector>
#include <stdio.h>
#include <algorithm>
#include <math.h>
#include <goal_regist_server/goalRegist.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>   
#include <actionlib/client/simple_action_client.h>   
#include <nav_msgs/Odometry.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;//MoveBaseクライアント
/////////////////////////////////////////////////名前空間の宣言/////////////////////////////////////
using namespace std;
//////////////////////////////////////////////構造体定義////////////////////////////////////////////
typedef struct goalPoint{

  float x;//ゴールx座標
  float y;//ゴールy座標
  string goalName;//ゴール名

}goalPoint;
////////////////////////////////////////////////グローバル変数宣言/////////////////////////////////////
vector<goalPoint> registedGP;//ゴール地点格納ベクタ
move_base_msgs::MoveBaseGoal goal;//ゴール地点送信用変数
bool goal_protect;//ゴール指令値送信プロテクタ
 geometry_msgs::Pose robot_pose;//オドメトリ結果取得用変数
/////////////////////////////////プロトタイプ宣言///////////////////////////////////////////////////////
bool registGoal(vector<goalPoint> &rgstGP,string goalname,float x,float y);   /*ゴール登録関数*/
bool getGoal(vector<goalPoint> &rgstGP,string goalname,goalPoint* output);    /*ゴール情報引き出し関数*/

////////////////////////////////Subscriberコールバック関数//////////////////////////////////////////////
//////////////////////////////////////ゴール地点登録用コールバック関数///////////////////////////////////
void GoalRegistCB(const goal_regist_server::goalRegist msg){

  registGoal(registedGP ,msg.GoalName,robot_pose.position.x,robot_pose.position.y);//ゴールベクタ登録関数に登録する関数

}
//////////////////////////////////////////actionlib起動用コールバック関数///////////////////////////////
void Go2PointCB(const std_msgs::String::ConstPtr& msg){
 //   float theta = 0.000;///移動方向にロボットが向く処理はまだ入れてない。(efqとqfeの処理を入れてない)
    goalPoint goal_point;
    if(getGoal(registedGP,msg->data,&goal_point)){//ゴールベクタから指定ゴール地点を取り出す関数
       printf("%s,(x,y)=(%1.4f,%1.4f)\n",goal_point.goalName.c_str(),goal_point.x,goal_point.y);
    }
    //base_link座標系
    goal.target_pose.header.frame_id = "base_link"; //base_link座標系
    //goal.target_pose.header.frame_id = "base_footprint";//base_footprint座標系
    goal.target_pose.header.stamp = ros::Time::now();//現在時刻
    goal.target_pose.pose.position.x = goal_point.x - robot_pose.position.x;
    goal.target_pose.pose.position.y = goal_point.y - robot_pose.position.y;
  //  theta = atan2(goal.target_pose.pose.position.x,goal.target_pose.pose.position.y);
  //  goal.target_pose.pose.orientation.z = theta - robot_pose.orientation.z;
    goal.target_pose.pose.orientation.w = 1;
    goal_protect = true;//送信アクティブ

}
//////////////////////////////////////オドメトリ(amcl)用コールバック関数///////////////////////////////////
//void odomCallback(const nav_msgs::Odometry &odom_msg){//オドメトリで取得する場合
void odomCallback(const geometry_msgs::PoseWithCovarianceStamped &odom_msg){//amclで取得する場合
	ROS_INFO("odom : x %lf  : y %lf\n", odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y); 
	robot_pose.position.x   =  odom_msg.pose.pose.position.x;
	robot_pose.position.y   =  odom_msg.pose.pose.position.x;
	robot_pose.position.z   =  odom_msg.pose.pose.position.z;
	robot_pose.orientation.x = odom_msg.pose.pose.orientation.x;
	robot_pose.orientation.y = odom_msg.pose.pose.orientation.y;
	robot_pose.orientation.z = odom_msg.pose.pose.orientation.z;
	robot_pose.orientation.w = odom_msg.pose.pose.orientation.w;

}
/////////////////////////////////////メイン処理/////////////////////////////////////////////////////////
int main(int argc, char** argv){

  vector<goalPoint>::const_iterator it;//イテレータ宣言
  ros::init(argc, argv, "goal_regist_node");//ノード名宣言
  ros::NodeHandle n;//ノードハンドラ宣言
  MoveBaseClient ac("move_base", true);                                  //MoveBaseクライアント用インスタンス


  /*//初期位置送信パブリッシャ
	  pub= nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 2, true);
	  ros::Time tmp_time = ros::Time::now();//現在時間取得
	　　geometry_msgs::PoseWithCovarianceStamped initPose;//初期位置用変数宣言
	　　initPose.header.stamp = tmp_time;  //  時間
	　　initPose.header.frame_id = "map";  //  フレーム
	　　initPose.pose.pose.position.x = 0.0;
	  initPose.pose.pose.position.y = 0.0;
	  initPose.pose.pose.position.z =  0;
	  initPose.pose.pose.orientation.w = 1.0;
	  pub.publish(initPose);// パブリッシュ amclパッケージに初期位置を送る
  */
  /*////デバッグ用///
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

  while(!ac.waitForServer(ros::Duration(5.0))){//MoveBaseサーバ起動待機
     ROS_INFO("Waiting for the move_base action server to come up");   
  }   
  ROS_INFO("Goal Regist Server ");//MoveBaseサーバ起動アナウンス

  ros::Subscriber sub1 = n.subscribe("GoalPoint", 100, GoalRegistCB);//ゴール地点登録用サブスクライバ
  ros::Subscriber sub2 = n.subscribe("GoToPoint", 100, Go2PointCB);//ゴール地点をアクションサーバへ送信
  ros::Subscriber sub3 = n.subscribe("amcl_pose", 100, odomCallback);//オドメトリ情報取得用サブスクライバ

  ros::Rate loop_rate(10);//ノード更新周期10Hz
  //無限ループ
  while(ros::ok()){

    if(goal_protect)
    {
    ac.sendGoal(goal);
    ac.waitForResult();//アクションサーバからの実行結果待ち待機   
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)//ゴール到着成功   
       ROS_INFO("Hooray, the base moved 1 meter forward");   
    else                                                            //ゴール到着失敗
       ROS_INFO("The base failed to move forward 1 meter for some reason"); 
       goal_protect = false;//送信インアクティブ 
    }
    ros::spinOnce();
    loop_rate.sleep();//周期分待機


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

       //ゴール名重複防止用ガード
       if(equal(goalname.begin(), goalname.end(), it->goalName.begin())){
       
         printf("The Goal [%s] is already registed \n",goalname.c_str());
         return 0;

       }

     }

     newgoal.goalName = goalname; newgoal.x = x; newgoal.y = y;
     printf("The Goal [%s] is registed successfully \n",goalname.c_str());
     rgstGP.push_back(newgoal);//ゴールベクタへ登録

}
//////////////////////////////////////////////////////////////////////////////////
///■ゴール情報引き出し関数                                                      　//
///-----------------------------------------------------------------------------//
///■ゴール地点格納ベクタ(rgstGP)から指定ゴール(goalname)を検索し、              　　　　　//
/// 該当するものがあれば、output引数から出力する                               　　　　 　//
///-----------------------------------------------------------------------------//
///■引数                                                                      　//
///・第１引数：ゴール地点格納ベクタ                                             　　　　　//
///・第２引数：ゴール名                                                       　　 　 //
///・第３引数：ゴールデータ                                                    　　　 　//
///■戻り値                                                                     //
///・成功1　失敗0                                                               　//
//////////////////////////////////////////////////////////////////////////////////
bool getGoal(vector<goalPoint> &rgstGP,string goalname,goalPoint* output){

     vector<goalPoint>::const_iterator it;
     //登録ゴール名検索処理
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

