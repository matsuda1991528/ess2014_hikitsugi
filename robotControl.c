#include<createoi.h>
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include"header.h"
#include"param.h"

extern const struct wheel_speed_t go_inward_100;

/*
ロボットを一定秒停止させる
【引数】
wait_time ロボットを停止させる秒数[sec]
【戻り値】
無し
*/
void robot_Stop_Wait
(double wait_time){
  directDrive(0, 0);
  waitTime(wait_time);
  return;
}


/*
ロボットを一定距離後進させる 
【引数】
back_distance 後進する距離[mm]
back_vel 後進する速度[mm/sec]
absolute_locate ロボットの自己位置情報
【戻り値】
無し
*/
void robot_GoBackward
(int target_distance, int back_vel, struct self_pos_t* absolute_locate){
  struct xy_coord_t start_pos;

  //TODO:速度と角速度の初期化がいるかの確認
  //self_initVelAndOmega(&absolute_locate->vel, &absolute_locate->omega);
  directDrive(go_inward_100.vel_left, go_inward_100.vel_right);
  start_pos.x = absolute_locate->pos.x;
  start_pos.y = absolute_locate->pos.y;
  getCurrentSelfPos(absolute_locate, go_inward_100);
  while(sqrt(pow(absolute_locate->pos.x - start_pos.x,2) + pow(absolute_locate->pos.y - start_pos.y,2)) < target_distance){
    getCurrentSelfPos(absolute_locate, go_inward_100);
    countCicleTime();
  }
  return;
}

/*
createを停止→後進→停止させる。
【引数】
wait_time ロボットを停止させる秒数[sec]
back_distance ロボットを後進させる距離[mm]
back_vel ロボットを後進させる速度[mm/sec]
【戻り値】
無し
*/
void robot_Stop_Back_Stop
(double wait_time, int back_distance, int back_vel, struct self_pos_t* absolute_locate){
  robot_Stop_Wait(wait_time);
  robot_GoBackward(back_distance, back_vel, absolute_locate);
  robot_Stop_Wait(wait_time);

  return;
}

/*
createを指定した角度旋回させた後、一定秒停止させる。
旋回に応じて自己位置情報（姿勢角度のみ）を更新する。
【引数】
target_angle 目標旋回角度[deg]
correction_angle 目標旋回角度の誤差補正角度[deg]
turn_speed 左右の車輪の回転速度[mm/sec]
absolute_locate_theta ロボットの自己姿勢角度[deg]
【戻り値】
absolute_locate_theta ロボットの自己姿勢角度[deg]
*/
int robot_Turn_Stop
(int target_angle, int correction_angle, struct wheel_speed_t turn_speed, int absolute_locate_theta){
  double wait_time = 0.5;
  int current_angle = getAngle();
  int initial_angle = current_angle;
  directDrive(turn_speed.vel_left, turn_speed.vel_right);
  while(1){
    current_angle += getAngle();
    if(abs(current_angle - initial_angle) >= (target_angle - correction_angle)){
      robot_Stop_Wait(wait_time);
      break;
    }
  }
  return absolute_locate_theta += target_angle;
}

