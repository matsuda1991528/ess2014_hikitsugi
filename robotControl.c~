#include<createoi.h>
#include<stdio.h>

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
(int back_distance, int back_vel, struct self_position_data_t* absolute_locate){
  struct xy_coord_data_t start_pos;
  int vel_left, vel_right;
  start_pos.x = absolute_locate->pos.x;
  start_pos.y = absolute_locate->pos.y;
  vel_left = vel_right = back_vel;
  
  //TODO:速度と角速度の初期化がいるかの確認
  
  directDrive(back_vel, back_vel);
  while(abs(absolute_locate->pos.x - start_pos.x <= back_distance ||abs(absolute_locate->pos.y - start_pos.y <= back_distance){
	getCurrentSelfPos(absolute_locate, vel_left, vel_right);
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
(double wait_time, int back_distance, int back_vel, struct self_position_data_t* absolute_locate){
  robot_Stop_Wait(wait_time);
  robot_GoBackward(back_distance, back_vel, absolute_locate);
  robot_Stop_Wait(wait_time);

  return;
}
