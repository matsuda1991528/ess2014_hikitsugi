#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<time.h>
#include"header.h"
#include"param.h"


/* 
サイクルタイム(自己位置取得の時間間隔)のカウント 
【引数】
無し
【戻り値】
無し
*/
void countCicleTime(){
  clock_t start, end;
  start = clock();
  while(1){
    end = clock();
    if(end - start >= CICLE_TIME)
      break;
  }
  return;
}

/* 
ロボットの自己位置情報の初期化 
x, y座標、姿勢角度、速度、角速度を０に設定
【引数】
absolute_locate ロボットの自己位置情報
【戻り値】
無し
*/
void initAbsolutePos
(struct self_pos_t* absolute_locate){
  absolute_locate->pos.x = 0;
  absolute_locate->pos.y = 0;
  absolute_locate->theta = 0;
  absolute_locate->vel = 0;
  absolute_locate->omega = 0;;
  return;
}


/*
ロボットの自己位置（速度・角速度のみ）の初期化
【引数】
vel  速度
omega　角速度
【戻り値】
無し
*/
void initVelAndOmega
(int* vel, int* omega){
  vel = 0;
  omega = 0;
  return;
}


/* 
ラジアン変換 
【引数】
angle 角度（度数法）
【戻り値】
angle * M_PI / 180　角度（弧度法）
*/
double getRadian
(int angle){
  return angle * M_PI / 180;
}

/*
１サイクルにおけるロボットの移動情報を取得する
詳しくはオドメトリ（自己位置推定法）の相対位置算出を参照のこと
【引数】
absolute_locate 自己位置情報
wheel_speed 車輪の回転速度[mm/sec]
【戻り値】
relative_locate １サイクル前の自己位置との相対位置
*/
struct self_pos_t getRelativeSelfPos
(struct self_pos_t* absolute_locate, struct wheel_speed_t wheel_speed){
  struct self_pos_t relative_locate;
  double past_theta_rad;
  
  past_theta_rad = getRadian(absolute_locate->theta);
  relative_locate.pos.x = absolute_locate->vel * cos(past_theta_rad) * CICLE_TIME_SEC;
  relative_locate.pos.y = absolute_locate->vel * sin(past_theta_rad) * CICLE_TIME_SEC;
  relative_locate.vel = (wheel_speed.vel_left + wheel_speed.vel_right) / 2;
  relative_locate.omega = WHEEL_DISTANCE * (wheel_speed.vel_right - wheel_speed.vel_left) / 2;
  relative_locate.theta = absolute_locate->omega * CICLE_TIME_SEC / 100;

  return relative_locate;
}

/*
前回取得した自己位置との相対位置(relative_locate)
に基づいて現在の自己位置の算出を行う 
【引数】
absolute_locate 自己位置情報
wheel_speed 車輪の回転速度
【戻り値】
無し
*/
void getCurrentSelfPos
(struct self_pos_t* absolute_locate, const struct wheel_speed_t wheel_speed){
  struct self_pos_t relative_locate;

  relative_locate = getRelativeSelfPos(absolute_locate, wheel_speed);
  absolute_locate->pos.x += relative_locate.pos.x;
  absolute_locate->pos.y += relative_locate.pos.y;
  absolute_locate->theta += relative_locate.theta;
  absolute_locate->vel = relative_locate.vel;
  absolute_locate->omega = relative_locate.omega;

  return;
}


/*
ファイル名x, y座標を書き込む
【引数】
*fp ファイルポインタ
*filename ファイル名
*mode ファイルの入出力モード
x x座標
y y座標
【戻り値】
無し
*/
void filePrintCoord
(FILE *fp, char *filename, char *mode, int x, int y){
  static int counter = 0;
  counter++;
  if((fp = fopen(filename, mode)) == NULL){
    printf("cannnot file open\n");
    exit(1);
  }
  else{
    fprintf(fp, "%4d, %4d\n", x, y);
  }

  return;
}
