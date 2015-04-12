#include<createoi.h>
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include"header.h"
#include"param.h"

static int judgeCorner
(struct xy_coord_data_t current_bump_pos, struct xy_coord_data_t before_bump_pos);

static struct xy_coord_data_t getBumpPos
(struct self_position_data_t* absolute_locate);

static int checkCornerProbability
(struct xy_coord_data_t current_bump_pos, struct xy_coord_data_t before_bump_pos);

static int judgeCorner
(struct xy_coord_data_t current_bump_pos, struct xy_coord_data_t before_bump_pos);

static struct xy_coord_data_t getCornerPos
(struct self_position_data_t* absolute_locate);

static int getFieldLength
(struct xy_coord_data_t begin_corner_pos, struct xy_coord_data_t end_corner_pos);

static struct field_corner_angle_data_t setEachCornerAngle
(struct self_position_data_t* absolute_locate, int* corner_approach_angle);


extern const struct wheel_speed_t turn_clockwise_100;
/* 
createが物体と衝突した位置座標(x, y)を返す 
【引数】
*absolute_locate ロボットの自己位置情報
【戻り値】
bump_pos 衝突位置座標
*/
static struct xy_coord_data_t getBumpPos
(struct self_position_data_t* absolute_locate){
  struct xy_coord_data_t bump_pos;

  bump_pos.x = absolute_locate->pos.x;
  bump_pos.y = absolute_locate->pos.y;

  return bump_pos;
}


/*
今回の衝突座標(x,y)と直前の衝突座標(x,y)が
近接している(CORN_PROB_THRE以内）ならば隅の可能性があるとして、
TRUEを返す。
そうでないならば、FALSEを返す
【引数】
current_bump_pos 今回の衝突位置座標
before_bump_pos 直前の衝突位置座標
【戻り値】
TRUE（隅の可能性あり） or FALSE（隅の可能性無し）
*/
static int checkCornerProbability
(struct xy_coord_data_t current_bump_pos, struct xy_coord_data_t before_bump_pos){
  int diff_x, diff_y;

  diff_x = current_bump_pos.x - before_bump_pos.y;
  diff_y = current_bump_pos.y - before_bump_pos.y;
  if(abs(diff_x) < CORN_PROB_THRE_X && abs(diff_y) < CORN_PROB_THRE_Y){
    return TRUE;
  }
  else{
    return FALSE;
  }
}


/*
createが衝突している位置がフィールド隅かを判定する関数である。 
直前の衝突座標と今回の衝突座標が近接しているならば、
corner_probability_flagを立てる。
連続して一定(JUDGE_CORNER_BUMP_NUM)回数corner_probability_flag
が立ったならば、フィールド隅として判断し、TRUEを返す。
隅と判断されないならばFALSEを返す。
【引数】
current_bump_pos 今回の衝突位置座標
before_bump_pos 直前の衝突位置座標
【戻り値】
TRUE（隅と判断） or FALSE（隅でない）
*/
static int judgeCorner
(struct xy_coord_data_t current_bump_pos, struct xy_coord_data_t before_bump_pos){
  int corner_probability_flag, judge_corner_flag;
  static int corner_probability_counter = 0;

  corner_probability_flag = checkCornerProbability(current_bump_pos, before_bump_pos);
  if(corner_probability_flag == TRUE){
    corner_probability_counter++;
  }
  else{
    corner_probability_counter = 0;
  }

  if(corner_probability_counter == JUDGE_CORNER_BUMP_NUM){
    return TRUE;
  }
  else{
    return FALSE;
  }
}

/* 
ロボットの姿勢角度をリセットする （誤差補正の為）
大会規定でフィールドは平行四辺形であるため、
一つ目の角を認識した時の姿勢角度を
0+TARGET_ANGLE*JUDGE_CORNER_BUMP_NUM°
3つ目の角を認識した時の姿勢角度を
180+TARGET_ANGLE*JUDGE_CORNER_BUMP_NUM°
にリセットする。
【引数】
corner_counter 検知した隅のラベル番号
absolute_locate ロボットの自己位置情報
【戻り値】
無し
*/
static void resetAbsoluteTheta
(int corner_counter, struct self_position_data_t* absolute_locate){
  if(corner_counter == 1){
    absolute_locate->theta = TARGET_ANGLE_TO_SERCH_FIELD_FORM * JUDGE_CORNER_BUMP_NUM;
  }
  else if(corner_counter == 3){
    absolute_locate->theta = 180 + TARGET_ANGLE_TO_SERCH_FIELD_FORM * JUDGE_CORNER_BUMP_NUM;
  }
  return;
}

/*
フィールド隅の位置座標を取得する
【引数】
absolute_locate　ロボットの自己位置座標
【戻り値】
corner_pos 隅の位置座標(x, y)
*/
static struct xy_coord_data_t getCornerPos
(struct self_position_data_t* absolute_locate){
  struct xy_coord_data_t corner_pos;

  corner_pos.x = absolute_locate->pos.x;
  corner_pos.y = absolute_locate->pos.y;
  return corner_pos;
}


/*
今回検知した隅の位置座標と直前に検知した隅の位置座標に
基づいて隅と隅の距離を算出する
【引数】
begin_corner_pos 直前に検知した隅の位置座標
end_corner_pos 今回検知した隅の位置座標
【戻り値】
field_length 隅と隅の距離
*/
static int getFieldLength
(struct xy_coord_data_t begin_corner_pos, struct xy_coord_data_t end_corner_pos){
  int diff_x, diff_y;
  int field_length;

  diff_x = end_corner_pos.x - begin_corner_pos.x;
  diff_y = end_corner_pos.y - begin_corner_pos.y;
  field_length = (int)sqrt(diff_x * diff_x + diff_y * diff_y);

  return field_length;
} 

/*
それぞれの隅へのcreateの進入姿勢角度から各隅の角度を求める
【引数】
absolute_locate ロボットの自己位置情報
corner_approach_angle 各隅への進入姿勢角度
【戻り値】
corner_angle 角隅の角度

例）
corner_approach_angle[1] == 0
corner_approach_angle[2] == 100
の場合、一つ目の隅へ姿勢角度０°で進入し、
２つ目の隅へ姿勢角度100°で進入したので、
一つ目の隅の角度(corner_angle.first)は100°
が代入される。
  */
static struct field_corner_angle_data_t setEachCornerAngle
(struct self_position_data_t* absolute_locate, int* corner_approach_angle){
  struct field_corner_angle_data_t corner_angle;

  corner_angle.first = corner_approach_angle[2] - corner_approach_angle[1];
  corner_angle.second = corner_approach_angle[3] - corner_approach_angle[2];
  corner_angle.third = corner_approach_angle[4] - corner_approach_angle[3];
  corner_angle.fourth = (360 - corner_approach_angle[4]) + TARGET_ANGLE_5 * JUDGE_CORNER_BUMP_NUM;

  return corner_angle;
}


struct field_outline_data_t form_getFieldFormInfo
(struct self_position_data_t *absolute_locate, const struct wheel_speed_t go_forward){
  struct xy_coord_data_t current_bump_pos;
  struct xy_coord_data_t before_bump_pos;
  struct xy_coord_data_t start_corner_pos;
  struct xy_coord_data_t first_corner_pos;
  struct xy_coord_data_t second_corner_pos;
  struct field_outline_data_t field_info;
  int corner_counter;
  int loop_counter = 0;
  int judge_bump;
  int judge_corner_flag;
  int virtical_field_length, side_field_length;
  int approach_corner_angle[CORNER_NUM];
  int draw_count = 0;

  char *fname1 = "field_form_coord.txt";
  FILE *fp1;


  /* 外壁情報取得開始時の隅の座標の取得 */
  start_corner_pos.x = absolute_locate->pos.x;
  start_corner_pos.y = absolute_locate->pos.y;
  
  /* 衝突位置座標の初期化 */
  before_bump_pos.x = current_bump_pos.x = 0;
  before_bump_pos.y = current_bump_pos.y = 0;
  
  /* createを直進させる */
  directDrive(go_forward.vel_left, go_forward.vel_right);
  
  while(1){
    getCurrentSelfPos(absolute_locate, go_forward);
    if(loop_counter % DRAW_COUNT_RANGE == 0){
      filePrintCoord(fp1, fname1, "w", absolute_locate->pos.x, absolute_locate->pos.y);
      /* createが衝突したかを判定する */
      /* getBumpsAndWheelDropsはcreateが衝突したならば以下の値を返す */
      /* 右側のbumpsensorが衝突を検知->1 左側のbumpsensor->2 両方のbumpsensorに衝突を検知->3 */
      judge_bump = getBumpsAndWheelDrops();
      if(judge_bump == 1 || judge_bump == 2 || judge_bump == 3){
	current_bump_pos = getBumpPos(absolute_locate);
	
	/*隅に到達したならば、judge_corner_flagを立てる*/
	judge_corner_flag = judgeCorner(current_bump_pos, before_bump_pos);
	
	if(judge_corner_flag == TRUE){
	  corner_counter++;
	  resetAbsoluteTheta(corner_counter, absolute_locate);
	  approach_corner_angle[corner_counter] = absolute_locate->theta;/* 隅に進入時のcreateの姿勢角度を保存 */
	  
	  /*
	    一つ目及び２つ目の隅を見つけた際にフィールドの縦横の長さを計算する 
	    フィールドの形は平行四辺形との規定があるため、２辺の長さしか算出して
	    いない。
	  */
	  if(corner_counter == 1){
	    first_corner_pos = getCornerPos(absolute_locate);
	    field_info.virtical_length = getFieldLength(start_corner_pos, first_corner_pos);
	  }
	  else if(corner_counter == 2){
	    second_corner_pos = getCornerPos(absolute_locate);
	    field_info.side_length = getFieldLength(first_corner_pos, second_corner_pos);
	  }
	}
	if(corner_counter == CORNER_NUM){
	  field_info.corner_angle = setEachCornerAngle(absolute_locate, approach_corner_angle);
	  robot_Stop_Back_Stop(1, BACK_DISTANCE, -100, absolute_locate);
	  absolute_locate->theta = robot_Turn_Stop(TARGET_ANGLE_5, CORRECTION_ANGLE_5, turn_clockwise_100, absolute_locate->theta);
	  break;
	}
	else{
	  robot_Stop_Back_Stop(0.2, BACK_DISTANCE, -100, absolute_locate);
	  absolute_locate->theta = robot_Turn_Stop(TARGET_ANGLE_5, CORRECTION_ANGLE_5, turn_clockwise_100, absolute_locate->theta);
	  before_bump_pos = current_bump_pos;
	}
	countCicleTime();
	draw_count++;
      }
    }
  }
  return field_info;
}
