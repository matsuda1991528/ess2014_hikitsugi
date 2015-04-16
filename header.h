#ifndef HEADER_H
#define HEADER_H

/* xy座標を表す構造体 */
struct xy_coord_t{
  int x;
  int y;
};

struct wheel_speed_t{
  int vel_left;
  int vel_right;
};


/* 自己位置を表す構造体 */
struct self_pos_t{
  struct xy_coord_t pos;  //xy座標
  int theta;              //姿勢角度
  int vel;                //速度
  int omega;              //角速度
};

/* 
環境地図情報を表す構造体 
min/max_pos -> セルの最大・最小座標
flag -> セルへの物体存在確率
*/
struct draw_map_t{
  struct xy_coord_t min_pos;
  struct xy_coord_t max_pos;
  int exist_flag;
};

/* ロボットがセンサで認識した物体の座標を表す構造体 */
struct scan_object_t{
  struct xy_coord_t right_end_pos; //右端の座標
  struct xy_coord_t left_end_pos;  //左端の座標
};

/* フィールド隅の角度を表す構造体 */
struct field_corner_angle_data_t{
  int first;  //一つ目の角度
  int second; //２つ目の角度
  int third;  //３つ目の角度
  int fourth; //４つ目の角度
};

/* フィールド外形情報を表す構造体 */
struct field_outline_t{
  struct field_corner_angle_data_t corner_angle;
  int virtical_length;
  int side_length;
};


/* プロトタイプ宣言 */
//commonFunc.c
void countCicleTime();
void initAbsolutePos(struct self_pos_t*);
void initVelAndOmega(int*, int*);
void getCurrentSelfPos
(struct self_pos_t* absolute_locate, const struct wheel_speed_t wheel_speed);
struct self_pos_t getRelativeSelfPos(struct self_pos_t*, struct wheel_speed_t);
void filePrintCoord
(FILE *fp, char *filename, char *mode, int x, int y);

//robotControl.c
void robot_Stop_Back_Stop(double, int, int, struct self_pos_t*);
int robot_Turn_Stop(int, int, struct wheel_speed_t, int);
struct field_outline_t form_getFieldFormInfo(struct self_pos_t*, const struct wheel_speed_t);

#endif
