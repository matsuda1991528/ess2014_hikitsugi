#ifndef HEADER_H
#define HEADER_H

/* xy座標を表す構造体 */
struct xy_coord_data_t{
  int x;
  int y;
};

struct wheel_speed_t{
  int vel_left;
  int vel_right;
};


/* 自己位置を表す構造体 */
struct self_position_data_t{
  struct xy_coord_data_t pos;  //xy座標
  int theta;                   //姿勢角度
  int vel;                     //速度
  int omega;                   //角速度
};

/* 
環境地図情報を表す構造体 
min/max_pos -> セルの最大・最小座標
flag -> セルへの物体存在確率
*/
struct draw_map_data_t{
  struct xy_coord_data_t min_pos;
  struct xy_coord_data_t max_pos;
  int exist_flag;
};

/* ロボットがセンサで認識した物体の座標を表す構造体 */
struct scan_object_data_t{
  struct xy_coord_data_t min_pos;
  struct xy_coord_data_t max_pos;
};

/* フィールド隅の角度を表す構造体 */
struct field_corner_angle_data_t{
  int first;
  int second;
  int third;
  int fourth;
};

/*  */
struct field_outline_data_t{
  struct field_corner_angle_data_t corner_angle;
  int virtical_length;
  int side_length;
};

#endif HEADER_H
