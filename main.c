/*
 * main.c
 *
 *  Created on: 2015/03/25
 *      Author: matsuda
 */
#include<createoi.h>
#include<stdio.h>
#include"header.h"
#include"param.h"

const struct wheel_speed_t turn_clockwise_100 = {-100, 100};
const struct wheel_speed_t turn_clock_100 = {100, -100};
const struct wheel_speed_t go_forward_100 = {100, 100};
const struct wheel_speed_t go_inward_100 = {-100, -100};
int main(){
  struct self_pos_t absolute_locate;

  startOI_MT("/dev/ttyUSB0");
  /* 自己位置情報の初期化 */
  initAbsolutePos(&absolute_locate);
  /* フィールド外形情報の取得 */
  form_getFieldFormInfo(&absolute_locate, go_forward_100);

  stopOI_MT();
}

