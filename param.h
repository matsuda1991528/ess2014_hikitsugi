#define FIELD_FORM_SERCH_VELO 100
#define CICLE_TIME_SEC 0.1 //自己位置情報を取得するサイクルタイム[sec]
#define CICLE_TIME 100000  //CICLE_TIME_SECをμsec単位に換算した値
#define DRAW_COUNT_RANGE 10 //whileをDRAW_COUNT_RANGE回すうちの1回座標を書き込む


/* 外壁情報取得に使用するパラメータ群 */
#define CORN_PROB_THRE_X 100 //隅か否かの判定を行うためのしきい値
#define CORN_PROB_THRE_Y 100
#define JUDGE_CORNER_BUMP_NUM 5
#define CORNER_NUM 4         //フィールド隅の個数
#define BACK_DISTANCE 50 //createが壁に衝突した際に後進する距離

#define TRUE 1
#define FALSE -1

#define TARGET_ANGLE_5 5
