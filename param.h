#define CICLE_TIME_SEC 0.1 //自己位置情報を取得するサイクルタイム[sec]
#define CICLE_TIME 100000  //CICLE_TIME_SECをμsec単位に換算した値
#define M_PI 3.14159265358979323846 //円周率
#define DRAW_COUNT_RANGE 10 //whileをDRAW_COUNT_RANGE回すうちの1回座標を書き込む


/* 外壁情報取得に使用するパラメータ群 */
#define CORN_PROB_THRE_X 150 //隅か否かの判定を行うためのしきい値
#define CORN_PROB_THRE_Y 150
#define JUDGE_CORNER_BUMP_NUM 3 //JUDGE_CORNER_BUMP_NUM回類似した位置で衝突したら隅と判定
#define TARGET_ANGLE_TO_SERCH_FIELD_FORM 5 //外壁情報取得用の旋回角度
#define CORNER_NUM 4         //フィールド隅の個数
#define BACK_DISTANCE 50 //createが壁に衝突した際に後進する距離

#define WHEEL_DISTANCE 30 //左右の車輪間の距離
#define TRUE 1
#define FALSE -1

#define TARGET_ANGLE_5 5
#define CORRECTION_ANGLE_5 2


#define FORWARD_VELO_100 100
#define FORWARD_VELO_200 200
#define INWARD_VELO_100 -100
#define INWARD_VELO_200 -200
