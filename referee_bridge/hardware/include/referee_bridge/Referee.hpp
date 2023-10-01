/** 移植：胡廷文 李逸凡 苗思雷 刘寒
  * 注解：裁判系统中，所有结构体使用__packed关键字进行字节对齐
	*      __packed字节对齐，比如说int float double char它的总大小是4 + 4 + 8 + 1 = 17。
	*      如果不用__packed，系统将以默认的方式对齐（32位系统是4字节），那么它占4 + 4 + 8 + 4 = 20（不足4字节以4字节补齐）。
	*      2021.10.2
	*/


#ifndef REFEREE_H
#define REFEREE_H

#include <stdbool.h>
#include <cstring>
#include <string>
#include <CRC.hpp>

#define LEN_HEADER 					5 		//帧头长
#define LEN_CMDID 					2	 		//命令码长度
#define LEN_TAIL 						2			//帧尾CRC16
#define JUDGE_FRAME_HEADER (0xA5)	//起始字节,协议固定为0xA5
#define FALSE 							0
#define TRUE 								1
#define BLUE 								0	
#define RED 								1
#define YES 								true
#define NO  								false
#define JUDGE_HUART					huart3

#define __packed __attribute__((packed))


//通信协议格式，偏移位置
typedef enum
{
	FRAME_HEADER = 0,
	CMD_ID = 5,
	DATA = 7,
} JudgeFrameOffset;

//frame_header格式：5字节帧头,偏移位置
typedef enum
{
	SOF = 0,		 				//起始位
	DATA_LENGTH = 1, 		//帧内数据长度,根据这个来获取数据长度
	SEQ = 3,					 	//包序号
	CRC8 = 4					  //CRC8
} FrameHeaderOffset;

//自定义帧头
typedef struct __packed
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
} Frame_Header_t;

typedef struct __packed
{
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
    uint16_t cmd_id;
    uint8_t data[256]{};
    bool CheckHeaderCRC8() {
      return Verify_CRC8_Check_Sum((uint8_t *)this, GetHeaderLength());
    }
    bool CheckPackCRC16()
    {
      return Verify_CRC16_Check_Sum((uint8_t *)this, GetLength());
    }
    uint16_t CheckTail()
    {
      uint16_t tail = *(uint16_t *)((uint8_t *)this + GetLength() - 2);
      return tail;
    }
    void AddCRC()
    {
      Append_CRC8_Check_Sum((uint8_t *)this, GetHeaderLength());
      Append_CRC16_Check_Sum((uint8_t *)this, GetLength());
    }
    size_t GetHeaderLength() { return 5; }
    size_t GetLength() { return 7 + data_length + 2; }
} FrameBuffer;

//命令码ID
typedef enum
{
	ID_game_state = 0x001,					 						//比赛状态数据
	ID_game_result = 0x002,					 						//比赛结果数据
	ID_game_robot_HP = 0x003,				 						//比赛机器人血量数据
	ID_event_data = 0x101,					 						//场地事件数据
	ID_supply_projectile_action = 0x102,			  //补给站动作标识

	ID_referee_warning = 0x104,								  //裁判警告信息
	ID_dart_remaining_time = 0x105,						  //飞镖发射口倒计时
	ID_game_robot_state = 0x201,							  //比赛机器人状态数据
	ID_power_heat_data = 0x202,								  //实时功率热量数据
	ID_game_robot_pos = 0x203,								  //机器人位置数据
	ID_buff_musk = 0x204,											  //机器人增益数据
	ID_aerial_robot_energy = 0x205, 						//空中机器人能量状态数据
	ID_robot_hurt = 0x206,					 						//伤害状态数据
	ID_shoot_data = 0x207,										  //实时射击数据
	ID_bullet_remaining = 0x208,							  //子弹剩余发射数(空中机器人，哨兵机器人以及 ICRA 机器人主控发送)
	ID_rfid_status = 0x209,										  //机器人 RFID 状态
	ID_dart_client_cmd = 0x20A,				 					//飞镖机器人客户端指令数据
	ID_ground_robot_position = 0x20B,                   //地面机器人位置数据，对哨兵机器人发送，以 1Hz 频率发送
	ID_robot_interactive_data = 0x0301,					//机器人间交互数据 10Hz
	ID_controller_interactive_data = 0x0302,		//自定义控制器交互数据接口 30Hz
	ID_map_interactive_data = 0x0303,						//客户端小地图交互数据
	ID_keyboard_information = 0x0304,						//键盘、鼠标信息
	ID_map_sentry = 0x0307,                                  // 选手端小地图接收哨兵数据，频率上限为 1Hz	
	ID_super_cap = 0x03F5 	//超级电容 
} CmdID;

typedef enum
{	
	ID_all_state_tx                         = 0x0A1
} TX_CmdID;


/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef struct __packed 
{ 
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t  SyncTimeStamp;
} ext_game_state_t; 


/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef struct __packed
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003  Byte:  2    比赛机器人存活数据 */
typedef struct __packed
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;


/* ID: 0x0004  Byte:  3    飞镖发射状态 */
typedef struct __packed
{
	uint8_t dart_belong;
	uint16_t stage_remaining_time;
} ext_dart_status_t;


/* ID: 0x0005  Byte:  3    . 人工智能挑战赛加成与惩罚区状态(用不到) */
typedef struct __packed
{
 uint8_t F1_zone_status:1;
 uint8_t F1_zone_buff_debuff_status:3;
 uint8_t F2_zone_status:1;
 uint8_t F2_zone_buff_debuff_status:3;
 uint8_t F3_zone_status:1;
 uint8_t F3_zone_buff_debuff_status:3;
 uint8_t F4_zone_status:1;
 uint8_t F4_zone_buff_debuff_status:3;
 uint8_t F5_zone_status:1;
 uint8_t F5_zone_buff_debuff_status:3;
 uint8_t F6_zone_status:1;
 uint8_t F6_zone_buff_debuff_status:3;
	
 uint16_t red1_bullet_left;
 uint16_t red2_bullet_left;
 uint16_t blue1_bullet_left;
 uint16_t blue2_bullet_left;	
}ext_ICRA_buff_debuff_zone_status_t;




/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef struct __packed
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  4    场地补给站动作标识数据 */
typedef struct __packed
{
 uint8_t supply_projectile_id;
 uint8_t supply_robot_id;
 uint8_t supply_projectile_step;
 uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


/* ID: 0x0104  Byte:  2    裁判警告信息 */
typedef struct __packed
{
 uint8_t level;
	
 uint8_t offending_robot_id;
} ext_referee_warning_t;


/* ID: 0x0105  Byte:  1    飞镖发射口倒计时 */
typedef struct __packed
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;


/* ID: 0X0201  Byte: 18    机器人状态数据 */
typedef struct __packed
{ 
	uint8_t robot_id;                       //机器人ID，可用来校验发送
	uint8_t robot_level;                    //1一级，2二级，3三级
	uint16_t remain_HP;                     //机器人剩余血量
	uint16_t max_HP;                        //机器人满血量
  uint16_t shooter_id1_17mm_cooling_rate;
	
  uint16_t shooter_id1_17mm_cooling_limit;
  uint16_t shooter_id1_17mm_speed_limit;
  uint16_t shooter_id2_17mm_cooling_rate;
  uint16_t shooter_id2_17mm_cooling_limit;
	
  uint16_t shooter_id2_17mm_speed_limit;
  uint16_t shooter_id1_42mm_cooling_rate;
  uint16_t shooter_id1_42mm_cooling_limit;
  uint16_t shooter_id1_42mm_speed_limit;
	
  uint16_t chassis_power_limit;
  uint8_t mains_power_gimbal_output : 1;
  uint8_t mains_power_chassis_output : 1;
  uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t; 


/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef struct __packed
{ 
 uint16_t chassis_volt;
 uint16_t chassis_current;
 float chassis_power;
 uint16_t chassis_power_buffer;
 uint16_t shooter_id1_17mm_cooling_heat;
 uint16_t shooter_id2_17mm_cooling_heat;
 uint16_t shooter_id1_42mm_cooling_heat;
 int16_t shooter_id1_17mm_residual_cooling_heat;	//剩余热量，非官方类型
 int16_t shooter_id2_17mm_residual_cooling_heat;	//剩余热量，非官方类型
 int16_t shooter_id1_42mm_residual_cooling_heat;	//剩余热量，非官方类型
} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef struct __packed
{   
	float x;   
	float y;                                    
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef struct __packed
{ 
	uint8_t power_rune_buff;  // bit 0：机器人血量补血状态 bit 1：枪口热量冷却加速 bit 2：机器人防御加成 bit 3：机器人攻击加成
} ext_buff_musk_t; 


/* ID: 0x0205  Byte:  3    空中机器人能量状态数据 */
typedef struct __packed
{ 
	uint8_t attack_time; 
} aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef struct __packed 
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  6    实时射击数据 */
typedef struct __packed 
{ 
 uint8_t bullet_type;
 uint8_t shooter_id;
 uint8_t bullet_freq;
 float bullet_speed; 
} ext_shoot_data_t; 

/* ID: 0x0208  Byte:  6    子弹剩余发射数 */
typedef struct __packed 
{
 uint16_t bullet_remaining_num_17mm;
 uint16_t bullet_remaining_num_42mm;
 uint16_t remaining_gold_coin;
} ext_bullet_remaining_t;


/* ID: 0x0209  Byte:  4    机器人 RFID 状态 */
typedef struct __packed 
{
 uint32_t rfid_status;
} ext_rfid_status_t;

//ID：0x020A  Byte:  12    飞镖机器人客户端指令数据
typedef struct __packed 
{
 uint8_t dart_launch_opening_status;
 uint8_t dart_attack_target;
 uint16_t target_change_time;
 uint8_t first_dart_speed;
 uint8_t second_dart_speed;
 uint8_t third_dart_speed;
 uint8_t fourth_dart_speed;
 uint16_t last_dart_launch_time;
 uint16_t operate_launch_cmd_time;
}ext_dart_client_cmd_t;

//ID：0x020B  Byte:  40     地面机器人位置数据，对哨兵机器人发送，以 1Hz 频率发送
typedef struct __packed 
{
 float hero_x;
 float hero_y;
 float engineer_x;
 float engineer_y;
 float standard_3_x;
 float standard_3_y;
 float standard_4_x;
 float standard_4_y;
 float standard_5_x;
 float standard_5_y;
}ext_ground_robot_position_t;

//ID 0x020C  Byte:  6   雷达标记进度数据，向雷达发送，以 1Hz 频率发送
typedef struct __packed 
{
 uint8_t mark_hero_progress;
 uint8_t mark_engineer_progress;
 uint8_t mark_standard_3_progress;
 uint8_t mark_standard_4_progress;
 uint8_t mark_standard_5_progress;
 uint8_t mark_sentry_progress;
}radar_mark_data_t;
//ID 0x0307  Byte:  103  选手端小地图接收哨兵数据，频率上限为 1Hz
typedef struct __packed 
{
 uint8_t intention;
 uint16_t start_position_x;
 uint16_t start_position_y;
 int8_t delta_x[49];
 int8_t delta_y[49];
}ext_map_sentry_data_t;

/* 裁判系统数据结构体 */
typedef struct
{
	Frame_Header_t 								FramHeader;						// 帧头信息
	ext_game_state_t 							GameState;						// 0x0001     比赛状态数据
	ext_game_result_t 							GameResult;						// 0x0002     比赛结果数据
	ext_game_robot_HP_t 						GameRobotHP;					// 0x0003     机器人血量
	ext_event_data_t							EventData;						// 0x0101   	场地数据
	ext_supply_projectile_action_t	            SupplyProjectileAction;		    // 0x0102 		补给站动作标识
	ext_referee_warning_t						RefereeWarning;					// 0x0104   	裁判警告信息
	ext_dart_remaining_time_t				    dart_remaining_time;			// 0x0105    	飞镖发射口倒计时
	ext_game_robot_state_t					    GameRobotStat;					// 0x0201   	比赛机器人状态
	ext_power_heat_data_t						PowerHeatData;					// 0x0202   	实时功率热量数据
	ext_game_robot_pos_t						GameRobotPos;					// 0x0203    	机器人位置
	ext_buff_musk_t								Buff;							// 0x0204  		机器人增益
	aerial_robot_energy_t						AerialRobotEnergy;				// 0x0205    	空中机器人能量状态
	ext_robot_hurt_t							RobotHurt;						// 0x0206    	伤害状态
	ext_shoot_data_t							ShootData;						// 0x0207     实时射击信息
	ext_bullet_remaining_t					    bullet_remaining;				// 0x0208	    子弹剩余发射数
	ext_rfid_status_t							RFIDState;						// 0x0209	    RFID状态
	ext_dart_client_cmd_t                       DartClient;        				// 0x020A     飞镖客户端
	ext_ground_robot_position_t       			GroundRobotPosition;            // 0x020B      地面机器人位置数据，对哨兵机器人发送
	ext_map_sentry_data_t                       MapSentry;                      // 0x0307       选手端小地图接收哨兵数据，频率上限为 1Hz
	uint16_t                        			self_client;       			    // 本机客户端                               SuperCap;
   	float										SuperCapRemain;
	float										SuperCapRemain2;					//超级电容剩余量                
}Judge_Info_t;


//命令码数据段长,根据官方协议来定义长度
typedef enum
{
	LEN_game_state = 11,					 						 		//比赛状态数据
	LEN_game_result = 1,												  //比赛结果数据
	LEN_game_robot_HP = 32,										 	  //比赛机器人存活数据
	LEN_event_data = 4,													  //场地事件数据
	LEN_supply_projectile_action = 4,		 					//补给站动作标识
	LEN_referee_warning = 2,				 							//裁判警告信息
	LEN_dart_remaining_time_t = 1,						    //飞镖发射口倒计时
	LEN_game_robot_state = 27,				  				  //比赛机器人状态数据
	LEN_power_heat_data = 16,									 	  //实时功率热量数据
	LEN_game_robot_pos = 16,	    				  	  	//机器人位置数据
	LEN_buff_musk = 1,						 								//机器人增益数据
	LEN_aerial_robot_energy = 1,							 		//空中机器人能量状态数据
	LEN_robot_hurt = 1,		//伤害状态数据
	LEN_shoot_data = 7,						 								//实时射击数据 
	LEN_bullet_remaining = 6,											//子弹剩余发射数(空中机器人，哨兵机器人以及 ICRA 机器人主控发送)
	LEN_rfid_status = 4,													//机器人 RFID 状态
	LEN_dart_client_cmd = 6,											//飞镖机器人客户端指令数据
	LEN_map_interactive_data = 15,								//客户端小地图交互数据
	LEN_keyboard_information = 12,								  //键盘、鼠标信息
	LEN_ground_robot_position = 40, 								  //地面机器人位置数据，对哨兵机器人发送，以 1Hz 频率发送
	LEN_map_sentry = 103                                             // 选手端小地图接收哨兵数据，频率上限为 1Hz
} JudgeDataLength;

    
/*****************超级电容剩余量**********************/
void get_supercap_remain(Judge_Info_t *ptr, uint8_t *Data);
	
/*****************判断红蓝方**********************/
bool is_red_or_blue(void);

/***************判断裁判系统是否连接**********************/
void Judge_IF_REF_ONL(void);

bool Judge_Read_Data(uint8_t *ReadFromUsart);
#endif

