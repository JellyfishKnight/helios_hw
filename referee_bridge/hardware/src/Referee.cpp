#include "Referee.hpp"
#include "CRC.hpp"

#define BLUE 0
#define RED 1
/**************裁判系统数据辅助****************/
float judge_time_online = 0;			//用于判断裁判系统是否连线
uint16_t ShootNum;								//统计发弹量,0x0003触发一次则认为发射了一颗
bool Hurt_Data_Update = FALSE;		//装甲板伤害数据是否更新,每受一次伤害置TRUE,然后立即置FALSE,给底盘闪避用
bool Judge_Data_TF = FALSE;				//裁判数据是否可用,辅助函数调用
/*****************系统数据定义**********************/
Judge_Info_t JudgeInfo;
/****************************************************/

// /**
//  * @brief  判断自己红蓝方
//  * @param  void
//  * @retval RED   BLUE
//  * @attention  数据打包,打包完成后通过串口发送到裁判系统
//  */
// bool Color;
// bool is_red_or_blue(void)
// {
// 	if (judge_system_data.GameRobotStat.robot_id > 10)
// 	{
// 		return BLUE;
// 	}
// 	else
// 	{
// 		return RED;
// 	}
// }

int hurt_count = 0;
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE; //数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误
	uint16_t judge_length; //统计一帧数据长度
	int CmdID = 0; //数据命令码解析
	//无数据包，则不作任何处理
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	//写入帧头数据,用于判断是否开始存储裁判数据
	memcpy(&JudgeInfo.FramHeader, ReadFromUsart, LEN_HEADER);
	//判断帧头数据是否为0xA5
	if (ReadFromUsart[SOF] == JUDGE_FRAME_HEADER)
	{
		//帧头CRC8校验
		if (Verify_CRC8_Check_Sum(ReadFromUsart, LEN_HEADER) == TRUE)
		{
			//统计一帧数据长度,用于CR16校验
			judge_length = ReadFromUsart[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;

			//帧尾CRC16校验
			if (Verify_CRC16_Check_Sum(ReadFromUsart, judge_length) == TRUE)
			{
				retval_tf = TRUE; //都校验过了则说明数据可用

				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				switch (CmdID)
				{
					case ID_game_state: //0x0001
						memcpy(&JudgeInfo.GameState, (ReadFromUsart + DATA), LEN_game_state);
						break;

					case ID_game_result: //0x0002
						memcpy(&JudgeInfo.GameResult, (ReadFromUsart + DATA), LEN_game_result);
						break;
					case ID_game_robot_HP: //0x0003
						memcpy(&JudgeInfo.GameRobotHP, (ReadFromUsart + DATA), LEN_game_robot_HP);
						
						break;

					case ID_event_data: //0x0101
						memcpy(&JudgeInfo.EventData, (ReadFromUsart + DATA), LEN_event_data);
						break;

					case ID_supply_projectile_action: //0x0102
						memcpy(&JudgeInfo.SupplyProjectileAction, (ReadFromUsart + DATA), LEN_supply_projectile_action);
						break;

					case ID_referee_warning: //0x0104
						memcpy(&JudgeInfo.RefereeWarning, (ReadFromUsart + DATA), LEN_referee_warning);
						break;

					case ID_game_robot_state: //0x0201
						memcpy(&JudgeInfo.GameRobotStat, (ReadFromUsart + DATA), LEN_game_robot_state);
						
					break;

					case ID_power_heat_data: //0x0202
						memcpy(&JudgeInfo.PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
						JudgeInfo.PowerHeatData.shooter_id1_17mm_residual_cooling_heat=JudgeInfo.GameRobotStat.shooter_id1_17mm_cooling_limit - JudgeInfo.PowerHeatData.shooter_id1_17mm_cooling_heat;
						JudgeInfo.PowerHeatData.shooter_id2_17mm_residual_cooling_heat=JudgeInfo.GameRobotStat.shooter_id2_17mm_cooling_limit - JudgeInfo.PowerHeatData.shooter_id2_17mm_cooling_heat;
					break;
					
					case ID_game_robot_pos: //0x0203
						memcpy(&JudgeInfo.GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
						//发送
						break;

					case ID_buff_musk: //0x0204
						memcpy(&JudgeInfo.Buff, (ReadFromUsart + DATA), LEN_buff_musk);
						break;

					case ID_aerial_robot_energy: //0x0205
						memcpy(&JudgeInfo.AerialRobotEnergy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
						break;

					case ID_robot_hurt: //0x0206
						memcpy(&JudgeInfo.RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
					hurt_count++;
//						if (JudgeInfo.RobotHurt.hurt_type == 0) //非装甲板离线造成伤害
//						{
//							Hurt_Data_Update = TRUE;
//						} //装甲数据每更新一次则判定为受到一次伤害
					break;

					case ID_shoot_data: //0x0207
						memcpy(&JudgeInfo.ShootData, (ReadFromUsart + DATA), LEN_shoot_data);

						break;
						
					case ID_bullet_remaining:      	//0x0208
						memcpy(&JudgeInfo.bullet_remaining, (ReadFromUsart + DATA), LEN_bullet_remaining);
						break; 

					case ID_rfid_status: //0x0209
						memcpy(&JudgeInfo.RFIDState, (ReadFromUsart + DATA), LEN_rfid_status);
						break;

					case ID_dart_client_cmd: //0x020A
						memcpy(&JudgeInfo.DartClient, (ReadFromUsart + DATA), LEN_dart_client_cmd);
						break;
//					case ID_map_sentry:            //0x0307
//						memcpy(&JudgeInfo.MapSentry, (ReadFromUsart + DATA), LEN_map_sentry);
//						break;
				}
			}
		}
		//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
		if (*(ReadFromUsart + sizeof(Frame_Header_t) + LEN_CMDID + JudgeInfo.FramHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//如果一个数据包出现了多帧数据,则再次读取
			Judge_Read_Data(ReadFromUsart + sizeof(Frame_Header_t) + LEN_CMDID + JudgeInfo.FramHeader.DataLength + LEN_TAIL);
		}
	}
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE; //辅助函数用
	}
	else //只要CRC16校验不通过就为FALSE
	{
		Judge_Data_TF = FALSE; //辅助函数用
	}
	return retval_tf; //对数据正误做处理
}


/*****************超级电容剩余量**********************/
void get_supercap_remain(Judge_Info_t *ptr, uint8_t *Data)
{
	((uint8_t *)(&ptr->SuperCapRemain))[0] = Data[0];
	((uint8_t *)(&ptr->SuperCapRemain))[1] = Data[1];
	((uint8_t *)(&ptr->SuperCapRemain))[2] = Data[2];
	((uint8_t *)(&ptr->SuperCapRemain))[3] = Data[3];
	 ptr->SuperCapRemain2 = (float)(Data[4]<<24 | Data[5]<<16 | Data[6]<<8 | Data[7]);
}
