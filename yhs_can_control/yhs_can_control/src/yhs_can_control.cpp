#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>


#include <boost/bind.hpp>
#include <boost/thread.hpp>


#include "yhs_can_control.h"


namespace yhs_tool {


CanControl::CanControl()
{
	ros::NodeHandle private_node("~");
	
}


CanControl::~CanControl()
{

}

//io控制回调函数
void CanControl::io_cmdCallBack(const yhs_can_msgs::io_cmd msg)
{
	static unsigned char count_1 = 0;

	cmd_mutex_.lock();

	memset(sendData_u_io_,0,8);

	sendData_u_io_[0] = msg.io_cmd_enable;

	sendData_u_io_[1] = 0xff;
	if(msg.io_cmd_upper_beam_headlamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xfd;

	if(msg.io_cmd_turn_lamp == 0)
		sendData_u_io_[1] &= 0xf3;
	if(msg.io_cmd_turn_lamp == 1)
		sendData_u_io_[1] &= 0xf7;
	if(msg.io_cmd_turn_lamp == 2)
		sendData_u_io_[1] &= 0xfb;

	sendData_u_io_[2] = msg.io_cmd_speaker;

	sendData_u_io_[3] = 0;
	sendData_u_io_[4] = 0;
	sendData_u_io_[5] = 0;

	count_1 ++;
	if(count_1 == 16)	count_1 = 0;

	sendData_u_io_[6] =  count_1 << 4;

	sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

	send_frames_[0].can_id = 0x98C4D7D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_io_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }	

	cmd_mutex_.unlock();
}

//速度控制回调函数
void CanControl::ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg)
{
	unsigned short vel = 0;
	short angular = msg.ctrl_cmd_steering * 100;
	static unsigned char count = 0;

	cmd_mutex_.lock();

	memset(sendData_u_vel_,0,8);

	if(msg.ctrl_cmd_velocity < 0) vel = 0;
	else
	{
		vel = msg.ctrl_cmd_velocity * 1000;
	}

	sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((vel & 0x0f) << 4));

	sendData_u_vel_[1] = (vel >> 4) & 0xff;

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (vel >> 12));

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendData_u_vel_[3] = (angular >> 4) & 0xff;

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0xf0 & ((msg.ctrl_cmd_Brake & 0x0f) << 4));

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0x0f & (angular >> 12));

	sendData_u_vel_[5] = 0;

	count ++;

	if(count == 16)	count = 0;

	sendData_u_vel_[6] =  count << 4;
	

	sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

	send_frames_[0].can_id = 0x98C4D2D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_vel_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      ROS_ERROR("send message failed, error code: %d",ret);
    }	

	cmd_mutex_.unlock();
}

//数据接收解析线程
void CanControl::recvData()
{

	while(ros::ok())
	{

		if(read(dev_handler_, &recv_frames_[0], sizeof(recv_frames_[0])) >= 0)
		{
			for(int j=0;j<1;j++)
			{
				
				switch (recv_frames_[0].can_id)
				{
					//速度控制反馈
					case 0x98C4D2EF:
					{
						yhs_can_msgs::ctrl_fb msg;
						msg.ctrl_fb_gear = 0x0f & recv_frames_[0].data[0];
						
						msg.ctrl_fb_velocity = (float)((unsigned short)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4)) / 1000;
						
						msg.ctrl_fb_steering = (float)((short)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4)) / 100;

						msg.ctrl_fb_Brake = (recv_frames_[0].data[4] & 0x30) >> 4;
						
						msg.ctrl_fb_mode = (recv_frames_[0].data[4] & 0xc0) >> 6;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							ctrl_fb_pub_.publish(msg);
						}

						break;
					}

					//左轮反馈
					case 0x98C4D7EF:
					{
						yhs_can_msgs::lr_wheel_fb msg;
						msg.lr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.lr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							lr_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//右轮反馈
					case 0x98C4D8EF:
					{
						yhs_can_msgs::rr_wheel_fb msg;
						msg.rr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
						msg.rr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							rr_wheel_fb_pub_.publish(msg);
						}

						break;
					}

					//io反馈
					case 0x98C4DAEF:
					{
						yhs_can_msgs::io_fb msg;
						if(0x01 & recv_frames_[0].data[0]) msg.io_fb_enable = true;	else msg.io_fb_enable = false;
	
						if(0x02 & recv_frames_[0].data[1]) msg.io_fb_upper_beam_headlamp = true;	else msg.io_fb_upper_beam_headlamp = false;

						msg.io_fb_turn_lamp = (0x0c & recv_frames_[0].data[1]) >> 2;

						if(0x10 & recv_frames_[0].data[1]) msg.io_fb_braking_lamp = true;	else msg.io_fb_braking_lamp = false;

						if(0x01 & recv_frames_[0].data[2]) msg.io_fb_speaker = true;	else msg.io_fb_speaker = false;

						if(0x02 & recv_frames_[0].data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;

						if(0x10 & recv_frames_[0].data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							io_fb_pub_.publish(msg);
						}

						break;
					}

					//里程计反馈
					case 0x98C4DEEF:
					{
						yhs_can_msgs::odo_fb msg;
						msg.odo_fb_accumulative_mileage = (float)((int)(recv_frames_[0].data[3] << 24 | recv_frames_[0].data[2] << 16 | recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;

						msg.odo_fb_accumulative_angular = (float)((int)(recv_frames_[0].data[7] << 24 | recv_frames_[0].data[6] << 16 | recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 1000;

						odo_fb_pub_.publish(msg);

						break;
					}

					//bms_Infor反馈
					case 0x98C4E1EF:
					{
						yhs_can_msgs::bms_Infor_fb msg;
						msg.bms_Infor_voltage = (float)((unsigned short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;

						msg.bms_Infor_current = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;

						msg.bms_Infor_remaining_capacity = (float)((unsigned short)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 100;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							bms_Infor_fb_pub_.publish(msg);
						}

						break;
					}

					//bms_flag_Infor反馈
					case 0x98C4E2EF:
					{
						yhs_can_msgs::bms_flag_Infor_fb msg;
						msg.bms_flag_Infor_soc = recv_frames_[0].data[0];

						if(0x01 & recv_frames_[0].data[1]) msg.bms_flag_Infor_single_ov = true;	else msg.bms_flag_Infor_single_ov = false;

						if(0x02 & recv_frames_[0].data[1]) msg.bms_flag_Infor_single_uv = true;	else msg.bms_flag_Infor_single_uv = false;

						if(0x04 & recv_frames_[0].data[1]) msg.bms_flag_Infor_ov = true;	else msg.bms_flag_Infor_ov = false;

						if(0x08 & recv_frames_[0].data[1]) msg.bms_flag_Infor_uv = true;	else msg.bms_flag_Infor_uv = false;

						if(0x10 & recv_frames_[0].data[1]) msg.bms_flag_Infor_charge_ot = true;	else msg.bms_flag_Infor_charge_ot = false;

						if(0x20 & recv_frames_[0].data[1]) msg.bms_flag_Infor_charge_ut = true;	else msg.bms_flag_Infor_charge_ut = false;

						if(0x40 & recv_frames_[0].data[1]) msg.bms_flag_Infor_discharge_ot = true;	else msg.bms_flag_Infor_discharge_ot = false;

						if(0x80 & recv_frames_[0].data[1]) msg.bms_flag_Infor_discharge_ut = true;	else msg.bms_flag_Infor_discharge_ut = false;

						if(0x01 & recv_frames_[0].data[2]) msg.bms_flag_Infor_charge_oc = true;	else msg.bms_flag_Infor_charge_oc = false;

						if(0x02 & recv_frames_[0].data[2]) msg.bms_flag_Infor_discharge_oc = true;	else msg.bms_flag_Infor_discharge_oc = false;

						if(0x04 & recv_frames_[0].data[2]) msg.bms_flag_Infor_short = true;	else msg.bms_flag_Infor_short = false;

						if(0x08 & recv_frames_[0].data[2]) msg.bms_flag_Infor_ic_error = true;	else msg.bms_flag_Infor_ic_error = false;

						if(0x10 & recv_frames_[0].data[2]) msg.bms_flag_Infor_lock_mos = true;	else msg.bms_flag_Infor_lock_mos = false;

						if(0x20 & recv_frames_[0].data[2]) msg.bms_flag_Infor_charge_flag = true;	else msg.bms_flag_Infor_charge_flag = false;

						msg.bms_flag_Infor_hight_temperature = (float)((short)(recv_frames_[0].data[4] << 4 | recv_frames_[0].data[3] >> 4)) / 10;

						msg.bms_flag_Infor_low_temperature = (float)((short)((recv_frames_[0].data[6] & 0x0f) << 8 | recv_frames_[0].data[5])) / 10;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							bms_flag_Infor_fb_pub_.publish(msg);
						}

						break;
					}

					//Drive_fb_MCUEcoder反馈
					case 0x98C4DCEF:
					{
						yhs_can_msgs::Drive_MCUEcoder_fb msg;
						msg.Drive_fb_MCUEcoder = (int)(recv_frames_[0].data[3] << 24 | recv_frames_[0].data[2] << 16 | recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0]); 

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							Drive_MCUEcoder_fb_pub_.publish(msg);
						}

						break;
					}

					//Veh_fb_Diag反馈
					case 0x98C4EAEF:
					{
						yhs_can_msgs::Veh_Diag_fb msg;
						msg.Veh_fb_FaultLevel = 0x0f & recv_frames_[0].data[0];

						if(0x10 & recv_frames_[0].data[0]) msg.Veh_fb_AutoCANCtrlCmd = true;	else msg.Veh_fb_AutoCANCtrlCmd = false;

						if(0x20 & recv_frames_[0].data[0]) msg.Veh_fb_AutoIOCANCmd = true;	else msg.Veh_fb_AutoIOCANCmd = false;

						if(0x01 & recv_frames_[0].data[1]) msg.Veh_fb_EPSDisOnline = true;	else msg.Veh_fb_EPSDisOnline = false;

						if(0x02 & recv_frames_[0].data[1]) msg.Veh_fb_EPSfault = true;	else msg.Veh_fb_EPSfault = false;

						if(0x04 & recv_frames_[0].data[1]) msg.Veh_fb_EPSMosfetOT = true;	else msg.Veh_fb_EPSMosfetOT = false;

						if(0x08 & recv_frames_[0].data[1]) msg.Veh_fb_EPSWarning = true;	else msg.Veh_fb_EPSWarning = false;

						if(0x10 & recv_frames_[0].data[1]) msg.Veh_fb_EPSDisWork = true;	else msg.Veh_fb_EPSDisWork = false;

						if(0x20 & recv_frames_[0].data[1]) msg.Veh_fb_EPSOverCurrent = true;	else msg.Veh_fb_EPSOverCurrent = false;

						
						
						if(0x10 & recv_frames_[0].data[2]) msg.Veh_fb_EHBecuFault = true;	else msg.Veh_fb_EHBecuFault = false;

						if(0x20 & recv_frames_[0].data[2]) msg.Veh_fb_EHBDisOnline = true;	else msg.Veh_fb_EHBDisOnline = false;

						if(0x40 & recv_frames_[0].data[2]) msg.Veh_fb_EHBWorkModelFault = true;	else msg.Veh_fb_EHBWorkModelFault = false;

						if(0x80 & recv_frames_[0].data[2]) msg.Veh_fb_EHBDisEn = true;	else msg.Veh_fb_EHBDisEn = false;


						if(0x01 & recv_frames_[0].data[3]) msg.Veh_fb_EHBAnguleFault = true;	else msg.Veh_fb_EHBAnguleFault = false;

						if(0x02 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOT = true;	else msg.Veh_fb_EHBOT = false;

						if(0x04 & recv_frames_[0].data[3]) msg.Veh_fb_EHBPowerFault = true;	else msg.Veh_fb_EHBPowerFault = false;

						if(0x08 & recv_frames_[0].data[3]) msg.Veh_fb_EHBsensorAbnomal = true;	else msg.Veh_fb_EHBsensorAbnomal = false;

						if(0x10 & recv_frames_[0].data[3]) msg.Veh_fb_EHBMotorFault = true;	else msg.Veh_fb_EHBMotorFault = false;

						if(0x20 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOilPressSensorFault = true;	else msg.Veh_fb_EHBOilPressSensorFault = false;

						if(0x40 & recv_frames_[0].data[3]) msg.Veh_fb_EHBOilFault = true;	else msg.Veh_fb_EHBOilFault = false;




						if(0x01 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUDisOnline = true;	else msg.Veh_fb_DrvMCUDisOnline = false;

						if(0x02 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUOT = true;	else msg.Veh_fb_DrvMCUOT = false;

						if(0x04 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUOV = true;	else msg.Veh_fb_DrvMCUOV = false;

						if(0x08 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUUV = true;	else msg.Veh_fb_DrvMCUUV = false;

						if(0x10 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUShort = true;	else msg.Veh_fb_DrvMCUShort = false;

						if(0x20 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUScram = true;	else msg.Veh_fb_DrvMCUScram = false;

						if(0x40 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUHall = true;	else msg.Veh_fb_DrvMCUHall = false;

						if(0x80 & recv_frames_[0].data[4]) msg.Veh_fb_DrvMCUMOSFEF = true;	else msg.Veh_fb_DrvMCUMOSFEF = false;


						if(0x10 & recv_frames_[0].data[5]) msg.Veh_fb_AUXBMSDisOnline = true;	else msg.Veh_fb_AUXBMSDisOnline = false;

						if(0x20 & recv_frames_[0].data[5]) msg.Veh_fb_AuxScram = true;	else msg.Veh_fb_AuxScram = false;

						if(0x40 & recv_frames_[0].data[5]) msg.Veh_fb_AuxRemoteClose = true;	else msg.Veh_fb_AuxRemoteClose = false;

						if(0x80 & recv_frames_[0].data[5]) msg.Veh_fb_AuxRemoteDisOnline = true;	else msg.Veh_fb_AuxRemoteDisOnline = false;

						unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

						if(crc == recv_frames_[0].data[7])
						{
								
							Veh_Diag_fb_pub_.publish(msg);
						}


						break;
					}

					//ultrasonic 301~304

					static yhs_can_msgs::ultrasonic ulmsg;

					case 0x301:
					{
						ulmsg.front_left = recv_frames_[0].data[1] * 2 > 255 ? 255:recv_frames_[0].data[1] * 2;
						ulmsg.front_right = recv_frames_[0].data[2] * 2 > 255 ? 255:recv_frames_[0].data[2] * 2;
						break;
					}

					case 0x302:
					{
						ulmsg.rear_left = recv_frames_[0].data[1] * 2 > 255 ? 255:recv_frames_[0].data[1] * 2;
						ulmsg.rear_right = recv_frames_[0].data[2] * 2 > 255 ? 255:recv_frames_[0].data[2] * 2;
						break;
					}

					case 0x303:
					{
						ulmsg.left_front = recv_frames_[0].data[1] * 2 > 255 ? 255:recv_frames_[0].data[1] * 2;
						ulmsg.left_rear = recv_frames_[0].data[2] * 2 > 255 ? 255:recv_frames_[0].data[2] * 2;
						break;
					}

					case 0x304:
					{
						ulmsg.right_front = recv_frames_[0].data[1] * 2 > 255 ? 255:recv_frames_[0].data[1] * 2;
						ulmsg.right_rear = recv_frames_[0].data[2] * 2 > 255 ? 255:recv_frames_[0].data[2] * 2;

						ultrasonic_pub_.publish(ulmsg);
						break;
					}
					

					default:
						break;
				}

			}

					
		}
	}
}

//数据发送线程
void CanControl::sendData()
{
	ros::Rate loop(100);


	while(ros::ok())
	{

		loop.sleep();
	}

}


void CanControl::run()
{

	ctrl_cmd_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 5, &CanControl::ctrl_cmdCallBack, this);
	io_cmd_sub_ = nh_.subscribe<yhs_can_msgs::io_cmd>("io_cmd", 5, &CanControl::io_cmdCallBack, this);

	ctrl_fb_pub_ = nh_.advertise<yhs_can_msgs::ctrl_fb>("ctrl_fb",5);
	lr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::lr_wheel_fb>("lr_wheel_fb",5);
	rr_wheel_fb_pub_ = nh_.advertise<yhs_can_msgs::rr_wheel_fb>("rr_wheel_fb",5);
	io_fb_pub_ = nh_.advertise<yhs_can_msgs::io_fb>("io_fb",5);
	odo_fb_pub_ = nh_.advertise<yhs_can_msgs::odo_fb>("odo_fb",5);
	bms_Infor_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_Infor_fb>("bms_Infor_fb",5);
	bms_flag_Infor_fb_pub_ = nh_.advertise<yhs_can_msgs::bms_flag_Infor_fb>("bms_flag_Infor_fb",5);
	Drive_MCUEcoder_fb_pub_ = nh_.advertise<yhs_can_msgs::Drive_MCUEcoder_fb>("Drive_MCUEcoder_fb",5);
	Veh_Diag_fb_pub_ = nh_.advertise<yhs_can_msgs::Veh_Diag_fb>("Veh_Diag_fb",5);

    ultrasonic_pub_ = nh_.advertise<yhs_can_msgs::ultrasonic>("ultrasonic",5);

	//打开设备
	dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (dev_handler_ < 0) 
	{
		ROS_ERROR(">>open can deivce error!");
		return;
	}
    else
	{
		ROS_INFO(">>open can deivce success!");
	}


	struct ifreq ifr;
	
	std::string can_name("can0");

	strcpy(ifr.ifr_name,can_name.c_str());

	ioctl(dev_handler_,SIOCGIFINDEX, &ifr);


    // bind socket to network interface
	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	int ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr),sizeof(addr));
	if (ret < 0) 
	{
		ROS_ERROR(">>bind dev_handler error!\r\n");
		return;
	}

	//创建接收发送数据线程
	boost::thread recvdata_thread(boost::bind(&CanControl::recvData, this));
//	boost::thread senddata_thread(boost::bind(&CanControl::sendData, this));

	ros::spin();
	
	close(dev_handler_);
}

}



//主函数
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "yhs_can_control_node");

	yhs_tool::CanControl cancontrol;
	cancontrol.run();

	return 0;
}
