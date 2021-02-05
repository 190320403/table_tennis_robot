#include "ros/ros.h"
//#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "serial/serial.h"
#include "iostream"
#include"stdio.h"

uint8_t data_to_send[10]={0};
uint8_t data_to_received[10]={0};
uint16_t pulses_to_send[5]={0};
uint16_t pulses_to_show[5]={0};
float robotsim_angles [5];

serial::Serial my_serial_port;

void map_angles_to_pulses(float received_angles[5])
{
	pulses_to_send[0]=((uint16_t)(((-received_angles[0]+1.57)/3.1415)*2000))+500;
	pulses_to_send[1]=((uint16_t)(((-received_angles[1]+1.57)/3.1415)*2000))+500;
	pulses_to_send[2]=((uint16_t)(((-received_angles[2]+2.442)/3.1415)*2000))+500;
	pulses_to_send[3]=((uint16_t)(((-received_angles[3]+1.6929)/3.1415)*2000))+500;
	pulses_to_send[4]=((uint16_t)(((-received_angles[4]+1.57)/3.1415)*2000))+500;

}

void pack_array (uint16_t array_to_be_packed[],uint8_t pack_box[])
{
	
	for(int i=0;i<5;i++)
	{
		pack_box[2*i]=(uint8_t)(array_to_be_packed[i]/100);
		pack_box[2*i+1]=(uint8_t)(array_to_be_packed[i]%(pack_box[2*i]*100));
	}
}

void unpack_array (uint16_t unpacked_box[],uint8_t array_to_be_unpacked[])
{
	for(int i=0;i<5;i++)
	{
		unpacked_box[i]=array_to_be_unpacked[2*i]*100+array_to_be_unpacked[2*i+1];
	}
}



void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	my_serial_port.open();
	//my_serial_port.flushInput();
	for (int i=0;i<5;i++)
	{
		robotsim_angles[i]=msg->position[i+1];
	}
        
	
	map_angles_to_pulses(robotsim_angles);

	pack_array(pulses_to_send,data_to_send);
	int a=my_serial_port.write(data_to_send,10);
	ROS_INFO("writen length =%d",a);
	ros::Duration(0.09).sleep();
	while(1)
        {
		
		if(my_serial_port.available()==10)
		{
			my_serial_port.read(data_to_received,10);
			ROS_INFO("successed ,current bytes is %d",my_serial_port.available());
			my_serial_port.flushInput();
			break;
		}
		else 
		{
			ROS_INFO("current bytes is %d",my_serial_port.available());
			ros::Duration(0.5).sleep();
			break;

		}
		
       }

	unpack_array(pulses_to_show,data_to_received);
	
	for(int i=0;i<5;i++)
	{
		ROS_INFO("pulses_to_show[%d]=%d",i,pulses_to_show[i]);
	}
	my_serial_port.close();
	

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "serial_to_stm_node");
	ros::NodeHandle nh;
	my_serial_port.setPort("/dev/ttyACM0");     //set the port
	my_serial_port.setBaudrate(57600);
	serial::Timeout timeout_set =serial::Timeout::simpleTimeout(1000);
	my_serial_port.setTimeout(timeout_set);
	my_serial_port.close();
	ros::Duration(2).sleep();

	if(my_serial_port.isOpen())
        {
		std::cout<<"opened"<<std::endl;
	}
	else
	{
		std::cout<<"not opened"<<std::endl;
	}
	ros::Subscriber pose_suber = nh.subscribe("/virtual_arm/joint_states", 5, chatterCallback);
	ros::spin();
	return 0;
}

