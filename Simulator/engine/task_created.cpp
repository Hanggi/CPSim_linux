#include "task_created.hh"
#include "can_api.h"
#include "data_list.hh"

extern list<CAN_Msg *> waiting_data;

float last_truck_angle = -1.0;

float last_speed = 0;

float angle_changes = 0;

// Task0 (CC1)
Task0::Task0(Task_info *task_info):Task(task_info)
{
}

Task0::~Task0()
{
}

void Task0::write()
{
	for(unsigned int i = 0; i < successors.size(); i++)
	{
		successors[i]->internal_data[0] = internal_data[0];
// hanggi test: CC1 
successors[i]->internal_data[1] = angle_changes;		last_speed = internal_data[0];
	}
}

void Task0::procedure()
{
	internal_data[0] = car_output[SPEED];
}


// Task1 (LK1)
Task1::Task1(Task_info *task_info):Task(task_info)
{
}

Task1::~Task1()
{
}

void Task1::write()
{
	for(unsigned int i = 0; i < successors.size(); i++)
	{
		successors[i]->internal_data[0] = internal_data[0];
		successors[i]->internal_data[1] = internal_data[1];
	}
}

void Task1::procedure()
{
	internal_data[0] = car_output[TRACK_WIDTH];
	internal_data[1] = car_output[DISTANCE];
}


// Task2 (CC2)
Task2::Task2(Task_info *task_info):Task(task_info)
{
}

Task2::~Task2()
{
}

void Task2::write()
{
	;

	// can send
	CAN_Msg *can_msg = new CAN_Msg(completion_time, 1, 0x7FF, 2, ACCEL, BRAKE, accel, brake, this->task_link->get_task_name());
	insert_can_msg(&waiting_data, can_msg);
// hanggi test: CC1 
}

void Task2::procedure()
{
	float speed = internal_data[0];
	float angle_changes = internal_data[1];
    brake = 0.0;
    float v = 1.0;

    if (angle_changes > 0.99 || angle_changes < -0.99) {
    	angle_changes = 0.00003;
    }

    if (angle_changes == 0) {
    	v = 1.5;
    }




    if(speed < 50.0 * v)
        accel = 1.0;
    else if(speed < 60.0 * v)
        accel = 0.5;
    else if(speed < 75.0 * v)
        accel = 0.3;
    else
        accel = 0.0;

    printf("ang_cha: %f \n", angle_changes);
}


// Task3 (LK2)
Task3::Task3(Task_info *task_info):Task(task_info)
{
}

Task3::~Task3()
{
}

void Task3::write()
{
	for(unsigned int i = 0; i < successors.size(); i++)
	{
		successors[i]->internal_data[2] = internal_data[2];
		successors[i]->internal_data[3] = internal_data[3];
// hanggi test: LK2 
		successors[i]->internal_data[4] = last_truck_angle;
		successors[i]->internal_data[5] = last_speed;
		angle_changes = internal_data[2] - last_truck_angle ;
		last_truck_angle = internal_data[2];
	}
}

void Task3::procedure()
{
	internal_data[2] = car_output[TRACK_ANGLE];
	internal_data[3] = car_output[YAW];
}


// Task4 (other1)
Task4::Task4(Task_info *task_info):Task(task_info)
{
}

Task4::~Task4()
{
}

void Task4::write()
{
	;
}

void Task4::procedure()
{
	;
}


// Task5 (other2)
Task5::Task5(Task_info *task_info):Task(task_info)
{
}

Task5::~Task5()
{
}

void Task5::write()
{
	;
}

void Task5::procedure()
{
	;
}


// Task6 (other3)
Task6::Task6(Task_info *task_info):Task(task_info)
{
}

Task6::~Task6()
{
}

void Task6::write()
{
	;
}

void Task6::procedure()
{
	;
}


// Task7 (LK3)
Task7::Task7(Task_info *task_info):Task(task_info)
{
}

Task7::~Task7()
{
}

void Task7::write()
{
	;

	// can send
	CAN_Msg *can_msg = new CAN_Msg(completion_time, 1, 0x7FE, 1, STEER, STEER, steering, steering, this->task_link->get_task_name());
	insert_can_msg(&waiting_data, can_msg);
// hanggi test: CC1 
}

void Task7::procedure()
{
	// sensor data
	float track_width = internal_data[0];
	float distance = internal_data[1];
	float track_angle = internal_data[2];
	float yaw = internal_data[3];
	float last_track_angle = internal_data[4];
	float last_speed = internal_data[5];

	// get difference of angle
	float angle = track_angle - yaw;
	float pi = 3.141592;
	while(angle > pi)
	    angle -= 2*pi;
	while(angle < -pi)
	    angle += 2*pi;

	// calculate steering value
	float SC = 1.0;

	steering = 0.00001;
	char str[128] = "Init start";

	float changes = track_angle - last_track_angle;

	if (track_angle - last_track_angle > 0.99 || track_angle - last_track_angle < -0.99) {
		steering = angle - SC*distance/track_width;
		strcpy(str, "Changes invalid!");
	} else {
//		steering = track_angle - last_track_angle;
		if (changes < 0.01 && changes > -0.01) {
			if (distance < 1 && distance > -1) {
				if ((distance < 0 && angle > 0) || (distance > 0 && angle < 0)) {
					steering = 0.6 * angle - 0.4*distance/track_width;
					strcpy(str, "On straight! close, out side");
				} else {
					steering = 0.6 * angle - 0.4*distance/track_width;
					strcpy(str, "On straight! close, in side");
				}
			} else {
				steering = 0.9*angle - 0.6*distance/track_width;
				strcpy(str, "On straight! faraway");
			}
		} else {
			if (((distance < 0 && angle > 0) && changes > 0) || ((distance > 0 && angle < 0) && changes < 0)) {
				if (last_speed < 80) {
					steering = 1.1*angle + changes*last_speed/42 - 1.1*distance/track_width;
				} else {
					steering = 1.1*angle + changes*last_speed/70 - 1.1*distance/track_width;
				}

				if (changes > 0) {
					strcpy(str, "In left corner! to out side");
				} else {
					strcpy(str, "In right corner! to out side");
				}
			} else {
				if ((distance < 0 && angle < 0) && changes > 0 || (distance > 0 && angle > 0) && changes < 0) {
					if (last_speed < 80) {
						steering = angle * last_speed/75 + changes*last_speed/42 - 0.3*distance/track_width;
					}else {
						steering = angle * last_speed/140  - 0.4*distance/track_width;
					}

					if (changes > 0) {
						strcpy(str, "In left corner! to in side");
					} else {
						strcpy(str, "In right corner! to in side");
					}
				} else {
					if (last_speed < 80) {
						steering = angle + changes*last_speed/80 - 0.8*distance/track_width;
					}else {
						steering = angle + changes*last_speed/120 - 0.3*distance/track_width;
					}

					if (changes > 0) {
						strcpy(str, "In left corner! to in side 222");
					} else {
						strcpy(str, "In right corner! to in side 222");
					}
				}

			}

		}
	}

	printf("ste: %10f, cha: %10f, dis: %10f -- %s\n",steering, changes, distance, str);
	printf("ang: %10f, dis: %10f, spd: %10f \n\n", angle, distance/track_width, last_speed);
	//		steering = (angle + changes) * last_speed/20;

}


