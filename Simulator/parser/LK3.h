var:1
float steering;

input:0

output:0

CAN_input:0

CAN_output:1
10:0x7FE:1
steering STEER

code:
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
