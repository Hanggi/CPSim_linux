var:2
float accel;
float brake;

input:0

output:0

CAN_input:0

CAN_output:1
10:0x7FF:2
accel ACCEL
brake BRAKE

code:
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
