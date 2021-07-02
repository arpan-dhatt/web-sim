unsigned int acc_x_o = 7;
unsigned int acc_y_o = 8;
unsigned int acc_z_o = 9;

unsigned int ang_x_o = 10;
unsigned int ang_y_o = 11;
unsigned int ang_z_o = 12;

float sensor_data[16] = { 0 };
float esc_data[4] = { 0.0 };

extern "C" float test() {
	return sensor_data[acc_y_o];
}

extern "C" float* sensor_data_ptr() {
	return sensor_data;
}

extern "C" float* esc_data_ptr() {
	return esc_data;
}

extern "C" void execute() {

}
