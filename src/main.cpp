#include <Arduino.h>
#include <SimpleFOC.h>
#include <pinmap_custom.h>

// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
MagneticSensorSPI sensor = MagneticSensorSPI(PA15, 14, 0x3FFF);
SPIClass SPI_3(PB5, PB4, PB3); //(mosi, miso, sclk)

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// current sensor
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// Commander interface constructor
Commander command = Commander(Serial);

void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doMotor(char* cmd){ command.motor(&motor,cmd); }
float normal_zero_angle = 0;

LowPassFilter diff_filter = LowPassFilter(0.05);
float currentlf_now =0;
int main_loop_frequency = 0;

// led control function
void doLed(char* cmd){ 
    if(atoi(cmd)) digitalWrite(LED_BUILTIN, HIGH); 
    else digitalWrite(LED_BUILTIN, LOW); 
};

void doGetFreq(char* cmd){
	Serial.println(main_loop_frequency);
}

void doGetI(char* cmd){
	Serial.println(current_sense.getDCCurrent(motor.electrical_angle));
}

void setup(){
	pinMode(LED_BUILTIN, OUTPUT);

	Serial.begin(115200);
	delay(100);

	sensor.init(&SPI_3);
	motor.linkSensor(&sensor);

	// driver config
	driver.voltage_power_supply = 15;
	driver.init();

	// link the driver to the current sense
 	current_sense.linkDriver(&driver);

	// link the motor and the driver
	motor.linkDriver(&driver);

	// limiting motor movements
	motor.phase_resistance = 0.251; // [Ohm]
	motor.phase_inductance = 0.0001; // [Ohm]
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
	motor.current_limit = 20;   // [Amps] - if phase resistance defined
	// motor.voltage_limit = 1;   // [V] - if phase resistance not defined
	motor.velocity_limit = 100; // [rad/s] cca 50rpm

	// aligning voltage
	motor.voltage_sensor_align = 1;

	// set torque mode to be used
	motor.torque_controller = TorqueControlType::voltage;   // ( default )
	// motor.torque_controller = TorqueControlType::dc_current;
	// motor.torque_controller = TorqueControlType::foc_current;

	// set motion control loop to be used
	// motor.controller = MotionControlType::torque; //      - torque control 
	motor.controller = MotionControlType::velocity; //    - velocity motion control
	// motor.controller = MotionControlType::angle; //       - position/angle motion control
	// motor.controller = MotionControlType::velocity_openloop; //    - velocity open-loop control
	// motor.controller = MotionControlType::angle_openloop; //       - position open-loop control

	motor.motion_downsample = 3.0; //gives us 10% speed range.

	motor.useMonitoring(Serial);
	motor.monitor_downsample = 1000; // set downsampling can be even more > 100
  	motor.monitor_variables =  _MON_TARGET | _MON_CURR_Q | _MON_VOLT_Q | _MON_CURR_D | _MON_VOLT_D ; // set monitoring of d and q currents _MON_TARGET | _MON_VEL | _MON_ANGLE |
	motor.monitor_decimals = 2; //!< monitor outputs decimal places


	motor.PID_velocity.P = 0.04;
	motor.PID_velocity.I = 1;
	motor.PID_velocity.D = 0;
	motor.PID_velocity.output_ramp = 1000;
	motor.LPF_velocity.Tf = 0.005;
	motor.PID_current_q.P = 0.5;
	motor.P_angle.P = 60.0;

	// init motor hardware
	motor.init();

	// current sense init hardware
	current_sense.init();
	// link the current sense to the motor
	motor.linkCurrentSense(&current_sense);

	  // align sensor and start FOC
	motor.initFOC();

	// set the initial motor target
	motor.target = 0; // unit depends on control mode 

	// add target command T
	command.add('L', doLed, "led on/off");
	command.add('F', doGetFreq, "get frequency of main loop");
	command.add('T', doTarget, "target angle");
	command.add('M',doMotor,"my motor motion");
	command.add('I',doGetI,"my motor motion");
	
	_delay(100);

	normal_zero_angle = motor.zero_electric_angle;

}



void loop(){

	static int i = 0;
	static int j = 0;
	static int millis_prev = 0;
	i ++;
	j ++;


	if(millis()-millis_prev>1000){
		main_loop_frequency = i;
		i=0;
		millis_prev = millis();
	}

	motor.loopFOC();
	motor.move();
	// motor.monitor(); 
	command.run();
	
	if(j%10 == 0){
		// motor.zero_electric_angle=-0.0000007*pow(motor.shaft_velocity_sp,2) + 0.0001*motor.shaft_velocity_sp + normal_zero_angle + 0.2;
		motor.zero_electric_angle=-0.0000005*pow(motor.shaft_velocity_sp,2) + 0.0008*motor.shaft_velocity_sp + normal_zero_angle + 0.3;
	}
}