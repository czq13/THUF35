// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsSingle.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsSingle.h"
#include <RC_Channel/RC_Channel.h>
#include <RC_Channel/RC_Channel_aux.h>
#include <AP_CHuart/AP_CHuart.h>
#include "stdio.h"
extern const AP_HAL::HAL& hal;
extern AP_CHuart chuart;


const AP_Param::GroupInfo AP_MotorsSingle::var_info[] PROGMEM = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // parameters 1 ~ 29 were reserved for tradheli
    // parameters 30 ~ 39 reserved for tricopter
    // parameters 40 ~ 49 for single copter and coax copter (these have identical parameter files)

    // @Param: ROLL_SV_REV
    // @DisplayName: Reverse roll feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Reversed,1:Normal
    AP_GROUPINFO("ROLL_SV_REV", 40, AP_MotorsSingle, _rev_roll, AP_MOTORS_SING_POSITIVE),

    // @Param: PITCH_SV_REV
    // @DisplayName: Reverse pitch feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Reversed,1:Normal
    AP_GROUPINFO("PITCH_SV_REV", 41, AP_MotorsSingle, _rev_pitch, AP_MOTORS_SING_POSITIVE),

	// @Param: YAW_SV_REV
    // @DisplayName: Reverse yaw feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Reversed,1:Normal
    AP_GROUPINFO("YAW_SV_REV", 42, AP_MotorsSingle, _rev_yaw, AP_MOTORS_SING_POSITIVE),

	// @Param: SV_SPEED
    // @DisplayName: Servo speed 
    // @Description: Servo update speed in hz
    // @Values: 50, 125, 250
    AP_GROUPINFO("SV_SPEED", 43, AP_MotorsSingle, _servo_speed, AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS),

    AP_GROUPEND
};
// init
void AP_MotorsSingle::Init()
{
    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the main ESC can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_7] = true;

    // we set four servos to angle
    _servo1.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo2.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo3.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo4.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo1.set_angle(AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);
    _servo2.set_angle(AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);
    _servo3.set_angle(AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);
    _servo4.set_angle(AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);

    // disable CH7 from being used as an aux output (i.e. for camera gimbal, etc)
    RC_Channel_aux::disable_aux_channel(CH_7);
}

// set update rate to motors - a value in hertz
void AP_MotorsSingle::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 3 motors (but not the servo on channel 7)
    uint32_t mask = 
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
		1U << AP_MOTORS_MOT_4 |
        1U << AP_MOTORS_MOT_5 |//澧炲姞閫氶亾5
        1U << AP_MOTORS_MOT_6 ;//澧炲姞閫氶亾6
    hal.rcout->set_freq(mask, _servo_speed);
    uint32_t mask2 = 1U << AP_MOTORS_MOT_7;
    hal.rcout->set_freq(mask2, _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsSingle::enable()
{
    // enable output channels
    hal.rcout->enable_ch(AP_MOTORS_MOT_1);
    hal.rcout->enable_ch(AP_MOTORS_MOT_2);
    hal.rcout->enable_ch(AP_MOTORS_MOT_3);
    hal.rcout->enable_ch(AP_MOTORS_MOT_4);
	hal.rcout->enable_ch(AP_MOTORS_MOT_5);//澧炲姞5閫氶亾浣胯兘
	hal.rcout->enable_ch(AP_MOTORS_MOT_6);//澧炲姞6閫氶亾浣胯兘
    hal.rcout->enable_ch(AP_MOTORS_MOT_7);
}

// output_min - sends minimum values out to the motor and trim values to the servos
void AP_MotorsSingle::output_min()
{
    // send minimum value to each motor
    hal.rcout->write(AP_MOTORS_MOT_1, _servo1.radio_min);
    hal.rcout->write(AP_MOTORS_MOT_2, _servo2.radio_min);
    hal.rcout->write(AP_MOTORS_MOT_3, _servo3.radio_min);
    hal.rcout->write(AP_MOTORS_MOT_4, _servo4.radio_min);
	hal.rcout->write(AP_MOTORS_MOT_5, 1000);//璁剧疆5閫氶亾鏈�灏忓��1000
	hal.rcout->write(AP_MOTORS_MOT_6, 1000);//璁剧疆6閫氶亾鏈�灏忓��1000
    hal.rcout->write(AP_MOTORS_MOT_7, _throttle_radio_min);
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsSingle::get_motor_mask()
{
    // single copter uses channels 1,2,3,4 and 7///////澧炲姞5,6/////
    return (1U << 0 | 1U << 1 | 1U << 2 | 1U << 3 | 1U << 4 |1U << 5 |1U << 6);
}

void AP_MotorsSingle::output_armed_not_stabilizing()
{
    int16_t throttle_radio_output;                                  // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t out_min = _throttle_radio_min + _min_throttle;

    // initialize limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    int16_t thr_in_min = rel_pwm_to_thr_range(_spin_when_armed_ramped);
    if (_throttle_control_input <= thr_in_min) {
        _throttle_control_input = thr_in_min;
        limit.throttle_lower = true;
    }
    if (_throttle_control_input >= _max_throttle) {
        _throttle_control_input = _max_throttle;
        limit.throttle_upper = true;
    }

    throttle_radio_output = calc_throttle_radio_output();

    // front servo
    _servo1.servo_out = 0;
    // right servo
    _servo2.servo_out = 0;
    // rear servo
    _servo3.servo_out = 0;
    // left servo
    _servo4.servo_out = 0;

    _servo1.calc_pwm();
    _servo2.calc_pwm();
    _servo3.calc_pwm();
    _servo4.calc_pwm();

    if (throttle_radio_output >= out_min) {
        throttle_radio_output = apply_thrust_curve_and_volt_scaling(throttle_radio_output, out_min, _throttle_radio_max);
    }

    hal.rcout->write(AP_MOTORS_MOT_1, _servo1.radio_out);
    hal.rcout->write(AP_MOTORS_MOT_2, _servo2.radio_out);
    hal.rcout->write(AP_MOTORS_MOT_3, _servo3.radio_out);
    hal.rcout->write(AP_MOTORS_MOT_4, _servo4.radio_out);
    hal.rcout->write(AP_MOTORS_MOT_7, throttle_radio_output);
}

// sends commands to the motors
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
static int16_t rc_5_i;
static int16_t rc_6_i;
static double a_posc2[4]={0,0,0,0};
double a_pos_pre2; //第二级舵机的位置预测值
void AP_MotorsSingle::output_armed_stabilizing()
{
    int16_t throttle_radio_output;                                  // total throttle pwm value, summed onto throttle channel minimum, typically ~1100-1900
    int16_t out_min = _throttle_radio_min + _min_throttle;

    // initialize limits flags
    limit.roll_pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // Throttle is 0 to 1000 only
    int16_t thr_in_min = rel_pwm_to_thr_range(_min_throttle);
    if (_throttle_control_input <= thr_in_min) {
        _throttle_control_input = thr_in_min;
            limit.throttle_lower = true;
        }
    if (_throttle_control_input >= _max_throttle) {
        _throttle_control_input = _max_throttle;
        limit.throttle_upper = true;
    }

    // calculate throttle PWM
    throttle_radio_output = calc_throttle_radio_output();

    // adjust for thrust curve and voltage scaling
    throttle_radio_output = apply_thrust_curve_and_volt_scaling(throttle_radio_output, out_min, _throttle_radio_max);

    // ensure motor doesn't drop below a minimum value and stop
      throttle_radio_output = max(throttle_radio_output, out_min);
	
	/*******************************************
     * 娣诲姞鑰�:THU zb
     * 鎻忚堪:roll,pitch,yaw pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400
	 * 鎻忚堪:杈撳嚭roll_out,pitch_out,yaw_out
     * 淇敼鏃ユ湡锛�2016/5/16
     ******************************************* */
       if (_throttle_control_input == 0) 
	   {
        // range check spin_when_armed
        if (_spin_when_armed_ramped < 0) {
             _spin_when_armed_ramped = 0;
        }
        if (_spin_when_armed_ramped > _min_throttle) {
            _spin_when_armed_ramped = _min_throttle;
        }
         hal.rcout->write(AP_MOTORS_MOT_1,   _throttle_radio_min +_min_throttle);
         hal.rcout->write(AP_MOTORS_MOT_2,   _throttle_radio_min +_min_throttle);
		 hal.rcout->write(AP_MOTORS_MOT_3,   _throttle_radio_min +_min_throttle);
		 hal.rcout->write(AP_MOTORS_MOT_4,   _throttle_radio_min +_min_throttle);
	   
        // Every thing is limited
        
        limit.throttle_lower = true;
	   }
	  
	  else{
		   
	   
    int16_t roll_pwm;                                               // roll pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400
    int16_t pitch_pwm;                                              // pitch pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400  
    int16_t yaw_pwm;     											// yaw pwm value, initially calculated by calc_roll_pwm() but may be modified after, +/- 400  
	double a_nozzle = 25/57.3;
	double a1_nozzle; //喷管第一级转动角度值
	double a2_nozzle; //喷管第二级转动角度值
	double Ky_nozzle; //喷管左右偏转角度，如果保持喷管从垂直到水平变化，不左右偏这个值为零，若左右偏Ky度，实际喷管就偏Ky度
	double Kn_nozzle; //喷管随动角度
	double  c1_nozzle; //一级舵机输入角度
	double  c2_nozzle; //二级舵机输入角度

	
	roll_pwm  = calc_roll_pwm();
    pitch_pwm = calc_pitch_pwm();
    yaw_pwm   = calc_yaw_pwm();
	
  
    static RC_Channel rc_5(CH_5);
    static RC_Channel rc_6(CH_6);
    static RC_Channel rc_7(CH_7);
	
	rc_5.calc_pwm();
    rc_6.calc_pwm();
	rc_7.calc_pwm();
    rc_5.radio_out = rc_5.radio_in;
	rc_6.radio_out = rc_6.radio_in;
	
	

	
		// send output to each motor,1-4閫氶亾瀹炵幇+褰㈠紡鏃嬬考妯″紡
    hal.rcout->write(AP_MOTORS_MOT_1, throttle_radio_output - roll_pwm);
    hal.rcout->write(AP_MOTORS_MOT_2, throttle_radio_output + roll_pwm);
    hal.rcout->write(AP_MOTORS_MOT_3, throttle_radio_output + pitch_pwm);
    hal.rcout->write(AP_MOTORS_MOT_4, throttle_radio_output - pitch_pwm + 60);
    hal.rcout->write(AP_MOTORS_MOT_7, 15*yaw_pwm);
	
	 
	
	//鎵嬪姩璋冭妭涓ょ骇鍠风锛�5,6鍚勮嚜鎺у埗涓�绾�
	//hal.rcout->write(AP_MOTORS_MOT_5, rc_5.radio_out);//鎵嬪姩璋冭妭//
	//hal.rcout->write(AP_MOTORS_MOT_6, rc_6.radio_out);//鎵嬪姩璋冭妭//
	
	
	///////////////////////////////////////////////////////////////
	//                  澶у柗绠″瀭鐩磋捣闄嶇▼搴忔                     //
	///////////////////////////////////////////////////////////////
	
	//////防止遥控器输入抖动///////
	if (abs(rc_5_i - rc_5.radio_out) > 2)
		rc_5_i = rc_5.radio_out;
	if (abs(rc_6_i - rc_6.radio_out) > 2)
		rc_6_i = rc_6.radio_out;
	Ky_nozzle = (0.02*(rc_5_i) - 30.0 )/57.3;//遥控器5 1000~2000映射-10~10度 在换成弧度
	Kn_nozzle = (200.0 - 0.1*(rc_6_i) )/57.3;//遥控器6 1000~2000映射100~0度 在换成弧度
	//printf("rc_5=%d,rc_6=%d   ",rc_5.radio_out,rc_6.radio_out);
	if(Kn_nozzle>99.9999/57.3)
	{
		Kn_nozzle=99.9999/57.3;
	}
	if(Kn_nozzle<0.0001)
	{
		Kn_nozzle=0.0001;
	}
    //chuart.setServoCtrl( -(180-a2_nozzle * 57.3)*2.5 , -(90-a1_nozzle * 57.3)*2.55 );
	  
	//chuart.sd[0].pos;//position of actuator 1
    //chuart.sd[1].pos;//position of actuator 2
	
	
	a2_nozzle =  acos((cos(Kn_nozzle/2)-cos(a_nozzle)*cos(a_nozzle))/(sin(a_nozzle)*sin(a_nozzle))) ;//二级喷管角度
	c2_nozzle = -(180-a2_nozzle * 57.3)*2.5; //二级舵机输入角度值

	a_pos_pre2=0.75*chuart.sd[1].pos*0.15+0.25* a_posc2[3];
	a_pos_pre2=0.75*a_pos_pre2+0.25* a_posc2[2];
	a_pos_pre2=0.75*a_pos_pre2+0.25* a_posc2[1];
	a_pos_pre2=0.75*a_pos_pre2+0.25* a_posc2[0];

	
	if((c2_nozzle-a_pos_pre2) > 3.5)
	{
		c2_nozzle = a_pos_pre2 + 3.5;
	}
	else if((c2_nozzle-a_pos_pre2) < -3.5)
	{
		c2_nozzle = a_pos_pre2 -3.5;
	}
    if(c2_nozzle > 0)
    {
        c2_nozzle = 0;
    }
    else if(c2_nozzle < -450)
    {
        c2_nozzle = -450;
    }

	a_posc2[3]=a_posc2[2];
	a_posc2[2]=a_posc2[1];
	a_posc2[1]=a_posc2[0];
	a_posc2[0]=c2_nozzle;

	a2_nozzle= (180+c2_nozzle/2.5)/57.3;
	
    a1_nozzle =  atan(tan(a2_nozzle/2)*cos(a_nozzle)) + Ky_nozzle  ;  	//一级喷管角度
	c1_nozzle =  -(90-a1_nozzle * 57.3-yaw_pwm*1.5)*2.55;//转换成一级舵机输入角度值
	chuart.setServoCtrl( c1_nozzle ,  c2_nozzle );
	hal.rcout->write(AP_MOTORS_MOT_6, int(-(90-a1_nozzle * 57.3-yaw_pwm*1.5)*2.55));//输出偏转62.658~0度映射 1000~1169 后半部分是偏航自动控制调节
	hal.rcout->write(AP_MOTORS_MOT_5, int(-(180-a2_nozzle * 57.3)*2.5));//输出偏转129.78~0度 映射1000~1284
	 }
}

// output_disarmed - sends commands to the motors
void AP_MotorsSingle::output_disarmed()
{
    // Send minimum values to all motors
    output_min();
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsSingle::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // flap servo 1
            hal.rcout->write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // flap servo 2
            hal.rcout->write(AP_MOTORS_MOT_2, pwm);
            break;
        case 3:
            // flap servo 3
            hal.rcout->write(AP_MOTORS_MOT_3, pwm);
            break;
        case 4:
            // flap servo 4
            hal.rcout->write(AP_MOTORS_MOT_4, pwm);
            break;
        case 5:
            // spin main motor
            hal.rcout->write(AP_MOTORS_MOT_7, pwm);
            break;
        default:
            // do nothing
            break;
    }
}
