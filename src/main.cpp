//   1/5/24
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"


//Motors and defining functions
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor FL(12,true);
pros::Motor MidL(11,true);
pros::Motor BL(2,true);
pros::Motor FR(18,false);
pros::Motor MidR(20,false);
pros::Motor BR(9,false);
pros::Motor intake(15,false);
pros::Motor puncher(14,false);
pros::Imu Gyro(1); // port for inertial
pros::ADIDigitalOut lift('h', LOW);
pros::ADIDigitalOut wings('g', LOW);
pros::Rotation rot(21);
   
//Drivetrain motor groups
pros::Motor_Group leftDb({FL,MidL, BL});
  pros::Motor_Group rightDb({FR,MidR,BR});




lemlib::Drivetrain_t drivetrain {
    &leftDb, // left drivetrain motors
    &rightDb, // right drivetrain motors
    10, // track width
    3.25, // wheel diameter
    400 // wheel rpm
    };
 


// Odom construct
    lemlib::OdomSensors_t sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &Gyro // inertial sensor
    };


// forward/backward PID
    lemlib::ChassisController_t lateralController {
    2.75, // kPs
    50, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    25 // slew rate
    };
 
// turning PID wow
    lemlib::ChassisController_t angularController {
    8.7, // kP
    52, // kD
    1, // smallErrorRange
    200, // smallErrorTimeout
    3, // largeErrorRange
    700, // largeErrorTimeout
    25 // slew rate
    };


// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);


/**
 * A callback function for  LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}


void screen() {
    while (true) {
        auto pose = chassis.getPose();
        pros::lcd::print(0, "X: %f", pose.x);
        pros::lcd::print(1, "Y: %f", pose.y);
        pros::lcd::print(2, "Heading: %f", pose.theta);
        pros::delay(20);
    }
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
    pros::Task screenDisplay(screen);
    chassis.calibrate();
  pros::lcd::set_text(1, "Hello PROS User!");


  pros::lcd::register_btn1_cb(on_center_button);
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}


/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
 /***/
 /** auto for league games
 void autonomous() {
    chassis.moveTo(0,-150,600);
    pros::delay(600);
    chassis.moveTo(0,60,600);
}
 
 */


void turn(double theta){
    double x = 10000 * (cos(theta * (M_PI / 180.0) + chassis.getPose().x));
    double y = 10000 * (sin(theta * (M_PI / 180.0) + chassis.getPose().y));
    chassis.turnTo(x,y,1000);


}
void autonomous() {


    // Match Far Auton --------------------------------------------------------------------------
     
   
    // AWP Close -------------------------------------------------------------------------------
    /*
    chassis.moveTo(0,-900,600);
    pros::delay(600);
    wings.set_value(HIGH);
    chassis.moveTo(0,10,600);
    pros::delay(600);
    chassis.turnTo(-33,17.7, 600);
    wings.set_value(LOW);
    pros::delay(600);
    chassis.moveTo(-34.25,40,600);
    chassis.moveTo(-55.4,16.6,600);
   
    pros::delay(7000);
    flydeck = 118;
    */


    //Auton skills
    //flydeck = 118;
 
}
   
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
//cata motor loop and configuration
void opcontrol() {




 
 while (true) {
  pros::delay(20);
  double power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  leftDb.move(power + turn);
  rightDb.move(power - turn);
 


//Puncher
   


//Lift
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        lift.set_value(HIGH);
        puncher = 128;
    }
   
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        lift.set_value(LOW);
        if(lift.set_value(LOW) == true ){
            double angle = rot.get_angle()/100.0;
            pros::lcd::print(0, "Angle: ", angle);
            if (angle < 46) {
                puncher.move_velocity(100);
            }
            else {
                puncher = 0;
            }
               
        }


       


        pros::delay(10);
    }


//WINGS
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
        wings.set_value(HIGH);
    }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
        wings.set_value(LOW);
       
        pros::delay(10);
    }


  //intake motor code
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        intake = 127;
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
       intake = -127;
    }
    else {
     intake = 0;
    }


 }
}



