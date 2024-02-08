
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
pros::Motor FL(12,false);
pros::Motor MidL(7,false);
pros::Motor BL(2,false);
pros::Motor FR(18,true);
pros::Motor MidR(20,true);
pros::Motor BR(9,true);
pros::Motor intake(15,false);
pros::Motor flywheel(14,false); 
pros::Imu Gyro(21); // port for inertial
pros::ADIDigitalOut lift('h', LOW);
pros::ADIDigitalOut wings('g', LOW);
   
//Drivetrain motor groups
pros::Motor_Group leftDb({FL,MidL, BL});
  pros::Motor_Group rightDb({FR,MidR,BR});




lemlib::Drivetrain drivetrain {
    &leftDb, // left drivetrain motors
    &rightDb, // right drivetrain motors
    10, // track width
    3.25, // wheel diameter
    400, // wheel rpm
    2
    };
 


// Odom construct
    lemlib::OdomSensors sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &Gyro // inertial sensor
    };


// forward/backward PID
    lemlib::ControllerSettings lateralController {
      //kp 9.71 kd 17 - good values
    12, // kPs
    0, //kI
    90, // kD
    0, //windupRange
    0.5, // smallErrorRange
    250, // smallErrorTimeout
    1, // largeErrorRange
    500, // largeErrorTimeout
    25 // slew rate
    };
 
// turning PID wow
    lemlib::ControllerSettings angularController {
      // kp: kd:
    -9, // kP
    0, //kI
    -75, // kD 
    0, //windupRange
    1, // smallErrorRange
    250, // smallErrorTimeout
  2, // largeErrorRange
  500, // largeErrorTimeout
    25// slew rate
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

void turn(double theta){
    double x = 10000 * (sin(theta * (M_PI / 180.0) + chassis.getPose().x));
    double y = 10000 * (cos(theta * (M_PI / 180.0) + chassis.getPose().y));
    chassis.turnTo(x,y,1000);


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

void autonomous() {
    
  
  /* 
  * Close side AWP (3 balls in offensive zone, descore from load zone and touch ball)
  */
  /*
  // Step 1 - Move backward to get in position for wings
   chassis.moveToPoint(0,12.63,10000,true);
   pros::delay(600);

  // Step 2 - Deploy wings 
   wings.set_value(HIGH);
   pros::delay(600);

  // Step 3 - Move forward and dislodge triball
   chassis.moveToPoint(0,0,10000,false);
   pros::delay(600);
  
  // Step 4 - Retract wings and drive forward a bit to group triballs
   wings.set_value(LOW);
   pros::delay(600);
   chassis.moveToPoint(2.88,-12.2,2000,false);
  
  // Step 5 - Move to push triballs to offensive zone and touch bar
   chassis.moveToPoint(16.6,-26.8,10000,false);
   pros::delay(600);
  */
  
  
  

  
  
  
  
  


  
  /*
  * Autonomous Far Side Elims
  */
    //Step 1 - Intakes turns on and robot moves forward to intake triball on the short barrier line
  intake.move(127);
  chassis.moveToPoint(0, 10, 500);


  //Step 2 - Robot moves back near the matchloading bar and it turns to face the triball on the matchloading bar
  chassis.moveToPoint(0,-27,750,false);
  chassis.moveToPoint(8.69, -36, 500, false);


  //Step 3 - Wings expand and knock ball out of load zone
  wings.set_value(HIGH);
  chassis.moveToPoint(8.69, -36, 1000, false);
  chassis.turnTo(20, -40, 250, false, 127, true);
  wings.set_value(LOW);
  chassis.turnTo(10, -60, 1000, true,127,false);

  //Step 4 - Move towards goal and score parallel, then move back
  chassis.moveToPoint(12, -80, 250, true);
  chassis.moveToPoint(50, -59, 1000, true);
  intake.move(-127);
  chassis.moveToPoint(-7, -47.7, 1000, false);
  intake.move(0);


  //PART 2
  chassis.turnTo(31.64, -1.05, 1000, true);
  intake.move(127);
  chassis.moveToPoint(32, 0, 2000, true);


  chassis.turnTo(34, -60, 1000, true, 127, false);
  intake.move(0);
  chassis.moveToPoint(34, -60, 300, true);
  intake = -127;
  pros::delay(100);


  chassis.moveToPoint(15.2, -19.7, 100, false, 127, false);
  intake.move(127);
  chassis.turnTo(50.5, -8, 1000, true);
  chassis.moveToPoint(50.5, -8, 1000, true);

  chassis.turnTo(54.5, -24.11, 200, true);
  chassis.turnTo(29.87,-35,1000,true, 127, true);
  intake.move(0);
  chassis.moveToPoint(28,-40,1000,true, 127, false);
  intake.move(-127);
  pros::delay(100);
  chassis.moveToPoint(29.87,-20,1000,false);
  chassis.turnTo(54.5, -24.11, 300, true);
  

  
  

  

  /*
  * Close side mid rush
  */
  /*
  chassis.moveToPoint(-7.96, 30.4, 1000, true);
  intake.move(127);
  chassis.moveToPoint(-22.5,59.5,1000,true);
  */

  
  /* Skills Auton
  */
  /*
  chassis.moveToPoint(-0.6, -32.5, 2000, false);
  chassis.moveToPoint(0, -12, 1000);
  pros::delay(500);
  chassis.turnTo(-16.5, 2, 1000);
  pros::delay(500);
  chassis.moveToPoint(-4, -3, 1000, false);
  wings.set_value(HIGH);
  flywheel.move(105);
  pros::delay(40000);
  flywheel.move(0);
  wings.set_value(LOW);
  chassis.turnTo(-13,16.5,1000,true);
  chassis.moveToPoint(-13,16.5,1000,true);
  chassis.turnTo(-86.74,52.58,1000,true);
  chassis.moveToPoint(-86.74,52.58,1000,true);
  chassis.turnTo(-117, 51.5, 1000, true);
  chassis.moveToPoint(-117, 51.5, 1000, true);
  chassis.turnTo(-136.7, 33.22, 1000, true);
  intake = -127;
  chassis.moveToPoint(-136.7, 33.22, 2000, true);
  chassis.moveToPoint(-117, 51.5, 700, false);
  chassis.moveToPoint(-140, 36, 2000, true);
  chassis.moveToPoint(-117, 51.5, 500, false);
  //chassis.moveToPoint(-135, 30, 1000, true);
  //intake.move(-127);
  */
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
  leftDb.move(power - turn);
  rightDb.move(power + turn);
//makes motors set to brake - no extra drift
  leftDb.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
  rightDb.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);


//flywheel
   


//Lift
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
        lift.set_value(HIGH);
        flywheel = 128;
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
        flywheel = 105;
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
        lift.set_value(HIGH);
        wings.set_value(HIGH);
        flywheel = 128;
    }

 
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        lift.set_value(LOW);
        wings.set_value(LOW);
        flywheel = -0;
    }
   
   
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        lift.set_value(LOW);
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



