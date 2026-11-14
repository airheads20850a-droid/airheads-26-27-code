#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"

// === 底盘电机 ===
pros::MotorGroup left_motors({-15, -18}); 
pros::MotorGroup right_motors({17, 16}); 

// === 升降系统电机 (修正为 red 齿轮) ===
pros::Motor liftL(14, pros::v5::MotorGears::red); // 这个是红色的！
pros::Motor liftR(-13, pros::v5::MotorGears::red); // 红色

// === 夹取系统电机 ===
pros::Motor intake_rotation(12); 
pros::Motor intake_angle(11);    
pros::Motor ground_intake(19);   

// === 传感器 ===
pros::IMU inertial_sensor(21); 
pros::Controller master(pros::E_CONTROLLER_MASTER);

// === 底盘配置 ===
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11.4, lemlib::Omniwheel::NEW_325, 450, 2);
lemlib::ControllerSettings lateral_controller(10, 0, 15, 0, 1, 100, 0, 0, 0);
lemlib::ControllerSettings angular_controller(1, 0, 5, 0, 1, 100, 0, 0, 0);
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &inertial_sensor);
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// ==========================================
// === 任务函数区 ===
// ==========================================

void drive(void* param) {
    while (true) {
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX*0.8);
        pros::delay(20);
    }
}

void mechanism_control(void* param) {
    while (true) {
        // 1. Lift 升降 (L1 上, L2 下)
        // 使用 move_velocity 保持绝对速度，100 代表红电机的最大 100 RPM
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            liftL.move_velocity(100);
            liftR.move_velocity(100);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            liftL.move_velocity(-100);
            liftR.move_velocity(-100);
        } else {
            liftL.brake();
            liftR.brake();
        }

        // 2. 夹取角度 (R1 上, R2 下)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_angle.move(-127); 
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake_angle.move(127); 
        } else {
            intake_angle.brake(); 
        }

        // 3. 吃入/吐出 (X 吃入, B 吐出)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            intake_rotation.move(127); 
            ground_intake.move(127); 
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            intake_rotation.move(-127); 
            ground_intake.move(-127); 
        } else {
            intake_rotation.brake(); 
            ground_intake.brake();
        }

        pros::delay(20);
    }
}

// ==========================================
// === PROS 生命周期函数 ===
// ==========================================

void initialize() {
    left_motors.set_gearing({pros::MotorGears::blue, pros::MotorGears::blue});
    right_motors.set_gearing({pros::MotorGears::blue, pros::MotorGears::blue});

    // Lift 初始化
    liftL.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    liftR.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    liftL.tare_position();
    liftR.tare_position();
    liftL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    liftR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Intake 初始化
    intake_angle.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    intake_angle.tare_position(); 
    
    intake_rotation.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    ground_intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); 

    master.clear();
    pros::lcd::initialize();
    
    chassis.calibrate();

    new pros::Task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "LiftL: %f", liftL.get_position());
            pros::lcd::print(4, "IntkAng: %f", intake_angle.get_position());
            pros::delay(50);
        }
    });
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    // 自动阶段代码
}

void opcontrol() {
    pros::Task drive_thread(drive);
    pros::Task mech_thread(mechanism_control);
    
    while (true) {
        pros::delay(20);
    }
}