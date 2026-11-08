#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"

// === 底盘电机 (保留你的正确端口) ===
pros::MotorGroup left_motors({-5, -16}); 
pros::MotorGroup right_motors({1, 6}); 

// === 升降系统电机 (恢复原来的 Lift) ===
pros::Motor liftL(20, pros::v5::MotorGears::green);
pros::Motor liftR(-12, pros::v5::MotorGears::green);

// === 夹取系统电机 ===
pros::Motor intake_rotation(9); // Port 9: 控制夹取转动 (吃入/吐出)
pros::Motor intake_angle(10);   // Port 10: 控制夹取装置角度 (上下翻转)

// === 底盘配置 ===
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.4, // Track Width 
                              lemlib::Omniwheel::NEW_325,
                              450, // Wheelbase
                              2 // horizontal drift
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, 0, 15, 0, 1, 100, 0, 0, 0);
// angular PID controller
lemlib::ControllerSettings angular_controller(1, 0, 5, 0, 1, 100, 0, 0, 0);

pros::IMU inertial_sensor(19);
pros::Controller master(pros::E_CONTROLLER_MASTER);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &inertial_sensor);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// ==========================================
// === 任务函数区 ===
// ==========================================

// 底盘控制任务
void drive(void* param) {
    while (true) {
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX*0.8);
        pros::delay(20);
    }
}

// 机械结构控制任务 (Lift + Intake)
void mechanism_control(void* param) {
    while (true) {
        // --- 1. 控制 Lift 升降 (L1 上升, L2 下降) ---
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            liftL.move(127);
            liftR.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            liftL.move(-127);
            liftR.move(-127);
        } else {
            liftL.brake();
            liftR.brake();
        }

        // --- 2. 控制夹取角度 (R1 上升, R2 下降) ---
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_angle.move(-127); // 向上抬升/翻转
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake_angle.move(127); // 向下降低/翻转
        } else {
            intake_angle.brake(); // 松开按键时锁死位置
        }

        // --- 3. 控制吃入/吐出 (X 吃入, B 吐出) ---
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            intake_rotation.move(127); // 吃入
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            intake_rotation.move(-127); // 吐出
        } else {
            intake_rotation.brake(); // 松开按键停止
        }

        pros::delay(20);
    }
}

// ==========================================
// === PROS 生命周期函数 ===
// ==========================================

void initialize() {
    // 底盘初始化 (设置电机齿轮比)
    left_motors.set_gearing({pros::MotorGears::blue, pros::MotorGears::blue});
    right_motors.set_gearing({pros::MotorGears::blue, pros::MotorGears::blue});

    // Lift 初始化
    liftL.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    liftR.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    liftL.tare_position();
    liftR.tare_position();
    liftL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // 防止重力掉下
    liftR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Intake 初始化
    intake_angle.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // 角度电机也锁死
    intake_rotation.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    intake_angle.tare_position(); // 重置角度编码器

    master.clear();
    pros::lcd::initialize();
    
    // 校准底盘传感器
    chassis.calibrate();

    // 屏幕显示任务 
    new pros::Task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "LiftL: %f", liftL.get_position()); // 显示升降电机位置
            pros::lcd::print(4, "IntkAng: %f", intake_angle.get_position()); // 显示夹取角度电机位置
            pros::delay(50);
        }
    });
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    // 自动阶段代码留空，等你需要写的时候再填
}

// 手动控制阶段
void opcontrol() {
    // 启动底盘控制线程
    pros::Task drive_thread(drive);

    // 启动机械结构控制线程 (Lift + Intake)
    pros::Task mech_thread(mechanism_control);
    
    // 保持 opcontrol 存活
    while (true) {
        pros::delay(20);
    }
}