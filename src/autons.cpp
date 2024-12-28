#include "vex.h"
#include <iostream>
#include <sstream>

// probably dont need this now
void default_constants() {
    DRIVE_PID_DEFAULT.set_max_output(9).set_kp(1.8).set_ki(3.5).set_kd(0.1).set_integral_range(2);
    HEADING_PID_DEFAULT.set_max_output(8).set_kp(0.2).set_ki(0).set_kd(0.02).set_integral_range(0);
    TURN_PID_DEFAULT.set_max_output(12).set_kp(0.31).set_ki(5).set_kd(0.01).set_integral_range(6);
    SWING_PID_DEFAULT.set_kp(0.4).set_ki(3).set_kd(0.002).set_integral_range(15);

    DRIVE_SETTLE_DEFAULT.set_settle_error(0.25).set_settle_time(100).set_timeout(3000).set_max_current(2.5);
    MOVE_ODOM_SETTLE_DEFAULT.set_settle_error(0.5).set_settle_time(100).set_timeout(3000).set_max_current(2.5);
    TURN_SETTLE_DEFAULT.set_settle_error(2).set_settle_time(100).set_timeout(1000).set_max_current(2.5);
    SWING_SETTLE_DEFAULT.set_settle_error(2).set_settle_time(100).set_timeout(1000).set_max_current(2.5);
    RAMSETE_SETTLE_DEFAULT.set_settle_error(0.5).set_settle_time(100).set_timeout(500).set_max_current(2.5);

    FOLLOW_PP_DEFAULT = FollowConfig().set_drive_min_voltage(3).set_max_full_speed_turn_radius(24).set_max_ld(24).set_acceleration_distance(6).set_point_spacing(0.5).set_max_start_deviation(12);
    FOLLOW_RAMSETE_DEFAULT = RAMSETEConfig().set_b(2.0f).set_zeta(0.7f).set_deceleration_distance(6).set_min_target_velocity(3);
    DRIVE_TO_POSE_BOOMERANG_DEFAULT = DriveToPoseConfig().set_lead_distance(0.55);
    
    DRIVE_VELOCITY_DEFAULT = VelocityControllerConfig().set_kv(1).set_ka(0).set_kf(0).set_kp(0).set_ki(0).set_kd(0).set_integral_range(0);
    
    // IGNORE THIS
    FollowConfig::default_initialized = true;
    RAMSETEConfig::default_initialized = true;
    DriveToPoseConfig::default_initialized = true;

    chassis.set_follow_constants(10, 0.24, 12, 6, 24, 18);
    chassis.set_follow_exit_conditions(1, 150, 6000);
}

/*
 * Make auton routines here.
 *    void auton_name() {
 *        // insert code here
 *    }    
 */


void tune_velocity_controller_part_1() {
    std::vector<float> inputs = {0.f, 2.f, 4.f, 6.f, 8.f, 10.f, 12.f};
    std::vector<float> outputs = {0.f};
    outputs.reserve(inputs.size());

    float direction = 1;
    for (auto & input: inputs) {
        if (input == 0)
            continue;
        
        chassis.drive_with_voltage(direction * input, direction * input);
        task::sleep(1000);
        outputs.emplace_back(direction * chassis.get_velocity_volts());
        direction = -direction;
    }

    chassis.stop(brake);

    for (int i = 0; i < inputs.size(); ++i) {
        std::cout << Vector2(inputs[i], outputs[i]).latex() << ",";
    }
    std::cout << "\b" << std::endl;
}

void collect_step_data(float left_step_input, float right_step_input, const VelocityControllerConfig &config, char tune_mode, unsigned int duration) {
    std::vector<float> outputs = {};
    duration /= 10;
    outputs.reserve(duration);

    VelocityController left_controller(0, config);
    VelocityController right_controller(0, config);

    auto speed = chassis.get_velocity_volts();

    for (int i = 0; i < duration; ++i) {
        std::cout << Vector2(i*0.01f, speed).latex() << ",";
        if (i % 30 == 0) 
            std::cout.flush();
        auto left_voltage = left_controller.compute(left_step_input, left_step_input - speed);
        auto right_voltage = right_controller.compute(right_step_input, right_step_input - speed);
        chassis.drive_with_voltage(left_voltage, right_voltage);
        task::sleep(10);
    }

    chassis.stop(brake);
    std::cout << "\b" << std::endl;
}

void drive_test() {
    // pass
    //chassis.set_coordinates(0, 0, 0);

    chassis.turn_to_point({48, 48});
    std::cout << '(' << chassis.get_X_position() << ", " << chassis.get_Y_position() << ", " << chassis.get_absolute_heading() << ')' << std::endl;
    chassis.drive_to_point({48, 48});
    chassis.drive_to_point({0, 0});
}

void pure_pursuit_test() {
    


// Align alliance stake
    /*chassis.follow({
        Vector2(-60, 0),
        Vector2(-62.242, -40.447),
    });
// Alliance stake
    chassis.follow({
        Vector2(-62, 0),
    });
// Ring 1
    chassis.follow({
        Vector2(-37.134, 0),
    });
// Get Mogo
    chassis.follow({
        Vector2(-23.585, -23.789),
    });
// Ring 2
    chassis.follow({
        Vector2(-23.585, -50.798),
        Vector2(-31.937, -62),
        Vector2(-54.198, -62.863),
    });
// Sweep corner
    chassis.follow({
    });
// Ring 3
    chassis.follow({
        Vector2(-59.449, -62.563),
    });
// Back up
    chassis.follow({
        Vector2(-30.333, -61.859),
    });
// Turn around
    chassis.follow({
    });*/



    //chassis.set_coordinates(0,0,90);    
    //chassis.drive_to_point(36, 36);
    //chassis.drive_to_point(0, 0);
    //chassis.drive_distance(-48, chassis.get_absolute_heading());
    //chassis.drive_distance(48, chassis.get_absolute_heading());
    //chassis.turn_to_angle(-90);
    //chassis.turn_to_angle(-90);
    //chassis.turn_to_angle(-45);

    /*
    chassis.follow({
        Vector2(0, 0),
        Vector2(17.486, 27.331),
        Vector2(17.241, 41.78),
    }, 
    FollowConfig().set_direction(FORWARD));*/
}

void skills() {
    chassis.set_coordinates(-60, 0, 0);
    intake.intake();
    task::sleep(750);
        // floor len 10.5, bot 14.5
    // Grab a ring
    chassis.drive_distance(10);
    chassis.turn_to_point({-48, 24}, 180);
    run_after([]() -> bool {return chassis.get_Y_position() > 24;}, []() {mogo_clamp.close();});
    chassis.drive_distance(-24, -90);
    mogo_clamp.close();

    chassis.turn_to_angle(90);
    chassis.drive_distance(36);
    chassis.turn_to_point({-64, 64}, 180);
    chassis.drive_distance(-24);
    mogo_clamp.open();

    chassis.drive_to_point({-48, 58});
    chassis.drive_to_point({-48, 24});
}

void red_negative() {
    intake.set_filter_mode(BLUE_RINGS);
    chassis.set_coordinates(-53, 26, 180);
    // Get Mogo
    mogo_clamp.open();
    run_after([]() {return chassis.get_X_position() > -30;}, []() {mogo_clamp.close();});
    chassis.drive_distance(-26, 180, DriveDistanceConfig().set_timeout(2000).set_drive_max_voltage(8).set_heading_max_voltage(6));
    mogo_clamp.close();
    // Rush rings
    chassis.turn_to_point({-9, 40}, 0, TurnConfig().set_timeout(750));
    intake.intake();
    chassis.drive_to_point({-10, 40});
    chassis.drive_distance(-8, chassis.get_absolute_heading());
    chassis.turn_to_point({-8, 50});
    chassis.drive_distance(12, chassis.get_absolute_heading());
    chassis.drive_distance(-32);
    chassis.turn_to_point({-24, 48});
    chassis.drive_distance(18, chassis.get_absolute_heading());
    chassis.turn_to_point({-36, 36}, 180);
    chassis.drive_distance(-17, chassis.get_absolute_heading());
    chassis.turn_to_point({-66, 66});
    DoinkerPiston.set(true);
    chassis.drive_distance(28, chassis.get_absolute_heading(), DriveDistanceConfig().set_timeout(1500));
    chassis.turn_to_angle(30);
    DoinkerPiston.set(false);
    chassis.turn_to_angle(135);
    intake.intake();
    chassis.drive_distance(8);
    chassis.drive_distance(-6);
    chassis.turn_to_point({-60, 0});
    intake.raise();
    chassis.drive_distance(60, chassis.get_absolute_heading());
}

void blue_negative() {
    intake.set_filter_mode(RED_RINGS);
    chassis.set_coordinates(53, 26, 0);
    // Get Mogo
    mogo_clamp.open();
    run_after([]() {return chassis.get_X_position() < 30;}, []() {mogo_clamp.close();});
    chassis.drive_distance(-26, 0, DriveDistanceConfig().set_timeout(2000).set_drive_max_voltage(8).set_heading_max_voltage(6));
    mogo_clamp.close();
    // Rush rings
    chassis.turn_to_point({9, 40}, 0, TurnConfig().set_timeout(750));
    intake.intake();
    chassis.drive_to_point({10, 40});//, true, false, 1500);
    chassis.drive_distance(-8, chassis.get_absolute_heading());
    chassis.turn_to_point({8, 50});
    chassis.drive_distance(14, chassis.get_absolute_heading());
    chassis.drive_distance(-33);
    chassis.turn_to_point({24, 48});
    chassis.drive_distance(20, chassis.get_absolute_heading());
    chassis.turn_to_point({36, 36}, 180);
    chassis.drive_distance(-17, chassis.get_absolute_heading());
    chassis.turn_to_point({66, 65});
    DoinkerPiston.set(true);
    chassis.drive_distance(28, chassis.get_absolute_heading(), DriveDistanceConfig().set_timeout(1500));
    chassis.turn_to_angle(-60);
    DoinkerPiston.set(false);
    chassis.turn_to_angle(45);
    intake.intake();
    chassis.drive_distance(8);
    chassis.drive_distance(-6);
    chassis.turn_to_point({56, 0});
    intake.raise();
    chassis.drive_distance(60, chassis.get_absolute_heading());
}

void blue_positive() {
    chassis.set_coordinates(53, -58, 180);
    // Rush
    run_after([]() -> bool {return (chassis.get_X_position() < 18);}, []() -> void {DoinkerPiston.set(true);});
    run_after([]() -> bool {return (chassis.get_X_position() < 26);}, []() -> void {intake.pickup();});
    // Rush
    chassis.follow({
        chassis.get_position(),
        Vector2(45, -57.5),
        Vector2(29.5, -49),
        Vector2(16, -44),
    });


    // Bring it back
    chassis.drive_distance(-42, 180);
    DoinkerPiston.set(false);

    // get close mogo
    mogo_clamp.open();
    run_after([]() -> bool {return (chassis.get_Y_position() > -26);}, []() -> void {mogo_clamp.close();});
    chassis.turn_to_angle(315);
    chassis.drive_to_point({23,-23});
    mogo_clamp.close();

    // get first ring
    intake.intake();

    // clear corner
    chassis.turn_to_angle(315);

    chassis.follow({
        chassis.get_position(),
        Vector2(62.5, -60),
    });
    chassis.turn_to_angle(-30);
    
    chassis.drive_with_voltage(12, 1);
    task::sleep(750);
    chassis.drive_distance(-6, 290, DriveDistanceConfig().set_timeout(1000));
    DoinkerPiston.set(true);
    chassis.turn_to_angle(-90);
}

void red_positive() {
    chassis.set_coordinates(-53, -58, 0);
    // Rush
    run_after([]() -> bool {return (chassis.get_X_position() > -18);}, []() -> void {DoinkerPiston.set(true);});
    // Rush
    chassis.follow({
        Vector2(-53, -58),
        Vector2(-20.406, -58),
        Vector2(-14.5, -56.5),
    });
    
    // Bring it back
    chassis.drive_distance(-29, 0);
    DoinkerPiston.set(false);

// Get Mogo
    mogo_clamp.open();
    chassis.turn_to_angle(270);
    run_after([]() -> bool {return (chassis.get_Y_position() > -26);}, []() -> void {mogo_clamp.close();});
    
    chassis.follow({
        Vector2(-35, -57),
        Vector2(-35, -39),
        Vector2(-23, -23),
    });
    mogo_clamp.close();

    // get first ring
    intake.intake();
    chassis.drive_to_point({-24, -42});//, true);

    // clear corner 
    chassis.drive_distance(-12, 270, DriveDistanceConfig().set_timeout(1500));

    chassis.follow({
        chassis.get_position(),
        Vector2(-37.014, -52.364),
        Vector2(-60, -62.5),
    });
    chassis.turn_to_point({-66, -62});
    
    chassis.drive_with_voltage(1, 12);
    task::sleep(750);
    chassis.drive_distance(-6, 230, DriveDistanceConfig().set_timeout(1000));
    DoinkerPiston.set(true);
    chassis.turn_to_angle(90);
}

void find_tracking_center(float turnVoltage, uint32_t time) {
    chassis.set_coordinates(0, 0, 0);
    unsigned long n = 0;
    float heading;

    std::cout << std::fixed << "\033[1mCopy this:\033[0m\n\\left[";
    chassis.drive_with_voltage(-turnVoltage, turnVoltage);

    std::ostringstream out;

    auto end_time = time + vex::timer::system();

    int i = 0;
    
    while (vex::timer::system() < end_time && i++ < 10000) {
        std::cout << "\\left(" << chassis.get_X_position() << "," << chassis.get_Y_position() << "\\right),";
        /*if (i % 250 == 0) {
            std::cout << "\\right]\n\\left[" ;
        } */
        if (i % 50 == 0) {
            std::cout.flush();
        }
        task::sleep(20);
    }    
    chassis.stop(brake);
    std::cout << "\b\\right]" << std::endl;

    std::cout << "Go to https://www.desmos.com/calculator/xxknoawdbt to solve for offsets." << std::endl;
}

void turn_test(){
    chassis.turn_to_angle(5);
    chassis.turn_to_angle(30);
    chassis.turn_to_angle(90);
    chassis.turn_to_angle(225);
    chassis.turn_to_angle(0);
}

void swing_test(){
    chassis.left_swing_to_angle(90);
    chassis.right_swing_to_angle(0);
}

void full_test(){
    chassis.drive_distance(24);
    chassis.turn_to_angle(-45);
    chassis.drive_distance(-36);
    chassis.right_swing_to_angle(-90);
    chassis.drive_distance(24);
    chassis.turn_to_angle(0);
}

void odom_test(){
    //chassis.set_coordinates(0, 0, 0);
    while(1){
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(0,50, "X: %f", chassis.get_X_position());
        Brain.Screen.printAt(0,70, "Y: %f", chassis.get_Y_position());
        Brain.Screen.printAt(0,90, "Heading: %f", chassis.get_absolute_heading());
        Brain.Screen.printAt(0,110, "Left: %f", chassis.get_left_position_in());
        Brain.Screen.printAt(0,130, "Right: %f", chassis.get_right_position_in());
        std::cout << '(' << chassis.get_X_position() << ", " << chassis.get_Y_position() << ", " << chassis.get_absolute_heading() << ')' << std::endl;
        task::sleep(20);
    }
}
/*
void negative_red() {
    chassis.set_coordinates(-53, 16.5, 180);
    LadyBrownMotor.setStopping(hold);
    LadyBrownMotor.setVelocity(100, percent);
    LadyBrownMotor.spinToPosition(LADY_BROWN_LOWER, deg, false);

    // Get Mogo
    MogoPiston.set(open);
    run_after([]() -> bool {return (chassis.get_position() - Vector2(-24,24)).norm() < 5;}, []() {MogoPiston.set(closed);});
    chassis.follow({
        Vector2(-36.765, 16.948),
        Vector2(-23.219, 23.721),
    });
    // Rings
    IntakeMotor.spin(fwd, 12, volt);
    chassis.turn_to_point(-7.164, 34.257);
    /*chassis.follow({
        Vector2(-7, 41.03),
        Vector2(-7, 56.081),
        Vector2(-13.436, 66.366),
    }, false, 4000, true, 8, 4);
    // Another 2 Rings + Climb Touch
    chassis.turn_to_point(-47.051, -1);

    run_after([]() -> bool {return chassis.get_Y_position() < 32;}, []() {IntakePiston.set(up);});
    run_after([]() -> bool {return chassis.get_Y_position() < 4;}, []() {IntakePiston.set(down);});
    chassis.follow({
        Vector2(-23.47, 47.051),
        Vector2(-47.051, -0.612),
        Vector2(-47.803, -11.148),
        //Vector2(-16.948, -11.65),
    }, false, 6000, true, 6, 3);
    MogoPiston.set(open);
}

void negative_blue() {
    chassis.set_coordinates(53, 16.5, 0);
    LadyBrownMotor.setStopping(hold);
    LadyBrownMotor.setVelocity(100, percent);
    LadyBrownMotor.spinToPosition(LADY_BROWN_LOWER, deg, false);

    // Get Mogo
    MogoPiston.set(open);
    run_after([]() -> bool {return (chassis.get_position() - Vector2(24,24)).norm() < 5;}, []() {MogoPiston.set(closed);});
    chassis.follow({
        Vector2(36.765, 16.948),
        Vector2(23.219, 23.721),
    });
    // Rings
    IntakeMotor.spin(fwd, 12, volt);
    chassis.turn_to_point(7.164, 34.257);
    chassis.follow({
        Vector2(7, 41.03),
        Vector2(7, 56.081),
        Vector2(13.436, 66.366),
    }, false, 4000, true, 8, 4);
    // Another 2 Rings + Climb Touch
    chassis.turn_to_point(47.051, -1);

    run_after([]() -> bool {return chassis.get_Y_position() < 32;}, []() {IntakePiston.set(up);});
    run_after([]() -> bool {return chassis.get_Y_position() < 4;}, []() {IntakePiston.set(down);});
    chassis.follow({
        Vector2(23.47, 47.051),
        Vector2(47.051, -0.612),
        Vector2(47.803, -11.148),
        //Vector2(16, -11.65),
    }, false, 6000, true, 8, 4);
    MogoPiston.set(open);
}

void positive_red() {
    chassis.set_coordinates(-54,-31,180);
    LadyBrownMotor.setStopping(hold);
    LadyBrownMotor.setVelocity(100, percent);
    LadyBrownMotor.spinToPosition(LADY_BROWN_LOWER, deg, false);

    // Get Mogo
    MogoPiston.set(open);
    run_after([]() -> bool {return (chassis.get_position() - Vector2(-24,-24)).norm() < 5;}, []() {MogoPiston.set(closed);});
    chassis.follow({
        Vector2(-34.006, -30.966),
        Vector2(-24.975, -23.691),
    });

    IntakeMotor.spin(forward, 12, volt);
    IntakePiston.set(up);

    // Ring 2
    chassis.drive_to_point(-45, -2.3, true, false, 3000, 8, 4);
    IntakePiston.set(false);
    chassis.drive_time(500, 6);
    wait(1000, msec);

    run_after([]() -> bool {return chassis.get_Y_position() < -12;}, []() {MogoPiston.set(open);/*IntakeMotor.spin(fwd, -12, volt);});
    run_after([]() -> bool {return (chassis.get_position() - Vector2(0,-48)).norm() < 12;}, []() {MogoPiston.set(closed);});

    // Mogo 2
    chassis.follow({
        Vector2(-47.181, -53.726),
        Vector2(-37.134, -58.423),
        Vector2(-25.301, -58.423),
        Vector2(-6, -55),
    }, false, 6000, true, 8, 4);

    // Ring 3
    chassis.drive_to_point(-29.097, -46);

    // touch the tower
    chassis.drive_to_point(-12.353, -11);
    MogoPiston.set(open);
}

void positive_blue() {
    chassis.set_coordinates(54,-31,0);
    LadyBrownMotor.setStopping(hold);
    LadyBrownMotor.setVelocity(100, percent);
    LadyBrownMotor.spinToPosition(LADY_BROWN_LOWER, deg, false);

    // Get Mogo
    MogoPiston.set(open);
    run_after([]() -> bool {return (chassis.get_position() - Vector2(24,-24)).norm() < 5;}, []() {MogoPiston.set(closed);});
    chassis.follow({
        Vector2(34.006, -30.966),
        Vector2(24.975, -23.691),
    });

    IntakeMotor.spin(forward, 12, volt);
    IntakePiston.set(up);

    // Ring 2
    chassis.drive_to_point(45, -2.3, true, false, 3000, 8, 4);
    IntakePiston.set(false);
    chassis.drive_time(500, 6);
    wait(1000, msec);

    run_after([]() -> bool {return chassis.get_Y_position() < -12;}, []() {MogoPiston.set(open);/*IntakeMotor.spin(fwd, -12, volt);});
    run_after([]() -> bool {return (chassis.get_position() - Vector2(0,-48)).norm() < 12;}, []() {MogoPiston.set(closed);});

    // Mogo 2
    chassis.follow({
        Vector2(47.181, -53.726),
        Vector2(37.134, -58.423),
        Vector2(25.301, -58.423),
        Vector2(6, -55),
    }, false, 6000, true, 8, 4);

    // Ring 3
    chassis.drive_to_point(29.097, -46);

    // touch the tower
    chassis.drive_to_point(12.353, -11);
    MogoPiston.set(open);
}*/

void tank_odom_test(){
    chassis.set_coordinates(0, 0, 0);
    chassis.drive_to_point({6, 18});
    //chassis.turn_to_point(12,0, 180);
    chassis.drive_to_point({12, 0});
    chassis.turn_to_angle(100);
    chassis.drive_to_point({0, 0});
}

void holonomic_odom_test(){
    chassis.set_coordinates(0, 0, 0);
    chassis.holonomic_drive_to_point(0, 18, 90);
    chassis.holonomic_drive_to_point(18, 0, 180);
    chassis.holonomic_drive_to_point(0, 18, 270);
    chassis.holonomic_drive_to_point(0, 0, 0);
}





Auton::Auton(std::string name, std::function<void(void)> auton_function, vex::color display_color): 
    name(name), auton_function(auton_function), display_color(display_color) {}

Auton::Auton(std::string name, std::function<void(void)> auton_function): 
    name(name), auton_function(auton_function), display_color(vex::red) {}
