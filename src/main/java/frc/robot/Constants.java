/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.N2;

import java.util.function.Function;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class DriveTrain {
        public static final int FRONT_LEFT = 4;
        public static final int FRONT_RIGHT = 3;
        public static final int BACK_LEFT = 2;
        public static final int BACK_RIGHT = 1;

        public static final int DEFAULT_SETTING_TIMEOUT_MS = 50;

        public static final double kS = 0.62; // kS
        public static final double kV = 2.42; // kV
        public static final double kA = 0.165; // kA
        public static final double kVAngular = 1.654294727;
        public static final double kAAngular = kA; // ???????
        public static final double TRACK_WIDTH_METERS = 0.5757943419; // About 22.6" from characterization, was 22" from CAD
        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
        public static final DifferentialDriveVoltageConstraint VOLTAGE_CONSTRAINT =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(kS, kV, kA),
                        KINEMATICS, 10); //10V max to account for battery sag
        public static final LinearSystem<N2, N2, N2> PLANT =
                LinearSystemId.identifyDrivetrainSystem(
                        kV,
                        kA,
                        kVAngular,
                        kAAngular);
        public static final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(13); // 13 ft/s = 3.9624
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2; // 2m/s/s = about 2 seconds to full speed
        // RAMSETE constants
        public static final double kRamseteB = 2; // default, should be good
        public static final double kRamseteZeta = 0.7; // default, should be good
        // Encoders (on back)
        //public static final boolean LEFT_BACK_SENSOR_PHASE = true;
        //public static final boolean RIGHT_BACK_SENSOR_PHASE = true;
        // Wheel
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6);
        public static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2.0d;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        public static final double DRIVE_GEARING = 10.71; // if ratio from motor to wheel is X:1, enter X, todo: ask Ed for real value
        public static final int ENCODER_TICKS_PER_ROTATION = (int) Math.round(2048 * DRIVE_GEARING); // 2048 for TalonFX, 4096 for TalonSRX
        // Drive velocity PID (TalonFX)
        public static final int VELOCITY_PID_IDX = 0;
        public static final double VELOCITY_P = 0.000953; // With 10.71 gearing in analysis (In Google Docs Notes as #A1)
        public static final double VELOCITY_I = 0; //
        public static final double VELOCITY_D = 0; //
        public static final int DRIVE_VELOCITY_ERROR_TOLERANCE = (int) (.1d * ENCODER_TICKS_PER_ROTATION); // .1 rotation tolerance
        // Drive velocity sampling settings
        //public static final int ROLLING_VELOCITY_SAMPLES = 4; // 1,2,4,8,16,32
        //public static final VelocityMeasPeriod VELOCITY_MEAS_PERIOD = VelocityMeasPeriod.Period_5Ms;
        public static final int STATUS_2_FEEDBACK_MS = 20; // 20ms default
        public static final int STATUS_3_QUADRATURE_MS = 40; // 160ms default

        public static final int MAX_BATTERY_V = 12;
        public static final boolean HAS_ENCODERS = true;
    }

    public static class Vision {
        public static class Limelight {
            public static double MOUNT_HEIGHT_M = Units.inchesToMeters(35.25); //todo: change
            public static double MOUNT_ANGLE_RAD = Math.toRadians(16); // Mount angle, from horizon //todo: change
            public static double PORT_CENTER_HEIGHT_M = Units.inchesToMeters(98.19);
        }
        public static class Pixy2Constants {
            public static final double HORIZONTAL_FOV_DEG = 60;
            public static final double MAX_X = 315;
        }
    }

    public static class RobotDimensions {
        public static final double LENGTH = 0.82042;
        public static final double WIDTH = 0.69596;
    }

    public static class GalacticSearch {
        // Area threshold for ball to be considered CLOSE (Pixy camera)
        public static final int PIXY_AREA_THRESHOLD_FOR_CLOSE_POWER_CELL = 3445; // todo: change

        /* https://www.desmos.com/calculator/sxcsdbgjfv */
        // Robot Start X
        public static final double ROBOT_START_X = Units.feetToMeters(3.82);
        // Distance to CLOSE ball
        public static final double CLOSE_BALL_DISTANCE = Units.feetToMeters(3.68);
        // Distance to FAR ball
        public static final double FAR_BALL_DISTANCE = Units.feetToMeters(11.18);
        // How close to get to the balls, FROM front of robot (hence adding half of length of obt)
        public static final double DESIRED_DISTANCE_TO_BALL = Units.inchesToMeters(12) + (RobotDimensions.LENGTH / 2d);
        // Speed to go when driving towards ball to intake
        public static final double DRIVE_TO_BALL_FOR_INTAKE_SPEED = 0.15;
        // Time to wait before moving again after ball is detected at intake sensor
        public static final double WAIT_AFTER_INTAKE_SECONDS = 0.5;

        // X of end zone, https://www.desmos.com/calculator/yijiq0dal9
        public static final double ROBOT_END_X = Units.feetToMeters(27.5);
    }

    public static class Intake {
        public static final int MOTOR_ID = 59;
        public static final boolean MOTOR_INVERTED = true;
        public static final int INTAKE_SENSOR = 1;
    }

    public static class VerticalHopper {
        // Looking at the robot from the front
        public static final int LEFT_MOTOR_ID = 16;
        public static final boolean LEFT_MOTOR_INVERTED = true;
        public static final int RIGHT_MOTOR_ID = 57;
        public static final int INTAKE_SENSOR = 1;

    }

    public static class Gate {
        public static final int SOLENOID_ID = 0;
        public static final boolean CLOSE_STATE = true;
        public static final boolean OPEN_STATE = !CLOSE_STATE;
    }

    public static class Shooter {
        public static final int LEFT_MOTOR_ID = 30;
        public static final int RIGHT_MOTOR_ID = 31;
        public static final boolean LEFT_MOTOR_INVERTED = false;
        public static final boolean RIGHT_MOTOR_INVERTED = true;
        public static final boolean LEFT_SENSOR_PHASE = false;
        public static final boolean RIGHT_SENSOR_PHASE = false;
        public static final int TICKS_PER_REV = 4096;
        public static class Characteristics {
            public static final double P = 0.95;
            public static final double I = 0.0;
            public static final double D = 0.0;
            public static final double MAX_BATTERY_V = 12;
            public static final double ksVolts = 0.281;
            public static final double kvVoltSecondsPerRotation = 0.162;
            public static final double RPM_THRESHOLD = 10;
        }
        public static final Function<Double, Double> METERS_TO_RPM_FUNCTION = meters -> {
            return meters * 247; // todo: Empirically find a formula
        };
    }

    public static class Winch {
        public static final int MAIN_MOTOR_ID = 17;
        public static final boolean MOTOR_INVERTED = true;
        //public static final int SECONDARY_MOTOR_ID = 18;
    }

    public static class HookElevator {
        public static final int MOTOR_ID = 19;
        public static final boolean MOTOR_INVERTED = true;
    }

    public static class ColorWheel {
        public static final int MOTOR_ID = 20;
        public static final boolean MOTOR_INVERTED = false;
        public static final I2C.Port COLOR_SENSOR_PORT = I2C.Port.kOnboard;
        public static final double DEFAULT_SPEED = 0.3;
    }
}
