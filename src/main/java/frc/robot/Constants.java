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
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;

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
        public static final int FRONT_LEFT = 12;
        public static final int FRONT_RIGHT = 58;
        public static final int BACK_LEFT = 60;
        public static final int BACK_RIGHT = 44;

        // todo : add from characterization data @
        public static final double KS_VOLTS = 0.22; // @
        public static final double KS_VOLT_SECONDS_PER_METER = 1.98; // @
        public static final double KS_VOLT_SECONDS_SQUARED_PER_METER = 0.2; // @
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(19.5); //todo : check
        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
        public static final DifferentialDriveVoltageConstraint VOLTAGE_CONSTRAINT = 
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(KS_VOLTS, KS_VOLT_SECONDS_PER_METER, KS_VOLT_SECONDS_SQUARED_PER_METER),
                 KINEMATICS, 10); //10V max to account for battery sag
        public static final double MAX_SPEED_METERS_PER_SECOND = 3; //todo : change @
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; //todo : change @, not as important due to voltage constraint
        // RAMSETE constants
        public static final double kRamseteB = 2; // default, should be good
        public static final double kRamseteZeta = 0.7; // default, should be good
        // Encoders (on back)
        public static final boolean LEFT_BACK_SENSOR_PHASE = false;
        public static final boolean RIGHT_BACK_SENSOR_PHASE = false;
        // Wheel
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6);
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        public static final int ENCODER_TICKS_PER_ROTATION = 4096;
        // Drive velocity PID (TALONSRX)
        public static final double DRIVE_P = 0.1; // @
        public static final double DRIVE_I = 0.1; // @
        public static final double DRIVE_D = 0.1; // @
        public static final int DRIVE_VELOCITY_ERROR_TOLERANCE = (int) (.1d * ENCODER_TICKS_PER_ROTATION); // .1 rotation tolerance
        // Drive velocity sampling settings
        public static final int ROLLING_VELOCITY_SAMPLES = 4; // 1,2,4,8,16,32
        public static final VelocityMeasPeriod VELOCITY_MEAS_PERIOD = VelocityMeasPeriod.Period_5Ms;
        public static final int STATUS_2_FEEDBACK_MS = 10; // 20ms default
        public static final int STATUS_3_QUADRATURE_MS = 20; // 160ms default

        public static final int MAX_BATTERY_V = 12;
    }

    public static class Vision {
        public static class Limelight {
            public static double MOUNT_HEIGHT_M = Units.inchesToMeters(35.25); //todo: change
            public static double MOUNT_ANGLE_RAD = Math.toRadians(16); // Mount angle, from horizon //todo: change
            public static double PORT_CENTER_HEIGHT_M = Units.inchesToMeters(98.19);
        }
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
        public static final boolean LEFT_MOTOR_INVERTED = true;
        public static final int TICKS_PER_REV = 4096;
        public static class Characteristics {
            public static final double P = 0.0;
            public static final double I = 0.0;
            public static final double D = 0.0;
            public static final double MAX_BATTERY_V = 12;
            public static final double ksVolts = 0.1;
            public static final double kvVoltSecondsPerMeter = 2.2;
            public static final double RPM_THRESHOLD = 200;
        }
    }

    public static class Climber {
        public static final int MAIN_MOTOR_ID = 17;
        public static final int SECONDARY_MOTOR_ID = 18;
    }

    public static class HookDeploy {
        public static final int MOTOR_ID = 19;
    }

    public static class ColorWheel {
        public static final int MOTOR_ID = 20;
        public static final boolean MOTOR_INVERTED = false;
        public static final I2C.Port COLOR_SENSOR_PORT = I2C.Port.kOnboard;
        public static final double DEFAULT_SPEED = 0.3;
    }
}
