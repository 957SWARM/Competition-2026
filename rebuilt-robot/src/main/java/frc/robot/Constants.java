package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {

    public class PivotConstants{
        
        public static final double MAX_ACCELERATION = 1;
        public static final double MAX_SPEED = 1;

        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KG = 0;
        public static final double KF = 0;

        public static final double CRUISE_VELOCITY = 1;
        public static final double CRUISE_ACCELERATION = 1;
        public static final double PIVOT_JERK = 0.1;

        public static final int ENCODER_ID = 0;
        public static final int MOTOR_ID = 1;

        public static final CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        public static final TalonFXConfiguration config = new TalonFXConfiguration();
        public static final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        public static final double STOW_ANGLE = 0;
        public static final double DEPLOY_ANGLE = 0.95;

        static {
            config.Slot0.kS = Constants.PivotConstants.KS;
            config.Slot0.kV = Constants.PivotConstants.KV;
            config.Slot0.kA = Constants.PivotConstants.KA;
            config.Slot0.kP = Constants.PivotConstants.KP;
            config.Slot0.kI = Constants.PivotConstants.KI;
            config.Slot0.kD = Constants.PivotConstants.KD;

            MotionMagicConfigs mmConfig = config.MotionMagic;
            mmConfig.MotionMagicCruiseVelocity = 1;
            mmConfig.MotionMagicAcceleration = 1;
            mmConfig.MotionMagicJerk = 0.1;

            config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

            limitConfigs.SupplyCurrentLimit = 30;
        }

    }

    public class IntakeConstants{
        
        public static final int INTAKE_ID = 2;

        public static final double INTAKE_VOLTAGE = 3.0;
        public static final double EJECT_VOLTAGE = -3.0;
    }

    public class ShooterConstants{

        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;

        public static final int SHOOTER_LEAD_ID = 3;
        public static final int SHOOTER_FOLLOW_ID = 4;

        public static final double SHOOTER_PASSIVE_VOLTAGE = 0;
        public static final double FROM_NEUTRAL_VOLTAGE = 5;

        public static final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration shooterFollowConfig = new TalonFXConfiguration();
        public static final CurrentLimitsConfigs shooterLimitConfigs = new CurrentLimitsConfigs();

        static {
            shooterConfig.Slot0.kS = KS;
            shooterConfig.Slot0.kV = KV;
            shooterConfig.Slot0.kA = KA;
            shooterConfig.Slot0.kP = KP;
            shooterConfig.Slot0.kI = KI;
            shooterConfig.Slot0.kD = KD;

            shooterFollowConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            shooterLimitConfigs.SupplyCurrentLimit = 40;
            

        }
        
    }

    public class HoodConstants{

        public static final int HOOD_ID = 5;
        public static final int ENCODER_ID = 6;

        public static final double ZEROING_SPEED = -0.1;
        public static final double ZERO_SPEED = 0.08;

        public static final double SHOOTING_ANGLE = 60;
        public static final double FROM_NEUTRAL_ANGLE = 30;

        public static final double ZEROING_TIME = 0.1;

        public static final double VELOCITY_TO_VOLTS = 1.0;

        
    }

    public class ConveyerConstants{

        public static final double FEED_VOLTAGE = 5;
        public static final double KICK_VOLTAGE = 8;

        public static final int CONVEYER_ID = 7;
        public static final int KICKER_ID = 8;

        public static final TalonFXConfiguration conveyerConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

        static{
            
            conveyerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            kickerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        }
    }

    public class ControllerConstants{

        public static final double TRIGGER_THRESHOLD = 0.5;
    }

    public class FieldConstants{

        public static final Pose2d RED_HUB_LOCATION = new Pose2d(11.912, 4.024, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_HUB_LOCATION = new Pose2d(4.628, 4.024, Rotation2d.fromDegrees(0));
    }

    public class TargetingConstants{

        public static final double MIN = 0;
        public static final double MAX = 1;
        public static final double DEADBAND = 0.1;
        public static final double KP = 1;
    }
}
