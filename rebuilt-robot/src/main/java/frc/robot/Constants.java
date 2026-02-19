package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {

    public class PivotConstants{
        
        public static final double MAX_ACCELERATION = 1;
        public static final double MAX_SPEED = 1;

        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KP = 40;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KG = 0;
        public static final double KF = 0;

        public static final double CRUISE_VELOCITY = 128;
        public static final double CRUISE_ACCELERATION = 128;

        public static final int ENCODER_ID = 15;
        public static final int MOTOR_ID = 19;

        public static final CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        public static final TalonFXConfiguration config = new TalonFXConfiguration();
        public static final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        public static final double STOW_ANGLE = 0.2;
        public static final double DEPLOY_ANGLE = 0;

        static {
            config.Slot0.kS = Constants.PivotConstants.KS;
            config.Slot0.kV = Constants.PivotConstants.KV;
            config.Slot0.kA = Constants.PivotConstants.KA;
            config.Slot0.kP = Constants.PivotConstants.KP;
            config.Slot0.kI = Constants.PivotConstants.KI;
            config.Slot0.kD = Constants.PivotConstants.KD;

            MotionMagicConfigs mmConfig = config.MotionMagic;
            mmConfig.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
            mmConfig.MotionMagicAcceleration = CRUISE_ACCELERATION;

            config.MotionMagic = mmConfig;

            config.Feedback.FeedbackRemoteSensorID = ENCODER_ID;
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            config.Feedback.RotorToSensorRatio = 45 * 2.5;
            config.Feedback.SensorToMechanismRatio = 1;

        }

    }

    public class IntakeConstants{
        
        public static final int INTAKE_ID = 17;

        public static final double INTAKE_VOLTAGE = 8;
        public static final double EJECT_VOLTAGE = -3.0;

        public static final TalonFXSConfiguration rollerConfig = new TalonFXSConfiguration();
        static {
            rollerConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
        }
    }

    public class ShooterConstants{

        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;

        public static final int SHOOTER_LEAD_ID = 9;
        public static final int SHOOTER_FOLLOW_ID = 22;

        public static final double SHOOTER_PASSIVE_VOLTAGE = 0;
        public static final double FROM_NEUTRAL_VOLTAGE = 5;

        public static final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration shooterFollowConfig = new TalonFXConfiguration();

        static {
            shooterConfig.Slot0.kS = KS;
            shooterConfig.Slot0.kV = KV;
            shooterConfig.Slot0.kA = KA;
            shooterConfig.Slot0.kP = KP;
            shooterConfig.Slot0.kI = KI;
            shooterConfig.Slot0.kD = KD;

            shooterFollowConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            

        }
        
    }

    public class HoodConstants{

        public static final int HOOD_ID = 16;
        public static final int ENCODER_ID = 14;

        public static final double ZEROING_SPEED = -0.35;
        public static final double ZERO_SPEED = 0.08;

        public static final double FROM_NEUTRAL_ANGLE = 30;
        public static final double IDLE_POSITION = 0.25;

        public static final double ZEROING_TIME = 0.1;
        public static final double MAX_HOOD_POS = 2.8;

        public static final double VELOCITY_TO_VOLTS = 1.0;

        public static final double HOOD_KP = 20;
        public static final double HOOD_KI = 0;
        public static final double HOOD_KD = 0;

        public static final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    }

    public class ConveyerConstants{

        public static final double FEED_VOLTAGE = 5;
        public static final double KICK_VOLTAGE = 12;

        public static final int CONVEYER_ID = 18;
        public static final int KICKER_ID = 20;

        public static final TalonFXConfiguration conveyerConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

        static {
            
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

        public static final Pose2d BLUE_TOP_PASSZONE = new Pose2d(0, 0, null);
        public static final Pose2d BLUE_BOTTOM_PASSZONE = new Pose2d(0, 0, null);
        public static final Pose2d RED_TOP_PASSZONE = new Pose2d(0, 0, null);
        public static final Pose2d RED_BOTTOM_PASSZONE = new Pose2d(0, 0, null);
    }

    public class TargetingConstants{

        public static final double MIN = 0;
        public static final double MAX = 5;
        public static final double DEADBAND = 0;
        public static final double KP = .1;
    }

    public class PowerConstants{
        
        public static final int HOOD_LIMIT = 20;

        public static final CurrentLimitsConfigs low = new CurrentLimitsConfigs();
        public static final CurrentLimitsConfigs mid_low = new CurrentLimitsConfigs();
        public static final CurrentLimitsConfigs mid_high = new CurrentLimitsConfigs();
        public static final CurrentLimitsConfigs high = new CurrentLimitsConfigs();

        static {
            
            low.SupplyCurrentLimit = 20;
            mid_low.SupplyCurrentLimit = 30;
            mid_high.SupplyCurrentLimit = 40;
            high.SupplyCurrentLimit = 50;
        }
    }
}
