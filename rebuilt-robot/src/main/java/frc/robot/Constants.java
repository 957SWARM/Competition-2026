package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;

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

        public static final double STOW_ANGLE = 0.224;
        public static final double DEPLOY_ANGLE = 0;
        public static final double AGITATE_ANGLE = 0.175;

        public static final double AGITATION_TIME = 0.5;

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

        public static final double INTAKE_VOLTAGE = 7;
        public static final double EJECT_VOLTAGE = -3.0;

        public static final double INTAKE_FROM_SPEED_SCALAR = 0.15;

        public static final TalonFXSConfiguration rollerConfig = new TalonFXSConfiguration();
        static {
            rollerConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
        }
    }

    public class ShooterConstants{

        public static final double KS = 0;
        public static final double KV = 0.12;
        public static final double KA = 0;
        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0.035;

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

        public static final double ZEROING_SPEED = -0.6;
        public static final double ZERO_SPEED = 0.08;

        public static final double FROM_NEUTRAL_ANGLE = 30;
        public static final double IDLE_POSITION = 0.25;
        public static final double MAX_IDLE_ANGLE = 0.5;

        public static final double ZEROING_TIME = 0.1;
        public static final double MAX_HOOD_POS = .37714;

        public static final double MAX_PERCENT_OUTPUT = 0.5;

        public static final double VELOCITY_TO_VOLTS = 1.0;

        public static final double HOOD_KP = 20;
        public static final double HOOD_KI = 0;
        public static final double HOOD_KD = 0.25;

        public static final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    }

    public class ConveyerConstants{

        public static final double FEED_VOLTAGE = 6;
        public static final double KICK_VELOCITY = 75;
        public static final double MAX_KICK_VELOCITY = 90;
        public static final double IDLE_FEED_VOLTAGE = 1;
        public static final double IDLE_KICK_VELOCITY = -33;

        public static final int CONVEYER_ID = 18;
        public static final int KICKER_ID = 20;

        public static final TalonFXConfiguration conveyerConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

        public static final double KS = 0;
        public static final double KV = 0.12;
        public static final double KA = 0;
        public static final double KP = 0.2;
        public static final double KI = 0;
        public static final double KD = 0.035;

        static {

            kickerConfig.Slot0.kS = KS;
            kickerConfig.Slot0.kV = KV;
            kickerConfig.Slot0.kA = KA;
            kickerConfig.Slot0.kP = KP;
            kickerConfig.Slot0.kI = KI;
            kickerConfig.Slot0.kD = KD;
            
            conveyerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            kickerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        }
    }

    public class ClimberConstants{

        public static final int CLIMBER_ID = 25;
        public static final double CLIMB_L1_SPEED = 0.05;
        public static final double HOME_CLIMBER_SPEED = -0.05;

        public static final int CLIMBER_LIMITSWITCH_CHANNEL = 0;

        public static final int TOF_ID = 23;
        public static final double TOF_THRESHOLD = 100.0;

        public static final double FIELD_DRIVE = 0.5;

        public static final double TRANSLATION_P = 0.05;
        public static final double ROTATION_P = 0.005;
        public static final double TRANSLATION_DEADBAND = 0.02;
        public static final double ROTATION_DEADBAND = 0.5;
        public static final double TRANSLATION_MIN = 0.35;
        public static final double TRANSLATION_MAX = 1;
        public static final double ROTATION_MIN = 0.05;
        public static final double ROTATION_MAX = 0.5;

        public static final TalonFXConfiguration climberConfig = new TalonFXConfiguration();

    }

    public class ControllerConstants{

        public static final double TRIGGER_THRESHOLD = 0.5;
    }

    public class FieldConstants{

        public static final Pose2d RED_HUB_LOCATION = new Pose2d(11.912, 4.024, Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_HUB_LOCATION = new Pose2d(4.628, 4.024, Rotation2d.fromDegrees(0));

        public static final Pose2d BLUE_TOP_PASSZONE = new Pose2d(1.9, 6.6, new Rotation2d());
        public static final Pose2d BLUE_BOTTOM_PASSZONE = new Pose2d(1.9, 1.5, new Rotation2d());
        public static final Pose2d RED_TOP_PASSZONE = new Pose2d(14.5, 6.6, new Rotation2d());
        public static final Pose2d RED_BOTTOM_PASSZONE = new Pose2d(14.5, 1.5, new Rotation2d());

        public static final Pose2d BLUE_CLIMB_LEFT = new Pose2d(0.748, 4.698, Rotation2d.fromDegrees(3.14159));
        public static final Pose2d BLUE_CLIMB_RIGHT = new Pose2d();
        public static final Pose2d RED_CLIMB_LEFT = new Pose2d();
        public static final Pose2d RED_CLIMB_RIGHT = new Pose2d();
        public static final Pose2d DEBUG_DRIVE_POINT = new Pose2d(2.309, 3.861, Rotation2d.fromDegrees(0));
        

        public static Pose2d TARGET_HUB = BLUE_HUB_LOCATION;

        static {
            if(DriverStation.getAlliance().get() != Alliance.Blue){
                TARGET_HUB = RED_HUB_LOCATION;
            }
            
        }
    }

    public class TargetingConstants{

        public static final double MIN = 0.25;
        public static final double MAX = 5;
        public static final double DEADBAND = 0.2; //FOR BOUNDED P LOOP
        public static final double KP = 0.15;

        public static final double kRa = 8;

        public static final double TARGETING_DEADBAND = 1.5; //DEGREES
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

    public class DriveConstants{

        public static final double MAX_SPEED = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MAX_ANGULAR = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        public static final double MAX_STRAFE_SHOOT_SPEED = 1.85;
        
        public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MAX_SPEED * 0.1).withRotationalDeadband(MAX_ANGULAR * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

        public static final SwerveRequest.FieldCentricFacingAngle drive45 = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MAX_SPEED * 0.1).withTargetDirection(null)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    }
}
