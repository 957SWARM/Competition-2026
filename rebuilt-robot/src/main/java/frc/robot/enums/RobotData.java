package frc.robot.enums;

import java.lang.annotation.Target;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class RobotData {
    
    public static RobotData instance;

    public static double distanceToTarget = 0;
    public static Rotation2d angleToTarget = new Rotation2d();
    public static TargetingPoint target = TargetingPoint.BLUE_HUB;
    public static double xVelocity = 0;
    public static double yVelocity = 0;
    public static double thetaVelocity = 0;
    public static Pose2d botPose = new Pose2d();

    private RobotData(){

    }

    public static RobotData getInstance(){
        if(instance == null){
            instance = new RobotData();
        }
        return instance;
    }

    public void updateTargetingInfo(TargetingPoint point, Pose2d botPose){
        target = point;
        angleToTarget = target.getAngleFromPoint(botPose);
        distanceToTarget = target.getDistanceToPoint(botPose);
    }

    public void updateDriveInfo(SwerveDriveState states){
        xVelocity = states.Speeds.vxMetersPerSecond;
        yVelocity = states.Speeds.vyMetersPerSecond;
        thetaVelocity = states.Speeds.omegaRadiansPerSecond;
        botPose = states.Pose;
    }
    

}  