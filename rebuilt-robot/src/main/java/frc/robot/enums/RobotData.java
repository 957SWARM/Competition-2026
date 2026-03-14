package frc.robot.enums;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotData {
    
    public static RobotData instance;

    public static double distanceToTarget = 0;
    public static Rotation2d angleToTarget = new Rotation2d();
    public static TargetingPoint visiontarget = TargetingPoint.BLUE_HUB;
    public static TargetingPoint climbTarget = TargetingPoint.BLUE_CLIMB_RIGHT;
    public static double xVelocity = 0;
    public static double yVelocity = 0;
    public static double thetaVelocity = 0;
    public static Pose2d botPose = new Pose2d();
    public static Pose2d lookAheadPose;
    public boolean mirrorAuto;

    private RobotData(){

    }

    public static RobotData getInstance(){
        if(instance == null){
            instance = new RobotData();
        }
        return instance;
    }

    public void updateTargetingInfo(TargetingPoint visionPoint, TargetingPoint climbPoint, Pose2d botPose){
        visiontarget = visionPoint;
        angleToTarget = visiontarget.getAngleFromPoint(botPose);
        distanceToTarget = visiontarget.getDistanceToPoint(botPose);
        climbTarget = climbPoint;
    }

    public void updateDriveInfo(SwerveDriveState states){
        xVelocity = states.Speeds.vxMetersPerSecond;
        yVelocity = states.Speeds.vyMetersPerSecond;
        thetaVelocity = states.Speeds.omegaRadiansPerSecond;
        botPose = states.Pose;

        new Rotation2d();
        lookAheadPose = new Pose2d(states.Pose.getX() - xVelocity, states.Pose.getY() - yVelocity, Rotation2d.fromDegrees(states.Pose.getRotation().getDegrees() + thetaVelocity));
    }
    

}  