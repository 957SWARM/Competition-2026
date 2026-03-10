package frc.robot.commands;


import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.SwarmDriveController;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TargetingConstants;
import frc.robot.enums.RobotData;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.TargetingHelper;

public class Sequencing {
    
    public static Command zeroHood(HoodSubsystem hood){
        return new WaitCommand(0.5).andThen(hood.zeroEncoder()
                    .withTimeout(HoodConstants.ZEROING_TIME)
                    .andThen(
                        hood.zeroEncoder()
                        .until(() -> hood.isStopped())).andThen(() -> hood.homeHood()));
    }

    public static Command shootToPoint(RollerSubsystem roller, ConveyerSubsystem conveyer, ShooterSubsystem shooter, KickerSubsystem kicker, CommandSwerveDrivetrain drive){
        return roller.intakeCommand(drive).alongWith(new WaitCommand(0).andThen(conveyer.runConveyerForwards()).alongWith(kicker.runKicker()));//.alongWith(new WaitCommand(1).andThen(kicker.runKicker()));
    }

    public static boolean isAlignedToHub() {
        if(Math.abs(RobotData.angleToTarget.getDegrees() - RobotData.botPose.getRotation().getDegrees()) < TargetingConstants.TARGETING_DEADBAND 
        && Math.abs(RobotData.xVelocity) < 0.1 
        && Math.abs(RobotData.yVelocity) < 0.1
        && Math.abs(RobotData.thetaVelocity) < 0.04){
            return true;
        }
        else{
            return false;
        }
    }

    public static boolean isDriving(SwarmDriveController xbox){
        if(Math.abs(xbox.getXLimitedInput()) > 0.1 
        && Math.abs(xbox.getYLimitedInput()) > 0.1
        ){
            return true;
        }
        else{
            return false;
        }
    }

    public static Command autoShootAndPlant(RollerSubsystem roller,
                                        ConveyerSubsystem conveyer,
                                        CommandSwerveDrivetrain drivetrain,
                                        KickerSubsystem kicker,
                                        ShooterSubsystem shooter) {

        return shootToPoint(roller, conveyer, shooter, kicker, drivetrain)
            .alongWith(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));

    }

    public static Command autoAlignAndDrive(CommandSwerveDrivetrain drivetrain, SwarmDriveController xbox)
    {
        return drivetrain.applyRequest(() ->
                DriveConstants.drive.withVelocityX(-xbox.getYLimitedInput() * DriveConstants.MAX_STRAFE_SHOOT_SPEED) // Drive forward with negative Y (forward)
                    .withVelocityY(-xbox.getXLimitedInput() * DriveConstants.MAX_STRAFE_SHOOT_SPEED) // Drive left with negative X (left)
                    .withRotationalRate(TargetingHelper.getRotationSpeed()));
    }

    public static Command autoShootToTarget(RollerSubsystem roller,
                                        ConveyerSubsystem conveyer,
                                        CommandSwerveDrivetrain drivetrain,
                                        SwarmDriveController xbox,
                                        KickerSubsystem kicker,
                                        ShooterSubsystem shooter)
    {
        return autoAlignAndDrive(drivetrain, xbox).until(() -> isAlignedToHub()).andThen(autoShootAndPlant(roller, conveyer, drivetrain, kicker, shooter));
    }

    public static Command agitate(PivotSubsystem pivot){
        return pivot.agitate().withTimeout(PivotConstants.AGITATION_TIME)
            .andThen(pivot.deploy().withTimeout(PivotConstants.AGITATION_TIME));
    }

    public static Command shootInAuto(RollerSubsystem roller,
                                        ConveyerSubsystem conveyer,
                                        CommandSwerveDrivetrain drivetrain,
                                        SwarmDriveController xbox,
                                        double maxSpeed,
                                        KickerSubsystem kicker,
                                        ShooterSubsystem shooter,
                                        PivotSubsystem pivot){
        return autoShootToTarget(roller, conveyer, drivetrain, xbox, kicker, shooter)
        .alongWith(agitate(pivot));
    }

    public static Command driveToPoint(CommandSwerveDrivetrain drivetrain, Pose2d target){
        return new InstantCommand().alongWith(drivetrain.applyRequest(() ->
            DriveConstants.drive.withVelocityX(TargetingHelper.boundedPLoop(ClimberConstants.TRANSLATION_MIN, ClimberConstants.TRANSLATION_MAX, target.getX(), ClimberConstants.TRANSLATION_P, drivetrain.getCurrentPose().getX(), ClimberConstants.TRANSLATION_DEADBAND) * DriveConstants.MAX_SPEED) // Drive forward with negative Y (forward)
                .withVelocityY(TargetingHelper.boundedPLoop(ClimberConstants.TRANSLATION_MIN, ClimberConstants.TRANSLATION_MAX, target.getY(), ClimberConstants.TRANSLATION_P, drivetrain.getCurrentPose().getY(), ClimberConstants.TRANSLATION_DEADBAND) * DriveConstants.MAX_SPEED) // Drive left with negative X (left)
                .withRotationalRate(TargetingHelper.boundedPLoop(ClimberConstants.ROTATION_MIN, ClimberConstants.ROTATION_MAX, target.getRotation().getDegrees(), ClimberConstants.ROTATION_P, drivetrain.getCurrentPose().getRotation().getDegrees(), ClimberConstants.ROTATION_DEADBAND) * DriveConstants.MAX_ANGULAR) // Drive counterclockwise with negative X (left)
        ));
    }



}

