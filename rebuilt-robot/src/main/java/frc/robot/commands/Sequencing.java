package frc.robot.commands;


import java.util.function.Supplier;
import java.util.function.ToDoubleFunction;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SwarmDriveController;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Sequencing {
    
    public static Command zeroHood(HoodSubsystem hood){
        return new WaitCommand(0.5).andThen(hood.zeroEncoder()
                    .withTimeout(HoodConstants.ZEROING_TIME)
                    .andThen(
                        hood.zeroEncoder()
                        .until(() -> hood.isStopped())).andThen(() -> hood.homeHood()));
    }

    public static Command shootToPoint(RollerSubsystem roller, ConveyerSubsystem conveyer, ShooterSubsystem shooter, KickerSubsystem kicker, CommandSwerveDrivetrain drive){
        return roller.intakeCommand(drive).alongWith(new WaitCommand(1).andThen(conveyer.pulseConveyor().repeatedly())).alongWith(new WaitCommand(1).andThen(kicker.runKicker()));
    }


    public static Command autoShootToTarget(RollerSubsystem roller,
                                        ConveyerSubsystem conveyer,
                                        CommandSwerveDrivetrain drivetrain,
                                        SwarmDriveController xbox,
                                        double maxSpeed,
                                        KickerSubsystem kicker,
                                        ShooterSubsystem shooter)
    {
        return shootToPoint(roller, conveyer, shooter, kicker, drivetrain).alongWith(drivetrain.applyRequest(() ->
                DriveConstants.drive.withVelocityX(-xbox.getYLimitedInput() * maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-xbox.getXLimitedInput() * maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(TargetingHelper.getRotationSpeed()) // Drive counterclockwise with negative X (left)
            ));
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
        return autoShootToTarget(roller, conveyer, drivetrain, xbox, maxSpeed, kicker, shooter)
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

