package frc.robot.commands;


import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.HoodConstants;
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
        return hood.zeroEncoder()
                    .withTimeout(HoodConstants.ZEROING_TIME)
                    .andThen(
                        hood.zeroEncoder()
                        .until(() -> hood.isStopped())).andThen(() -> hood.homeHood());
    }

    public static Command shootToPoint(RollerSubsystem roller, ConveyerSubsystem conveyer, ShooterSubsystem shooter, KickerSubsystem kicker){
        return roller.intakeCommand().alongWith(new WaitCommand(1).andThen(conveyer.pulseConveyor().repeatedly())).alongWith(new WaitCommand(1).andThen(kicker.runKicker()));
    }


    public static Command autoShootToTarget(RollerSubsystem roller,
                                        ConveyerSubsystem conveyer,
                                        CommandSwerveDrivetrain drivetrain,
                                        CommandXboxController xbox,
                                        double maxSpeed,
                                        Supplier<Pose2d> targetPose,
                                        SwerveRequest.FieldCentric drive,
                                        KickerSubsystem kicker,
                                        ShooterSubsystem shooter)
    {
        return shootToPoint(roller, conveyer, shooter, kicker).alongWith(drivetrain.applyRequest(() ->
                drive.withVelocityX(-xbox.getLeftY() * maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-xbox.getLeftX() * maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(TargetingHelper.getAdjustedRotationSpeed(TargetingHelper.rotationWithOffSet(drivetrain.getCurrentPose(), (Pose2d) targetPose, xbox, drivetrain.getState().Speeds.vyMetersPerSecond, maxSpeed),
                                                                                 TargetingHelper.getAngleToGoalPose(drivetrain.getCurrentPose().getRotation(), TargetingHelper.getPassTargetRotation2d(drivetrain.getCurrentPose()).getRotation()))) // Drive counterclockwise with negative X (left)
            ));
    }

    public static Command agitate(PivotSubsystem pivot){
        return pivot.agitate().withTimeout(PivotConstants.AGITATION_TIME)
            .andThen(pivot.deploy().withTimeout(PivotConstants.AGITATION_TIME));
    }



}
