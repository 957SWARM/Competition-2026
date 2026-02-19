package frc.robot.commands;


import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.KickerSubsystem;

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

    public static Command shootToHub(RollerSubsystem roller, ConveyerSubsystem conveyer, ShooterSubsystem shooter, KickerSubsystem kicker){
        return roller.intakeCommand().alongWith(new WaitCommand(1).andThen(conveyer.runConveyerForwards())).alongWith(new WaitCommand(1).andThen(kicker.runKicker()));
    }

    public static Command shootFromNeutral(HoodSubsystem hood, ShooterSubsystem shooter, RollerSubsystem roller, ConveyerSubsystem conveyer, KickerSubsystem kicker){
        return shootToHub(roller, conveyer, shooter, kicker)
            .alongWith(hood.driveHood(() -> Constants.HoodConstants.FROM_NEUTRAL_ANGLE, conveyer.isConveyerRunningSupplier()))
            .alongWith(shooter.shoot(() -> Constants.ShooterConstants.FROM_NEUTRAL_VOLTAGE));
    }

    public static Command autoShootToHub(RollerSubsystem roller,
                                        ConveyerSubsystem conveyer,
                                        CommandSwerveDrivetrain drivetrain,
                                        CommandXboxController xbox,
                                        double maxSpeed,
                                        Supplier<Pose2d> hubPose,
                                        SwerveRequest.FieldCentric drive,
                                        KickerSubsystem kicker,
                                        ShooterSubsystem shooter)
    {
        return shootToHub(roller, conveyer, shooter, kicker).alongWith(drivetrain.applyRequest(() ->
                drive.withVelocityX(-xbox.getLeftY() * maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-xbox.getLeftX() * maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(TargetingHelper.getRotationSpeed(hubPose.get(), drivetrain.getCurrentPose())) // Drive counterclockwise with negative X (left)
            ));
    }



}
