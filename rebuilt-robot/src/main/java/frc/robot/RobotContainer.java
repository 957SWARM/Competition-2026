// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Sequencing;
import frc.robot.commands.TargetingHelper;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  
  private final HoodSubsystem hood = new HoodSubsystem();
  private final PivotSubsystem pivot = new PivotSubsystem();
  private final RollerSubsystem roller = new RollerSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final Conveyer conveyer = new Conveyer();

  CommandXboxController xbox = new CommandXboxController(0);

  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    pivot.setDefaultCommand(pivot.stow());
    roller.setDefaultCommand(roller.stopIntakeCommand());
    hood.setDefaultCommand(hood.driveHood(() -> TargetingHelper.getExpectedHoodPosition(getDistanceFromHub())));
    shooter.setDefaultCommand(shooter.shoot(() -> TargetingHelper.getExpectedShooterVoltage(getDistanceFromHub())));
    conveyer.setDefaultCommand(conveyer.stopConveyer());

    xbox.y().toggleOnTrue(pivot.deploy());
    xbox.a().toggleOnTrue(roller.intakeCommand());

    xbox.axisGreaterThan(3, ControllerConstants.TRIGGER_THRESHOLD).toggleOnTrue(Sequencing.shootToHub(roller, conveyer));
    xbox.axisGreaterThan(2, ControllerConstants.TRIGGER_THRESHOLD).toggleOnTrue(Sequencing.shootFromNeutral(hood, shooter, roller, conveyer));

    new Trigger(() -> !hood.hasBeenZeroed()).onTrue(Sequencing.zeroHood(hood));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private double getDistanceFromHub(){
    //Call limelight info to get distance
    return 5;
  }

  
}
