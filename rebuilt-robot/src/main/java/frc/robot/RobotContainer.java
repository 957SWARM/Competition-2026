// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  
  private final HoodSubsystem hood = new HoodSubsystem();
  private final PivotSubsystem pivot = new PivotSubsystem();
  private final RollerSubsystem roller = new RollerSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  CommandXboxController xbox = new CommandXboxController(0);

  double shooterSetpoint = 0;
  double hoodSetpoint = 0;
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    pivot.setDefaultCommand(pivot.stow());
    roller.setDefaultCommand(roller.stopIntakeCommand());
    hood.setDefaultCommand(hood.driveHood(() -> hoodSetpoint));
    shooter.setDefaultCommand(shooter.shoot(() -> shooterSetpoint));

    xbox.y().toggleOnTrue(pivot.deploy());
    xbox.a().toggleOnTrue(roller.intakeCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  
}
