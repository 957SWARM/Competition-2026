// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




//RIGHT HERE, C:\Competition-2026\rebuilt-robot, robot code file location!!!!!!!!!

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.Sequencing;
import frc.robot.commands.TargetingHelper;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
@Logged
public class RobotContainer {
  
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final SendableChooser<Command> autoChooser;

  private final HoodSubsystem hood = new HoodSubsystem();
  private final PivotSubsystem pivot = new PivotSubsystem();
  private final RollerSubsystem roller = new RollerSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ConveyerSubsystem conveyer = new ConveyerSubsystem();
  private final KickerSubsystem kicker = new KickerSubsystem();

  CommandXboxController xbox = new CommandXboxController(0);

  PIDController pid = new PIDController(0.25, 0, 0);



  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


     public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

     public PathPlannerAuto auto = null;
  
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    autoChooser.addOption("OG auto", new InstantCommand());


    configureBindings();
  }

  private void configureBindings() {
    pivot.setDefaultCommand(pivot.stow());
    roller.setDefaultCommand(roller.stopIntakeCommand());
    hood.setDefaultCommand(hood.driveHood(() -> TargetingHelper.getExpectedHoodPosition(getDistanceFromHub(drivetrain.getCurrentPose())), conveyer.isConveyerRunningSupplier()));
    shooter.setDefaultCommand(shooter.shoot(() -> TargetingHelper.getExpectedShooterVoltage(getDistanceFromHub(drivetrain.getCurrentPose()))));
    conveyer.setDefaultCommand(conveyer.stopConveyer());
    kicker.setDefaultCommand(kicker.stopKicker());

    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-xbox.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-xbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-xbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

    xbox.y().toggleOnTrue(pivot.deploy());
    xbox.a().toggleOnTrue(roller.intakeCommand());

    xbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
        xbox.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-xbox.getLeftY(), -xbox.getLeftX()))
        ));

    xbox.axisGreaterThan(3, ControllerConstants.TRIGGER_THRESHOLD).toggleOnTrue(Sequencing.shootToHub(roller, conveyer, shooter, kicker));
    xbox.axisGreaterThan(2, ControllerConstants.TRIGGER_THRESHOLD).toggleOnTrue(Sequencing.shootFromNeutral(hood, shooter, roller, conveyer, kicker));

    new Trigger(() -> !hood.hasBeenZeroed()).onTrue(Sequencing.zeroHood(hood));

    final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

    xbox.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    auto = new PathPlannerAuto("first auto");
  }

  public Command getAutonomousCommand() {
    return auto;
  }

  public double getDistanceFromHub(Pose2d robotPose){
    //Call limelight info to get distance
    Pose2d targetHub = new Pose2d(FieldConstants.RED_HUB_LOCATION.getX(), FieldConstants.RED_HUB_LOCATION.getY(), Rotation2d.fromDegrees(0));
    if(DriverStation.getAlliance().get() == Alliance.Blue){
      targetHub = new Pose2d(FieldConstants.BLUE_HUB_LOCATION.getX(), FieldConstants.BLUE_HUB_LOCATION.getY(), Rotation2d.fromDegrees(0));
    }
    double distance = TargetingHelper.getDistanceToGoalPose(drivetrain.getCurrentPose(), targetHub);
    return distance;
  }
 

  
}
