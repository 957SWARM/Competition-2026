// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




//RIGHT HERE, C:\Competition-2026\rebuilt-robot, robot code file location!!!!!!!!!

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.annotation.Target;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.TargetingConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.DriveToClimbPoint;
import frc.robot.commands.Sequencing;
import frc.robot.commands.ShootSequencing;
import frc.robot.commands.TargetingHelper;
import frc.robot.enums.RobotData;
import frc.robot.enums.TargetingPoint;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
@Logged
@SuppressWarnings("unused")
public class RobotContainer {
  
  private double MaxSpeed = DriveConstants.MAX_SPEED; // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = DriveConstants.MAX_ANGULAR; // 3/4 of a rotation per second max angular velocity

  private final SendableChooser<Command> autoChooser;
  private Command Left1Neutral;
  private Command Left1Depot;
  private Command Left2NeutralDepot;
  private Command Left2NeutralBump;
  private Command Right1Neutral;

  public final Field2d field = new Field2d();

  private final HoodSubsystem hood = new HoodSubsystem();
  private final PivotSubsystem pivot = new PivotSubsystem();
  private final RollerSubsystem roller = new RollerSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ConveyerSubsystem conveyer = new ConveyerSubsystem();
  private final KickerSubsystem kicker = new KickerSubsystem();

  SwarmDriveController xbox = new SwarmDriveController(0, 2, 4);

  PIDController pid = new PIDController(0.25, 0, 0);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Command shootInAuto = ShootSequencing
                                        .autoAlignAndShootSequence(drivetrain, xbox, kicker, conveyer, roller)
                                        .alongWith(new WaitCommand(1.6)
                                        .andThen(pivot.trashCompact()));

    public PathPlannerAuto auto = null;
  
  public RobotContainer() {

    //shootInAuto.addRequirements(roller, conveyer, kicker, drivetrain);

    NamedCommands.registerCommand("Deploy Pivot", pivot.deploy());
    NamedCommands.registerCommand("Stow Pivot", pivot.stow());
    NamedCommands.registerCommand("Intake", roller.intakeCommand());
    NamedCommands.registerCommand("Shoot to Hub", ShootSequencing.autoAlignAndShootSequence(drivetrain, xbox, kicker, conveyer, roller).alongWith(new WaitCommand(1.6).andThen(pivot.trashCompact())));
    NamedCommands.registerCommand("Idle Ballpath", conveyer.idleConveyer().alongWith(kicker.idleKicker()));
    NamedCommands.registerCommand("Agitate", Sequencing.agitate(pivot).repeatedly());
    NamedCommands.registerCommand("Stop", Commands.runOnce(() -> drivetrain.setControl(DriveConstants.drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0))));
    //NamedCommands.registerCommand("To zone", null);

    // new EventTrigger("Shoot")
    //   .whileTrue(shootInAuto);
    // new EventTrigger("Stop")
    //   .onTrue(drivetrain.applyRequest(() -> DriveConstants.drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)));

    Left1Neutral = new PathPlannerAuto("Left 1 Neutral");
    Left1Depot = new PathPlannerAuto("Left 1 Depot");
    Left2NeutralDepot = new PathPlannerAuto("Left 2 Neutral Depot");
    Left2NeutralBump = new PathPlannerAuto("Left 2 Neutral Bump");

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putBoolean("Incremental Shooter Control", false);
    SmartDashboard.putBoolean("Disable Vision Localization", false);
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    autoChooser.addOption("Left 1 Neutral", Left1Neutral);
    autoChooser.addOption("Left 1 Depot", Left1Depot);
    autoChooser.addOption("Left 2 Neutral Depot", Left2NeutralDepot);

    configureBindings();
  }

  private void configureBindings() {
    pivot.setDefaultCommand(pivot.deploy());
    roller.setDefaultCommand(roller.stopIntakeCommand());
    hood.setDefaultCommand(hood.driveHood(() -> getExpectedHoodPosition(), kicker.isKickerFeeding()));
    //hood.setDefaultCommand(hood.driveHood(() -> hood.incrementalHoodPos, () -> true));
    shooter.setDefaultCommand(shooter.shoot(() -> getExpectedShooterVelocity()));
    //shooter.setDefaultCommand(shooter.shoot(() -> shooter.incrementalShooterVel));
    conveyer.setDefaultCommand(conveyer.idleConveyer()); 
    kicker.setDefaultCommand(kicker.idleKicker());

    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                DriveConstants.drive.withVelocityX(-xbox.getYLimitedInput() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-xbox.getXLimitedInput() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-xbox.getThetaLimitedInput() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

    xbox.leftBumper().toggleOnTrue(pivot.stow());
    xbox.rightBumper().toggleOnTrue(roller.intakeCommand());
    xbox.b().toggleOnTrue(roller.ejectCommand().alongWith(conveyer.runConveyerBackwards()));
    xbox.a().whileTrue(Sequencing.agitate(pivot).repeatedly());
    xbox.a().toggleOnFalse(pivot.deploy());
    xbox.x().whileTrue(new DriveToClimbPoint(drivetrain));
    xbox.y().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(DriveConstants.MAX_SPEED * 0.1)
            .withTargetDirection(Rotation2d.fromDegrees(Math.floor(RobotData.botPose.getRotation().getDegrees() / 90.0) * 90.0 + 45.0))
            .withVelocityX(-xbox.getYLimitedInput() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-xbox.getXLimitedInput() * MaxSpeed)
            .withHeadingPID(10, 0, 0)));
    xbox.leftTrigger().whileTrue(ShootSequencing.shootNoLockSequence(kicker, conveyer, roller));

    // new Trigger(xbox.povDown().and(() -> Sequencing.isDriving(xbox))).whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.FieldCentricFacingAngle()
    //         .withDeadband(DriveConstants.MAX_SPEED * 0.1)
    //         .withTargetDirection(Rotation2d.fromRadians(xbox.getAngleOfMotion()))
    //         .withVelocityX(-xbox.getYLimitedInput() * MaxSpeed) // Drive forward with negative Y (forward)
    //         .withVelocityY(-xbox.getXLimitedInput() * MaxSpeed)
    //         .withHeadingPID(10, 0, 0)));

    //xbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //xbox.b().whileTrue(drivetrain.applyRequest(() ->
            //point.withModuleDirection(new Rotation2d(-xbox.getLeftY(), -xbox.getLeftX()))
        //));

    //xbox.axisGreaterThan(3, ControllerConstants.TRIGGER_THRESHOLD).toggleOnTrue(Sequencing.shootToHub(roller, conveyer, shooter, kicker));
    //xbox.axisGreaterThan(2, ControllerConstants.TRIGGER_THRESHOLD).toggleOnTrue(Sequencing.shootFromNeutral(hood, shooter, roller, conveyer, kicker));
    
    new Trigger(() -> !hood.hasBeenZeroed() && DriverStation.isEnabled()).whileTrue(Sequencing.zeroHood(hood));
    
    final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

    //Zero gyro, Set robot orientation in attempt to uncook MT2
    xbox.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)
    .andThen(Commands.runOnce(() -> LimelightHelpers.SetRobotOrientation("limelight", 0, 0, 0, 0, 0, 0))));

    new Trigger((xbox.rightTrigger().and(() -> !TargetingHelper.isDriving(xbox)))).whileTrue(ShootSequencing.autoAlignAndShootSequence(drivetrain, xbox, kicker, conveyer, roller).alongWith(new WaitCommand(1.6).andThen(pivot.trashCompact())));
    //new Trigger((xbox.rightTrigger().and(() -> Sequencing.isDriving(xbox)))).whileTrue(Sequencing.autoAlignAndDrive(drivetrain, xbox));
    new Trigger((xbox.rightTrigger().and(() -> TargetingHelper.isDriving(xbox)))).whileTrue(ShootSequencing.shootOnMoveSequence(drivetrain, xbox, kicker, conveyer, roller));
    
    Trigger incrementShoot = new Trigger(() -> SmartDashboard.getBoolean("Incremental Shooter Control", false));

    //DEBUGGING TRIGGERS

    incrementShoot
      .onTrue(Commands.runOnce(() -> shooter.incrementalShooterVel = shooter.getShooterVelocity())
      .alongWith(Commands.runOnce(() -> hood.incrementalHoodPos = hood.getHoodPosition())));
    incrementShoot
      .whileTrue(shooter.shoot(() -> shooter.incrementalShooterVel)
      .alongWith(hood.driveHood(() -> hood.incrementalHoodPos, () -> true)));

    new Trigger(xbox.povRight().and(incrementShoot)).onTrue(Commands.runOnce(() -> hood.increaseHoodPosition()));
    new Trigger(xbox.povLeft().and(incrementShoot)).onTrue(Commands.runOnce(() -> hood.decreaseHoodPosition()));

    new Trigger(xbox.povUp().and(incrementShoot)).onTrue(Commands.runOnce(() -> shooter.increaseShooterVel()));
    new Trigger(xbox.povDown().and(incrementShoot)).onTrue(Commands.runOnce(() -> shooter.decreaseShooterVel()));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void updateDrivebaseOdemetry(){

    //Working Megatag1 Code (More reliable, less accurate)
      if(LimelightHelpers.getTV("limelight") 
      && LimelightHelpers.getTA("limelight") > 0.11 
      && !SmartDashboard.getBoolean("Disable Vision Localization", false)){
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        //REMOVED FOR AUTO TESTING
        drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, TargetingConstants.VISION_STD_DEVS);
      }
    
    //Megatag2 (Still iffy, somethings up with the rotation still)
    // LimelightHelpers.SetRobotOrientation("limelight", drivetrain.getCurrentPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    // if(LimelightHelpers.getTV("limelight")){
    //   LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    //   drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
    // }

    field.setRobotPose(drivetrain.getCurrentPose());
  }

  public double getExpectedShooterVelocity(){
    return TargetingHelper.getExpectedShooterVelocity(RobotData.distanceToTarget);
  }

  public double getExpectedHoodPosition(){
    return TargetingHelper.getExpectedHoodPosition(RobotData.distanceToTarget);
  }

  public CommandScheduler getCurrentCommand(){
    return CommandScheduler.getInstance();
  }

  public SwerveDriveState getDriveState(){
    return drivetrain.getState();
  }

  public double getDistanceToTarget(){
    return RobotData.distanceToTarget;
  }

  public double getAngleToHub(){
    return RobotData.angleToTarget.getDegrees();
  }

  public boolean isAlignedToHub(){
    return TargetingHelper.isAlignedToTarget();
  }

  public Command updateShootAndHood(){
    return Commands.run(() -> shooter.shoot(() -> getExpectedShooterVelocity()).alongWith(hood.driveHood(() -> getExpectedHoodPosition(), kicker.isKickerFeeding())));
  }
  
}
