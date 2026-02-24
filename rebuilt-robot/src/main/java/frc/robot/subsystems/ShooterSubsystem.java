package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PowerConstants;
import frc.robot.Constants.ShooterConstants;
@Logged
public class ShooterSubsystem extends SubsystemBase{
    
    TalonFX shooterLead = new TalonFX(ShooterConstants.SHOOTER_LEAD_ID);
    TalonFX shooterFollow = new TalonFX(ShooterConstants.SHOOTER_FOLLOW_ID);

    double voltageLead, voltageFollow;

    VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    VelocityVoltage request;

    PIDController closedloop = new PIDController(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD);

    public double incrementalShooterVolts = 8;

    public ShooterSubsystem(){
        voltageLead = 0;
        voltageFollow = 0;

        shooterLead.getConfigurator().apply(ShooterConstants.shooterConfig);
        shooterLead.getConfigurator().apply(PowerConstants.mid_high);

        shooterFollow.getConfigurator().apply(ShooterConstants.shooterConfig);
        shooterFollow.getConfigurator().apply(ShooterConstants.shooterFollowConfig);
        shooterFollow.getConfigurator().apply(PowerConstants.mid_high);

        shooterFollow.setControl(new Follower(shooterLead.getDeviceID(), MotorAlignmentValue.Opposed));

        request = new VelocityVoltage(1).withSlot(0);

    }

    public Command shoot(DoubleSupplier voltage){
        return this.runOnce(() -> 
            shooterLead.setControl(request.withVelocity(70))
        );
    }

    //DEBUGGING COMMANDS

    public void increaseShooterVolts(){
        if(incrementalShooterVolts < 12){
            incrementalShooterVolts += 0.25;
        }
    }

    public void decreaseShooterVolts(){
        if (incrementalShooterVolts > 0){
            incrementalShooterVolts -= 0.25;
        }
    }

}
