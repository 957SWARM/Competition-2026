package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PowerConstants;

@Logged
public class PivotSubsystem extends SubsystemBase{

    TalonFX pivot = new TalonFX(PivotConstants.MOTOR_ID);
    CANcoder encoder = new CANcoder(PivotConstants.ENCODER_ID);

    MotionMagicVoltage request;
    

    public PivotSubsystem(){
        pivot.getConfigurator().apply(PivotConstants.config);
        pivot.getConfigurator().apply(PowerConstants.low);

        request = new MotionMagicVoltage(0);
    }

    public double getPosition(){
        return encoder.getPosition().getValueAsDouble();
    }

    public Command deploy(){
        return this.run(() ->
            pivot.setControl(request.withPosition(PivotConstants.DEPLOY_ANGLE))
        );
    }

    public Command stow(){
        return this.run(() -> 
            pivot.setControl(request.withPosition(PivotConstants.DEPLOY_ANGLE))
        );
    }

    

   /*  public Command agitate(){
        return this.run(() -> 
            
        );
    } */






}
