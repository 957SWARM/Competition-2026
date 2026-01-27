package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class RollerSubsystem extends SubsystemBase{
    
    TalonFX intake;

    public double voltage;

    public RollerSubsystem(){
        intake = new TalonFX(IntakeConstants.INTAKE_ID);

        voltage = 0;
    }

    public Command intakeCommand(){
        return this.run(() -> 
            intake.setVoltage(IntakeConstants.INTAKE_VOLTAGE)
        );
    }
    
    public Command ejectCommand(){
        return this.run(() -> 
            intake.setVoltage(IntakeConstants.EJECT_VOLTAGE)
        );
    }

    public Command stopIntakeCommand(){
        return this.run(() -> 
            intake.setVoltage(0)
        );
    }

    public double getVoltage(){
        return voltage;
    }


}
