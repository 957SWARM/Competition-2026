package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PowerConstants;
import frc.robot.commands.TargetingHelper;
@Logged
public class RollerSubsystem extends SubsystemBase{
    
    TalonFXS intake;

    public double voltage;

    public RollerSubsystem(){
        intake = new TalonFXS(IntakeConstants.INTAKE_ID);

        

        intake.getConfigurator().apply(IntakeConstants.rollerConfig);
        intake.getConfigurator().apply(PowerConstants.mid_low);

        voltage = 0;
    }

    public Command intakeCommand(CommandSwerveDrivetrain drive){
        return this.run(() -> 
            intake.setVoltage(IntakeConstants.INTAKE_VOLTAGE + (IntakeConstants.INTAKE_FROM_SPEED_SCALAR * TargetingHelper.getDriveSpeed(drive)))
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
