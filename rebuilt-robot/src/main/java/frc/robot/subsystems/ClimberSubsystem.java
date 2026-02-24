package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
    
    TalonFX climber = new TalonFX(ClimberConstants.CLIMBER_ID);

    DigitalInput climberSwitch = new DigitalInput(ClimberConstants.CLIMBER_LIMITSWITCH_CHANNEL);

    public ClimberSubsystem(){

        climber.getConfigurator().apply(ClimberConstants.climberConfig);

    }

    public Command raiseClimberLevelOne(){
        return this.run(() -> 
            climber.set(ClimberConstants.CLIMB_L1_SPEED)
        );
    }

    public Command homeClimber(){
        return this.run(() -> 
            climber.set(ClimberConstants.HOME_CLIMBER_SPEED)
        );
    }

    public boolean isClimberAtHome(){
        return climberSwitch.get();
    }



}
