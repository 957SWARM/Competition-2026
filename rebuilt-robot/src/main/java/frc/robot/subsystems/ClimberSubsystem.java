package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class ClimberSubsystem extends SubsystemBase{
    
    TalonFX climber = new TalonFX(ClimberConstants.CLIMBER_ID);

    private TimeOfFlight tof;
    LinearFilter tofFilter = LinearFilter.singlePoleIIR(0.1, Units.millisecondsToSeconds(30));

    private double tofRange;

    public ClimberSubsystem(){

        climber.getConfigurator().apply(ClimberConstants.climberConfig);

        tof = new TimeOfFlight(ClimberConstants.TOF_ID);
        tof.setRangingMode(RangingMode.Short, 30);

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

    public void periodic(){
        tofRange = tof.getRange();
    }

    public boolean checkToF(){
        return tofFilter.calculate(tofRange) <= ClimberConstants.TOF_THRESHOLD;
    }

    public double readToF(){
        return tof.getRange();
    }




}
