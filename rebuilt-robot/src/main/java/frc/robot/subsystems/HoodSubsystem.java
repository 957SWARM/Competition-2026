package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.PowerConstants;
@Logged
public class HoodSubsystem extends SubsystemBase{
    
    CANcoder hoodEncoder = new CANcoder(HoodConstants.ENCODER_ID);

    TalonSRX hood = new TalonSRX(HoodConstants.HOOD_ID);

    boolean zeroed = false;

    public HoodSubsystem(){
        hood.configRemoteFeedbackFilter(HoodConstants.ENCODER_ID, RemoteSensorSource.CANCoder, 0);
        hood.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, 0);

        hood.configPeakCurrentLimit(PowerConstants.HOOD_LIMIT);
    }

    public boolean isStopped(){
        return Math.abs(hoodEncoder.getVelocity().getValueAsDouble()) < 0.08;
    }

    public boolean hasBeenZeroed(){
        return zeroed;
    }
  
    public void resetHood(){
        hoodEncoder.setPosition(0);
    }

    public Command zeroEncoder(){
        return this.run(() -> 
            hood.set(ControlMode.PercentOutput, HoodConstants.ZEROING_SPEED)
        );
    }

    public Command driveHood(DoubleSupplier positionSupplier, BooleanSupplier conveyerSupplier){
        return this.run(() -> {
            if(conveyerSupplier.getAsBoolean()){
                hood.set(ControlMode.MotionMagic, positionSupplier.getAsDouble());
            } else {
                resetHood();
            }
        }
      
        );
    }



}
