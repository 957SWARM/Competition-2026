package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwarmDriveController extends CommandXboxController {
    
    SlewRateLimiter slewX;
    SlewRateLimiter slewY;
    SlewRateLimiter slewTheta;


    public SwarmDriveController(int port, double translationRate, double thetaRate){
        super(port);
        slewX = new SlewRateLimiter(translationRate);
        slewY = new SlewRateLimiter(translationRate);
        slewTheta = new SlewRateLimiter(thetaRate);
    }

    public double getXLimitedInput() {
        return slewX.calculate(getLeftX());
    }

    public double getYLimitedInput(){
        return slewY.calculate(getLeftY()); 
    }

    public double getThetaLimitedInput(){
        return slewTheta.calculate(getRightX());
    }

    public void resetSlew(){
        slewX.reset(0);
        slewY.reset(0);
        slewTheta.reset(0);
    }
}
