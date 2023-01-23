
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OrientateSubsystem extends SubsystemBase {

    double kI = 0;
    double kD = 0;
    double kP = 0;
    double pidCalc;
    float navYaw;
    float setPoint;
  
    CANSparkMax leftFront;
    CANSparkMax leftBack;
    CANSparkMax rightFront;
    CANSparkMax rightBack;
    
    private AHRS navGyro = new AHRS(SPI.Port.kMXP);
    private PIDController pid = new PIDController(kP, kI, kD);  

    public OrientateSubsystem() {

        setPoint = 0;
        pid.setTolerance(1);

        leftFront = new CANSparkMax(Constants.leftFrontMotor, 
        CANSparkMaxLowLevel.MotorType.kBrushless);
        leftBack = new CANSparkMax(Constants.leftBackMotor, 
            CANSparkMaxLowLevel.MotorType.kBrushless);
        rightFront = new CANSparkMax(Constants.rightFrontMotor, 
            CANSparkMaxLowLevel.MotorType.kBrushless);
        rightBack = new CANSparkMax(Constants.rightBackMotor, 
            CANSparkMaxLowLevel.MotorType.kBrushless);

    }

    @Override
    public void periodic() {

        navYaw = navGyro.getYaw();
        pidCalc = pid.calculate(navYaw, setPoint);

        leftFront.set(pidCalc);
        leftBack.set(pidCalc);
        rightFront.set(pidCalc);
        rightBack.set(pidCalc);
        
    }
}
