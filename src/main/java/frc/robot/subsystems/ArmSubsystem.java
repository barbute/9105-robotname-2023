
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase 
{
    private CANSparkMax armMotor;
    private CANSparkMax handMotor;

    private RelativeEncoder armEncoder;
    private RelativeEncoder handEncoder;

    public ArmSubsystem()
    {
        armMotor = new CANSparkMax(Constants.IntakeConstants.armMotor, 
        CANSparkMaxLowLevel.MotorType.kBrushless);
        handMotor = new CANSparkMax(Constants.IntakeConstants.handMotor, 
        CANSparkMaxLowLevel.MotorType.kBrushless);

        armEncoder = armMotor.getEncoder();
        handEncoder = handMotor.getEncoder();

        armMotor.setIdleMode(IdleMode.kBrake);
        handMotor.setIdleMode(IdleMode.kBrake);

    }

    //Note: Get conversion factor later
    public double getHandEncoder()
    {
        return handEncoder.getPosition() * Constants.IntakeConstants.kLinearDistanceConversionFactor;
    }

    public double getArmEncoder()
    {
        return armEncoder.getPosition() * Constants.IntakeConstants.kLinearDistanceConversionFactor;
    }

    public void setArmMotorSpeed(double motorSpeed)
    {
        armMotor.set(motorSpeed);
    }

    public void setHandMotorSped(double motorSpeed)
    {
        handMotor.set(motorSpeed);
    }
}
