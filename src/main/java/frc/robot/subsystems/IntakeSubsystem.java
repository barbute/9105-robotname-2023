package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{

    private CANSparkMax arm;
    private CANSparkMax grabber;

    public IntakeSubsystem() {
        arm = new CANSparkMax(Constants.armMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        grabber = new CANSparkMax(Constants.grabMotor, CANSparkMaxLowLevel.MotorType.kBrushless);

        arm.setIdleMode(IdleMode.kBrake);
        grabber.setIdleMode(IdleMode.kBrake);
    }

    public void lift() {
        arm.set(1);
    }

    public void down() {
        arm.set(-1);
    }

    public void grab() {
        grabber.set(1);
    }

    public void release() {
        grabber.set(-1);
    }
}
