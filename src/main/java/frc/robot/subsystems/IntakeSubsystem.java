package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{

    private WPI_VictorSPX arm;
    private WPI_VictorSPX grabber;

    public IntakeSubsystem() {
        arm = new WPI_VictorSPX(Constants.armMotor);
        grabber = new WPI_VictorSPX(Constants.grabMotor);

        arm.setNeutralMode(NeutralMode.Brake);
        grabber.setNeutralMode(NeutralMode.Brake);
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
