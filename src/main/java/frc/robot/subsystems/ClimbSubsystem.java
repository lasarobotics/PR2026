package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    private final TalonFX m_climbMotor;
    private final PositionVoltage s_climbPositionRequest = new PositionVoltage(0.0);

    public ClimbSubsystem() {
        m_climbMotor = new TalonFX(Constants.ClimbConstants.CLIMB_MOTOR_ID);
        m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void moveTo(double rotations) {
      m_climbMotor.setControl(s_climbPositionRequest.withPosition(rotations));
    }

    public Command goToCommand(double rotations) {
        return runOnce(() -> moveTo(rotations));
    }

    public Command setUp() {
        return goToCommand(Constants.ClimbConstants.UP_SET_POINT);
    }

    public Command setBack() {
        return goToCommand(Constants.ClimbConstants.BACK_SET_POINT);
    }

    public Command setForward() {
        return goToCommand(Constants.ClimbConstants.FORWARD_SET_POINT);
    }

    @Override
    public void periodic() {
        
    }

    public void close() {
        m_climbMotor.close();
    }
}
