package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants;

public class ClimbSubsystem extends StateMachine{

    public enum ClimbStates implements SystemState {
        NOTHING {
            @Override
            public SystemState nextState() {
                return this;
            }
        },
        UP {
            @Override
            public void initialize() {
                getInstance().m_climbMotor.setControl(Constants.ClimbConstants.UP_SET_POINT);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_backButton.getAsBoolean()) {
                    return BACK;
                }
                if (getInstance().m_forwardButton.getAsBoolean()) {
                    return FORWARD;
                }
                return UP;
            }
        },
        FORWARD {
            @Override
            public void initialize() {
                getInstance().m_climbMotor.setControl(Constants.ClimbConstants.FORWARD_SET_POINT);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_upButton.getAsBoolean()) {
                    return UP;
                }
                if (getInstance().m_backButton.getAsBoolean()) {
                    return BACK;
                }
                return FORWARD;
            }

            
        },
        BACK {
            @Override
            public void initialize() {
                getInstance().m_climbMotor.setControl(Constants.ClimbConstants.BACK_SET_POINT);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_upButton.getAsBoolean()) {
                    return UP;
                }
                if (getInstance().m_forwardButton.getAsBoolean()) {
                    return FORWARD;
                }
                return BACK;
            }
        }
    }

    private static ClimbSubsystem s_climbInstance;
    private final TalonFX m_climbMotor;

    private BooleanSupplier m_upButton;
    private BooleanSupplier m_backButton;
    private BooleanSupplier m_forwardButton;
    private BooleanSupplier m_positiveVoltageButton;
    private BooleanSupplier m_negativeVoltageButton;

    public static ClimbSubsystem getInstance() {
        if (s_climbInstance == null) {
            s_climbInstance = new ClimbSubsystem();
        }
        return s_climbInstance;
    }

    private ClimbSubsystem() {
        super(ClimbStates.UP);
        m_climbMotor = new TalonFX(Constants.ClimbConstants.CLIMB_MOTOR_ID);

        TalonFXConfiguration climbConfiguration = new TalonFXConfiguration();

        climbConfiguration
            .CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(5)
                .withSupplyCurrentLimit(1)
                .withSupplyCurrentLowerLimit(1);
                
        climbConfiguration
            .Slot0
                .withKP(5)
                .withKD(1);

        m_climbMotor.getConfigurator().apply(climbConfiguration);
    }

    public void configureBindings (
        BooleanSupplier upButton,
        BooleanSupplier backButton,
        BooleanSupplier forwardButton,
        BooleanSupplier positiveVoltageButton,
        BooleanSupplier negativeVoltageButton
    ) {
        m_upButton = upButton;
        m_backButton = backButton;
        m_forwardButton = forwardButton;
        m_positiveVoltageButton = positiveVoltageButton;
        m_negativeVoltageButton = negativeVoltageButton;
    }

    @Override
    public void periodic() {

        if (m_positiveVoltageButton.getAsBoolean())
            m_climbMotor.setControl(new VoltageOut(0.5)); 
        else if (m_negativeVoltageButton.getAsBoolean())
            m_climbMotor.setControl(new VoltageOut(-0.5));


        Logger.recordOutput(getName() + "/buttons/Up", m_upButton);
        Logger.recordOutput(getName() + "/buttons/Back", m_backButton);
        Logger.recordOutput(getName() + "/buttons/Forward", m_forwardButton);
        Logger.recordOutput(getName() + "/state", getState().toString());
        Logger.recordOutput(getName() + "/currentPosition", m_climbMotor.getPosition().getValueAsDouble());
    }

}