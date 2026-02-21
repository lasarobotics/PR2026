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
        START {
            @Override
            public void initialize() {
                getInstance().m_climbMotor.setControl(Constants.ClimbConstants.START_SET_POINT);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_L1Button.getAsBoolean()) {
                    return L1;
                }
                if (getInstance().m_L2Button.getAsBoolean()) {
                    return L2;
                }
                if (getInstance().m_R2CButton.getAsBoolean()) {
                    return R2C;
                }
                return START;
            }
        },
        R2C {
            @Override
            public void initialize() {
                getInstance().m_climbMotor.setControl(Constants.ClimbConstants.R2C_SET_POINT);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_L1Button.getAsBoolean()) {
                    return L1;
                }
                if (getInstance().m_L2Button.getAsBoolean()) {
                    return L2;
                }
                if (getInstance().m_startButton.getAsBoolean()) {
                    return START;
                }
                return R2C;
            }
        },
        L1 {
            @Override
            public void initialize() {
                getInstance().m_climbMotor.setControl(Constants.ClimbConstants.L1_SET_POINT);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_L2Button.getAsBoolean()) {
                    return L2;
                }
                if (getInstance().m_startButton.getAsBoolean()) {
                    return START;
                }
                if (getInstance().m_R2CButton.getAsBoolean()) {
                    return R2C;
                }
                return L1;
            }

            
        },
        L2 {
            @Override
            public void initialize() {
                getInstance().m_climbMotor.setControl(Constants.ClimbConstants.L2_SET_POINT);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_L1Button.getAsBoolean()) {
                    return L1;
                }
                if (getInstance().m_startButton.getAsBoolean()) {
                    return START;
                }
                if (getInstance().m_R2CButton.getAsBoolean()) {
                    return R2C;
                }
                return L2;
            }
        }
    }

    private static ClimbSubsystem s_climbInstance;
    private final TalonFX m_climbMotor;
    private boolean testingControl;

    private BooleanSupplier m_L1Button;
    private BooleanSupplier m_R2CButton;
    private BooleanSupplier m_L2Button;
    private BooleanSupplier m_startButton;
    private BooleanSupplier m_positiveVoltageButton;
    private BooleanSupplier m_negativeVoltageButton;

    public static ClimbSubsystem getInstance() {
        if (s_climbInstance == null) {
            s_climbInstance = new ClimbSubsystem();
        }
        return s_climbInstance;
    }

    private ClimbSubsystem() {
        super(ClimbStates.L1);
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
        BooleanSupplier L1Button,
        BooleanSupplier R2CButton,
        BooleanSupplier L2Button,
        BooleanSupplier startButton,
        BooleanSupplier positiveVoltageButton,
        BooleanSupplier negativeVoltageButton
    ) {
        m_L1Button = L1Button;
        m_R2CButton = R2CButton;
        m_L2Button = L2Button;
        m_startButton = startButton;
        m_positiveVoltageButton = positiveVoltageButton;
        m_negativeVoltageButton = negativeVoltageButton;
    }

    @Override
    public void periodic() {

        if (m_positiveVoltageButton.getAsBoolean())
        {
            m_climbMotor.setControl(new VoltageOut(1.5)); 
            testingControl = true;
        }
        else if (m_negativeVoltageButton.getAsBoolean())
        {
            m_climbMotor.setControl(new VoltageOut(-1.5));
            testingControl = true;
        }
        else if (testingControl)
        {
            m_climbMotor.setControl(new VoltageOut(0));
            testingControl = false;
        }

        Logger.recordOutput(getName() + "/buttons/L1", m_L1Button);
        Logger.recordOutput(getName() + "/buttons/Back", m_startButton);
        Logger.recordOutput(getName() + "/buttons/L2", m_L2Button);
        Logger.recordOutput(getName() + "/buttons/L2", m_R2CButton);
        Logger.recordOutput(getName() + "/state", getState().toString());
        Logger.recordOutput(getName() + "/currentPosition", m_climbMotor.getPosition().getValueAsDouble());
    }

}