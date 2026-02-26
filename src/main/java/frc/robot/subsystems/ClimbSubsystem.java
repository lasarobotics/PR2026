package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

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
                if (getInstance().dioInput.get())
                {
                    getInstance().m_climbMotor.setControl(new VoltageOut(-6));
                }
                else
                {
                    getInstance().m_climbMotor.setControl(new VoltageOut(0));
                    getInstance().m_climbMotor.setPosition(0);
                }
            }
            @Override
            public void execute(){

                if(!getInstance().dioInput.get()){
                    getInstance().m_climbMotor.setControl(new VoltageOut(0));
                    getInstance().m_climbMotor.setPosition(0);
                }
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
                if(!getInstance().dioInput.get()){
                    return R2C;
                }
                return START;
            }
        },
        R2C {
            @Override
            public void initialize() {
                getInstance().m_climbMotor.setControl(Constants.ClimbConstants.R2C_SET_POINT);
                DriveSubsystem.postClimbZero();

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
                DriveSubsystem.wheelPushTower();
            }
            //TODO make sure wheels are straight when climbing or else!!! :c
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
                DriveSubsystem.wheelPushTower();
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
    // Ready To Climb
    private BooleanSupplier m_R2CButton;
    private BooleanSupplier m_L2Button;
    //initial starting position - zero
    private BooleanSupplier m_startButton;
    private BooleanSupplier m_positiveVoltageButton;
    private BooleanSupplier m_negativeVoltageButton;

    private DigitalInput dioInput;

    public static ClimbSubsystem getInstance() {
        if (s_climbInstance == null) {
            s_climbInstance = new ClimbSubsystem();
        }
        return s_climbInstance;
    }

    private ClimbSubsystem() {
        super(ClimbStates.START);
        dioInput = new DigitalInput(9);
        m_climbMotor = new TalonFX(Constants.ClimbConstants.CLIMB_MOTOR_ID);
    
        TalonFXConfiguration climbConfiguration = new TalonFXConfiguration();

        climbConfiguration
            .CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(80)
                .withSupplyCurrentLimit(80)
                .withSupplyCurrentLowerLimit(40);
                
        climbConfiguration
            .Slot0
                .withKP(2.5)
                .withKD(0);
        climbConfiguration
            .MotorOutput
                .NeutralMode = NeutralModeValue.Brake;
        m_climbMotor.getConfigurator().apply(climbConfiguration);
        m_climbMotor.setPosition(0);
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
            m_climbMotor.setControl(new VoltageOut(12)); 
            testingControl = true;
        }
        else if (m_negativeVoltageButton.getAsBoolean())
        {
            m_climbMotor.setControl(new VoltageOut(-12));
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
        Logger.recordOutput(getName() + "/port9", dioInput.get());
    }

}