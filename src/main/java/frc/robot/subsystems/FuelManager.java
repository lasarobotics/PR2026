package frc.robot.subsystems;


import java.util.function.BooleanSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class FuelManager extends StateMachine implements AutoCloseable {

    public enum FuelManagerStates implements SystemState {
        NOTHING {
            @Override
            public SystemState nextState() {
                return this;
            }
        },
        REST {
            @Override
            public void initialize() {
                getInstance().m_intakeMotor.set(0);
                getInstance().m_shootMotor.set(0);
                getInstance().m_middleMotor.set(0);
            }
            @Override
            public SystemState nextState() {
                if (getInstance().m_intakeButton.getAsBoolean()){
                    return INTAKE;
                }
                if (getInstance().m_shootButton.getAsBoolean()){
                    return SHOOT;
                }
                return REST;
            }
        },
        INTAKE {
            @Override
            public void initialize() {
                getInstance().m_intakeMotor.set(Constants.FuelManagerConstants.INTAKE_MOTOR_SPEED);
                getInstance().m_middleMotor.set(Constants.FuelManagerConstants.MIDDLE_MOTOR_INTAKE_SPEED);
            }
            @Override
            public SystemState nextState() {
                if (getInstance().m_intakeButton.getAsBoolean()){
                    return this;
                }
                return REST;
            }
        },
        SHOOT {
            @Override
            public void initialize() {
                getInstance().m_shootMotor.set(Constants.FuelManagerConstants.SHOOT_MOTOR_SPEED);
                getInstance().m_middleMotor.set(Constants.FuelManagerConstants.MIDDLE_MOTOR_SHOOT_SPEED);
                getInstance().m_intakeMotor.set(Constants.FuelManagerConstants.INTAKE_MOTOR_SPEED);
            }
            @Override
            public SystemState nextState() {
                if (getInstance().m_shootButton.getAsBoolean()){
                    return this;
                }
                return REST;
            }
        }
    }
    
    private static FuelManager s_FuelManagerInstance;
    private final TalonFX m_intakeMotor;
    private final TalonFX m_shootMotor;
    private final TalonFX m_middleMotor;
    private BooleanSupplier m_intakeButton;
    private BooleanSupplier m_shootButton;

    private FuelManager(){
        super(FuelManagerStates.REST);

        m_intakeMotor = new TalonFX(Constants.FuelManagerConstants.INTAKE_MOTOR_ID);
        m_shootMotor =  new TalonFX(Constants.FuelManagerConstants.SHOOT_MOTOR_ID);
        m_middleMotor =  new TalonFX(Constants.FuelManagerConstants.MIDDLE_MOTOR_ID);
    }

    public static FuelManager getInstance(){
        if(s_FuelManagerInstance == null){
            s_FuelManagerInstance = new FuelManager();
        }
        return s_FuelManagerInstance;
    }

    public void configureBindings(BooleanSupplier intakeButton, BooleanSupplier shootButton){
        m_intakeButton = intakeButton;
        m_shootButton = shootButton;
    }

    @Override
    public void periodic(){
        Logger.recordOutput(getName() + "/Intake Button", m_intakeButton);
        Logger.recordOutput(getName() + "/Shoot Button", m_shootButton);
        Logger.recordOutput(getName() + "/Current State", getState().toString());
        Logger.recordOutput(getName() + "/Inake Motor Speed", m_intakeMotor.getVelocity().getValueAsDouble());
    }

    @Override
    public void close(){
        m_intakeMotor.close();
        m_shootMotor.close();
        m_middleMotor.close();
    }
}
