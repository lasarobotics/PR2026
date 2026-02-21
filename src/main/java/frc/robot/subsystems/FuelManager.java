package frc.robot.subsystems;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants;

public class FuelManager extends StateMachine{

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
                 getInstance().m_shootMotorLeader.set(0);
                getInstance().m_middleMotor.set(0);
                getInstance().m_intakeMotor.set(0);
            }
            @Override
            public SystemState nextState() {
                if (getInstance().m_intakeButton.getAsBoolean()){
                    return INTAKE;
                }
                if (getInstance().m_shootButton.getAsBoolean()){
                    return SHOOT;
                }
                if (getInstance().m_staticShootButton.getAsBoolean()){
                    return STATIC_SHOOT;
                }
                return REST;
            }
        },
        INTAKE {
            @Override
            public void initialize() {
                getInstance().m_middleMotor.setControl(getInstance().m_shooterVelocityVoltage.withVelocity(Constants.FuelManagerConstants.MIDDLE_MOTOR_INTAKE_SPEED)); // TODO add vraible speed
                getInstance().m_intakeMotor.setControl(getInstance().m_shooterVelocityVoltage.withVelocity(Constants.FuelManagerConstants.INTAKE_MOTOR_SPEED));
            }
            @Override
            public SystemState nextState() {
                if (getInstance().m_intakeButton.getAsBoolean()){
                    return INTAKE;
                }
                return REST;
            }
        },
        SHOOT {
            @Override
            public void initialize() {
                getInstance().m_shootMotorLeader.setControl(getInstance().m_shooterVelocityVoltage.withVelocity(Constants.FuelManagerConstants.SHOOT_MOTOR_SPEED));
                getInstance().m_intakeMotor.setControl(getInstance().m_shooterVelocityVoltage.withVelocity(Constants.FuelManagerConstants.INTAKE_MOTOR_SPEED));
            }

            @Override
            public void execute() 
            {   
                if (Math.abs(getInstance().m_shootMotorFollower.getRotorVelocity().getValueAsDouble() - Constants.FuelManagerConstants.SHOOT_MOTOR_SPEED) <= Math.abs(Constants.FuelManagerConstants.SHOOT_MOTOR_SPEED) * Constants.FuelManagerConstants.SHOOTER_WITHIN_RANGE_COEFFICIENT)
                {
                    getInstance().m_middleMotor.setControl(getInstance().m_shooterVelocityVoltage.withVelocity(Constants.FuelManagerConstants.MIDDLE_MOTOR_SHOOT_SPEED)); // TODO add vraible speed
                }
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_shootButton.getAsBoolean()){
                    return SHOOT;
                }
                return REST;
            }
        
        },
        STATIC_SHOOT {
            @Override
            public void initialize() {
                getInstance().m_shootMotorLeader.setControl(getInstance().m_shooterVelocityVoltage.withVelocity(Constants.FuelManagerConstants.SHOOT_MOTOR_SPEED));
                getInstance().m_middleMotor.setControl(getInstance().m_shooterVelocityVoltage.withVelocity(Constants.FuelManagerConstants.MIDDLE_MOTOR_SHOOT_SPEED)); // TODO add vraible speed
                getInstance().m_intakeMotor.setControl(getInstance().m_shooterVelocityVoltage.withVelocity(Constants.FuelManagerConstants.INTAKE_MOTOR_SPEED));
            }
            @Override
            public SystemState nextState() {
                if (getInstance().m_staticShootButton.getAsBoolean()){
                    return STATIC_SHOOT;
                }
                return REST;
            }
    }
}
    
    private static FuelManager s_FuelManagerInstance;
    private final TalonFX m_intakeMotor;
    private final TalonFX m_shootMotorLeader;
    private final TalonFX m_shootMotorFollower;
    private final TalonFX m_middleMotor;
    private BooleanSupplier m_intakeButton;
    private BooleanSupplier m_shootButton;
    private BooleanSupplier m_staticShootButton;
    private VelocityVoltage m_shooterVelocityVoltage;

    private FuelManager(){
        super(FuelManagerStates.REST);

        m_intakeMotor = new TalonFX(Constants.FuelManagerConstants.INTAKE_MOTOR_ID);
        m_shootMotorLeader = new TalonFX(Constants.FuelManagerConstants.SHOOT_MOTOR_LEADER_ID);
        m_shootMotorFollower = new TalonFX(Constants.FuelManagerConstants.SHOOT_MOTOR_FOLLOWER_ID);
        m_middleMotor =  new TalonFX(Constants.FuelManagerConstants.MIDDLE_MOTOR_ID);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

        shooterConfig
            .Slot0
                .withKP(0.55)
                .withKI(0)
                .withKD(0.01)
                .withKS(0.2)
                .withKV(0.1);
        m_shooterVelocityVoltage = new VelocityVoltage( 0);
        m_intakeMotor.getConfigurator().apply(shooterConfig);//TODO add individual configs
        m_middleMotor.getConfigurator().apply(shooterConfig);

        m_shootMotorLeader.getConfigurator().apply(shooterConfig);
        m_shootMotorFollower.setControl(
            new Follower(m_shootMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed)
        );
    }

    public static FuelManager getInstance(){
        if(s_FuelManagerInstance == null){
            s_FuelManagerInstance = new FuelManager();
        }
        return s_FuelManagerInstance;
    }

    public void configureBindings(BooleanSupplier intakeButton, BooleanSupplier shootButton, BooleanSupplier staticShootButton){
        m_intakeButton = intakeButton;
        m_shootButton = shootButton;
        m_staticShootButton = staticShootButton;
    }

    @Override
    public void periodic(){
        Logger.recordOutput(getName() + "/Intake Button", m_intakeButton.getAsBoolean());
        Logger.recordOutput(getName() + "/Shoot Button", m_shootButton.getAsBoolean());
        Logger.recordOutput(getName() + "/Current State", getInstance().getState().toString());
        Logger.recordOutput(getName() + "/Intake Motor Speed", m_intakeMotor.getRotorVelocity().getValueAsDouble());
        Logger.recordOutput(getName() + "/Indexer Speed", m_middleMotor.getRotorVelocity().getValueAsDouble());

        //Distance from Hub: x:2.4, y:1.55, Speed:-74.5
        //Distance from Hub: x:0.2, y:-1.82, Speed: 67
        //Distance from Hub: x:-1.83, y:-3.24, Speed: -87.5
        //Distance from Hub: x: .014, y: 1.475, Speed: -65.4
    }

}
