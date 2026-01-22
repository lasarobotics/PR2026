package frc.robot.subsystems;


import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

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

            }
            @Override
            public SystemState nextState() {
                return this;
            }
        },

        INTAKE {
            @Override
            public void initialize() {

            }
            @Override
            public void execute() {

            }
            @Override
            public SystemState nextState() {
                return this;
            }
        },

        SHOOT {
            @Override
            public void initialize() {

            }
            @Override
            public SystemState nextState() {
                return this;
            }
        }


    }
    
    private final TalonFX m_intakeMotor;
    private final TalonFX m_shootMotor;
    private final TalonFX m_middleMotor;

    private FuelManager(){
        super(FuelManagerStates.REST);

        m_intakeMotor = new TalonFX(Constants.FuelManagerConstants.intakeMotorId);
        m_shootMotor =  new TalonFX(Constants.FuelManagerConstants.shootMotorId);
        m_middleMotor =  new TalonFX(Constants.FuelManagerConstants.middleMotorId);
        

    }

    @Override
    public void close(){
        // TODO
    }
}
