package frc.robot.subsystems;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import edu.wpi.first.wpilibj.DriverStation;

public class DriveSubsystem extends StateMachine implements AutoCloseable {

    public class TO_BE_DECIDED {
        // THIS IS A CLASS TO SET VARIABLE TYPES THAT ARE TO BE DECIDED
        // THIS IS A CLASS TO SET VARIABLE TYPES THAT ARE TO BE DECIDED
        // THIS IS A CLASS TO SET VARIABLE TYPES THAT ARE TO BE DECIDED
        // THIS IS A CLASS TO SET VARIABLE TYPES THAT ARE TO BE DECIDED
        // THIS IS A CLASS TO SET VARIABLE TYPES THAT ARE TO BE DECIDED
    }

    public enum DriveStates implements SystemState {
        NOTHING {
            @Override
            public SystemState nextState() {
                return this;
            }
        },
        AUTO {
            @Override
            public SystemState nextState() {
                if (!DriverStation.isAutonomous()) {
                    return DRIVER_CONTROL;
                }
                if (s_shouldAutoAlign) {
                    return AUTO_ALIGN;
                }
                return this;
            }
        },
        DRIVER_CONTROL {
            @Override
            public SystemState nextState() {
                // TODO
                return this;
            }
        },
        AUTO_ALIGN {
            @Override
            public SystemState nextState() {
                // TODO
                return this;
            }
        }
    }

    private static boolean s_shouldAutoAlign = false;
    private static TO_BE_DECIDED m_drivetrain;
    private static TO_BE_DECIDED s_drive;
    private static TO_BE_DECIDED s_driveRobotCentric;
    private static TO_BE_DECIDED s_autoDrive;
    private static TO_BE_DECIDED m_quest;

    private static boolean s_isAligned;

    public DriveSubsystem() {
        super(DriveStates.DRIVER_CONTROL);

    }

    @Override
    public void periodic() {

    }

    @Override
    public void close() {

    }
}
