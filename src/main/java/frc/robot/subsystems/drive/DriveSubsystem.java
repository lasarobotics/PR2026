package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

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
    private static CommandSwerveDrivetrain s_drivetrain;
    private static SwerveRequest.FieldCentric s_drive;
      private static DoubleSupplier s_driveRequest = () -> 0;
    private static DoubleSupplier s_strafeRequest = () -> 0;
    private static DoubleSupplier s_rotateRequest = () -> 0;
 

    private static boolean s_isAligned;

    public DriveSubsystem() {
        super(DriveStates.DRIVER_CONTROL);
        s_drivetrain = TunerConstants.createDrivetrain();

        s_drive =
            new SwerveRequest.FieldCentric()
                .withDeadband(Constants.Drive.MAX_SPEED.times(DriveSubsystem.DEADBAND_SCALAR))
                .withRotationalDeadband(Constants.Drive.MAX_ANGULAR_RATE.times(0.1)) // Add a
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void close() {

    }
}
