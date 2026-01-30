package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class DriveSubsystem extends StateMachine implements AutoCloseable {


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
                return AUTO;
            }
        },
        DRIVER_CONTROL {
            @Override
            public SystemState nextState() {
                // TODO
                if(getInstance().m_autoAIMButton.getAsBoolean()){
                    return AUTO_AIM;
                }
                return DRIVER_CONTROL;
            }
        },
        AUTO_AIM {
            @Override
            public SystemState nextState() {
                // TODO
                if(getInstance().m_autoAIMButton.getAsBoolean()){
                    return AUTO_AIM;
                }
                return DRIVER_CONTROL;
            }
        }
    }
    private static DriveSubsystem s_driveSubsystemInstance;
    private static CommandSwerveDrivetrain s_drivetrain;
    private static SwerveRequest.FieldCentric s_drive;
      private static DoubleSupplier s_driveRequest = () -> 0;
    private static DoubleSupplier s_strafeRequest = () -> 0;
    private static DoubleSupplier s_rotateRequest = () -> 0;
    private BooleanSupplier m_autoAIMButton;

    private static boolean s_isAIMed;

    public DriveSubsystem() {
        super(DriveStates.DRIVER_CONTROL);
        s_drivetrain = TunerConstants.createDrivetrain();

        s_drive =
            new SwerveRequest.FieldCentric()
                .withDeadband(Constants.DriveConstants.MAX_SPEED.times(Constants.DriveConstants.DEADBAND_SCALAR))
                .withRotationalDeadband(Constants.DriveConstants.MAX_ANGULAR_RATE.times(0.1)) // Add a
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    }

    @Override
    public void periodic() {

    }

    public static DriveSubsystem getInstance(){
        if(s_driveSubsystemInstance == null){
            s_driveSubsystemInstance = new DriveSubsystem();
        }
        return s_driveSubsystemInstance;
    }
    public void configureBindings(BooleanSupplier autoAIMButton){
        m_autoAIMButton = autoAIMButton;
    }
    @Override
    public void close() {

    }
}
