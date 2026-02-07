package frc.robot.subsystems.drive;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
            public void execute(){
                getInstance().s_drivetrain.setControl(
            getInstance().s_drive
                .withVelocityX(
                    Constants.DriveConstants.MAX_SPEED
                        .times(-Math.pow(getInstance().s_strafeRequest.getAsDouble(), 1))
                        .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                .withVelocityY(
                    Constants.DriveConstants.MAX_SPEED
                        .times(-Math.pow(getInstance().s_driveRequest.getAsDouble(), 1))
                        .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                .withRotationalRate(
                    Constants.DriveConstants.MAX_ANGULAR_RATE
                        .times(-getInstance().s_rotateRequest.getAsDouble())
                        .times(Constants.DriveConstants.FAST_SPEED_SCALAR)));
            }

            @Override
            public SystemState nextState() {
                // TODO
                if (getInstance().s_autoAimButton.getAsBoolean())
                {
                    return AUTO_AIM;
                }
                return DRIVER_CONTROL;
            }
        },
        AUTO_AIM {
            @Override 
            public void execute(){
                Pose2d currentPose2d = getInstance().s_drivetrain.getState().Pose;
                Translation2d currentTranslation2d = currentPose2d.getTranslation();
                double currentRotation = currentPose2d.getRotation().getRadians();
                Translation2d translationDiff = currentTranslation2d.minus(getInstance().s_hubPos);
                double desiredAngle = Math.atan(translationDiff.getY()/translationDiff.getX());
                double pidOutputAngle = getInstance().rotationPIDController.calculate(currentRotation, desiredAngle);
                getInstance().s_drivetrain.setControl(
                getInstance().s_drive
                    .withVelocityX(
                        Constants.DriveConstants.MAX_SPEED
                            .times(-Math.pow(getInstance().s_strafeRequest.getAsDouble(), 1))
                            .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                    .withVelocityY(
                        Constants.DriveConstants.MAX_SPEED
                            .times(-Math.pow(getInstance().s_driveRequest.getAsDouble(), 1))
                            .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                    .withRotationalRate(
                        pidOutputAngle));
                
            }
            @Override
            public SystemState nextState() {
                // TODO
                if(getInstance().s_autoAimButton.getAsBoolean()){
                    return AUTO_AIM;
                }
                return DRIVER_CONTROL;
            }
        }
    }
    private static DriveSubsystem s_driveSubsystemInstance;
    private CommandSwerveDrivetrain s_drivetrain;
    private SwerveRequest.FieldCentric s_drive;
    private DoubleSupplier s_driveRequest;
    private DoubleSupplier s_strafeRequest;
    private DoubleSupplier s_rotateRequest;
    private BooleanSupplier s_autoAimButton;
    private Translation2d s_hubPos;
    private PIDController rotationPIDController = new PIDController(Constants.DriveConstants.TURN_P,Constants.DriveConstants.TURN_I,Constants.DriveConstants.TURN_D);

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
        s_hubPos = getHubPos();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Pose", s_drivetrain.getState().Pose);
        Logger.recordOutput("leftJoystickX", s_strafeRequest);
        Logger.recordOutput("leftJoystickY", s_driveRequest);
        Logger.recordOutput("AutoAimButton", s_autoAimButton);
    }

    public static DriveSubsystem getInstance(){
        if(s_driveSubsystemInstance == null){
            s_driveSubsystemInstance = new DriveSubsystem();
        }
        return s_driveSubsystemInstance;
    }

    public static Translation2d getHubPos(){
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                return Constants.HubConstants.RED_HUB_POS;
            }
            if (ally.get() == Alliance.Blue) {
            return Constants.HubConstants.BLUE_HUB_POS;
            }
        }
        return Constants.HubConstants.BLUE_HUB_POS;
    }
    public void configureBindings(BooleanSupplier autoAimButton, DoubleSupplier strafeRequest,DoubleSupplier driveRequest,DoubleSupplier rotateRequest){
        s_autoAimButton = autoAimButton;
        s_strafeRequest = strafeRequest;
        s_driveRequest = driveRequest;
        s_rotateRequest = rotateRequest;
    }
    @Override
    public void close() {
        // TODO
    }
}