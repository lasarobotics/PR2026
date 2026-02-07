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
                getInstance().m_drivetrain.setControl(
                    getInstance().m_drive
                        .withVelocityX(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-Math.pow(getInstance().m_driveRequest.getAsDouble(), 1))
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                        .withVelocityY(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-Math.pow(getInstance().m_strafeRequest.getAsDouble(), 1))
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                        .withRotationalRate(
                            Constants.DriveConstants.MAX_ANGULAR_RATE
                                .times(-getInstance().m_rotateRequest.getAsDouble())
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR)));
            }

            @Override
            public SystemState nextState() {
                // TODO
                if (getInstance().m_autoAimButton.getAsBoolean())
                {
                    return AUTO_AIM;
                }
                return DRIVER_CONTROL;
            }
        },
        AUTO_AIM {
            @Override 
            public void execute(){
                Pose2d currentPose2d = getInstance().m_drivetrain.getState().Pose;
                Translation2d currentTranslation2d = currentPose2d.getTranslation();
                double currentRotation = currentPose2d.getRotation().getRadians();
                Translation2d translationDiff = Constants.HubConstants.HUB_POS.minus(currentTranslation2d);
                double desiredAngle = Math.atan2(translationDiff.getY(), translationDiff.getX());
                double pidOutputAngle = getInstance().rotationPIDController.calculate(currentRotation, desiredAngle);

                getInstance().m_drivetrain.setControl(
                    getInstance().m_drive
                        .withVelocityX(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-Math.pow(getInstance().m_driveRequest.getAsDouble(), 1))
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                        .withVelocityY(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-Math.pow(getInstance().m_strafeRequest.getAsDouble(), 1))
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                        .withRotationalRate(
                            pidOutputAngle)
                );
            }
            @Override
            public SystemState nextState() {
                // TODO
                if(getInstance().m_autoAimButton.getAsBoolean()){
                    return AUTO_AIM;
                }
                return DRIVER_CONTROL;
            }
        },
        CLIMB_ALIGN {
            @Override 
            public void execute(){
                Pose2d currentPose2d = getInstance().m_drivetrain.getState().Pose;
                Translation2d currentTranslation2d = currentPose2d.getTranslation();
                double currentRotation = currentPose2d.getRotation().getRadians();
                Translation2d translationDiff = Constants.HubConstants.HUB_POS.minus(currentTranslation2d);
                double desiredAngle = Math.atan2(translationDiff.getY(), translationDiff.getX());
                double pidOutputAngle = getInstance().rotationPIDController.calculate(currentRotation, desiredAngle);

                getInstance().m_drivetrain.setControl(
                    getInstance().m_drive
                        .withVelocityX(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-Math.pow(getInstance().m_strafeRequest.getAsDouble(), 1))
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                        .withVelocityY(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-Math.pow(getInstance().m_driveRequest.getAsDouble(), 1))
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                        .withRotationalRate(
                            pidOutputAngle)
                );
            }
            @Override
            public SystemState nextState() {
                // TODO
                if(getInstance().m_autoAimButton.getAsBoolean()){
                    return AUTO_AIM;
                }
                return DRIVER_CONTROL;
            }
        }
    }
    private static DriveSubsystem s_driveSubsystemInstance;
    private CommandSwerveDrivetrain m_drivetrain;
    private SwerveRequest.FieldCentric m_drive;
    private DoubleSupplier m_driveRequest;
    private DoubleSupplier m_strafeRequest;
    private DoubleSupplier m_rotateRequest;
    private BooleanSupplier m_autoAimButton;
    private PIDController rotationPIDController;
    private PIDController translationPidController;

    public DriveSubsystem() {
        super(DriveStates.DRIVER_CONTROL);
        m_drivetrain = TunerConstants.createDrivetrain();

        m_drive =
            new SwerveRequest.FieldCentric()
                .withDeadband(Constants.DriveConstants.MAX_SPEED.times(Constants.DriveConstants.DEADBAND_SCALAR))
                .withRotationalDeadband(Constants.DriveConstants.MAX_ANGULAR_RATE.times(0.1)) // Add a
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

        rotationPIDController = new PIDController(Constants.DriveConstants.TURN_P, Constants.DriveConstants.TURN_I, Constants.DriveConstants.TURN_D);
        rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
        translationPidController = new PIDController(Constants.DriveConstants.TRANSLATION_P, Constants.DriveConstants.TRANSLATION_I, Constants.DriveConstants.TRANSLATION_D);


        setPerspective();
    }

    @Override
    public void periodic() {

        setPerspective();

        Logger.recordOutput("Pose", m_drivetrain.getState().Pose);
        Logger.recordOutput("leftJoystickX", m_strafeRequest);
        Logger.recordOutput("leftJoystickY", m_driveRequest);
        Logger.recordOutput("AutoAimButton", m_autoAimButton);
    }

    public static DriveSubsystem getInstance(){
        if(s_driveSubsystemInstance == null){
            s_driveSubsystemInstance = new DriveSubsystem();
        }
        return s_driveSubsystemInstance;
    }

    public static void setPerspective(){
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                getInstance().m_drivetrain.setOperatorPerspectiveForward(CommandSwerveDrivetrain.kRedAlliancePerspectiveRotation);
            }
            if (ally.get() == Alliance.Blue) {
                getInstance().m_drivetrain.setOperatorPerspectiveForward(CommandSwerveDrivetrain.kBlueAlliancePerspectiveRotation);
            }
        }
    }
    public void configureBindings(BooleanSupplier autoAimButton, DoubleSupplier strafeRequest,DoubleSupplier driveRequest,DoubleSupplier rotateRequest){
        m_autoAimButton = autoAimButton;
        m_strafeRequest = strafeRequest;
        m_driveRequest = driveRequest;
        m_rotateRequest = rotateRequest;
    }
    @Override
    public void close() {
        // TODO
    }
}