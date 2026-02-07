package frc.robot.subsystems.drive;

import java.lang.constant.Constable;
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
import edu.wpi.first.units.measure.AngularVelocity;
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
                AngularVelocity rotationRate = Constants.DriveConstants.MAX_ANGULAR_RATE
                        .times(-getInstance().m_rotateRequest.getAsDouble())
                        .times(Constants.DriveConstants.FAST_SPEED_SCALAR);
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
                            rotationRate)
                );

                Logger.recordOutput("controlRotationRate", rotationRate);
            }

            @Override
            public SystemState nextState() {
                // TODO
                if(getInstance().m_autoAIMButton.getAsBoolean()){
                    return AUTO_AIM;
                }
                if(getInstance().m_climbAlignButton.getAsBoolean()){
                    return CLIMB_ALIGN;
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
                double desiredAngle = Math.atan2(translationDiff.getY(),translationDiff.getX()); 
                double pidOutputAngle = getInstance().rotationPIDController.calculate(currentRotation,desiredAngle);

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
                            Constants.DriveConstants.MAX_ANGULAR_RATE
                                .times(pidOutputAngle)
                        )
                );

                Logger.recordOutput("TranslationDiff", translationDiff);
                Logger.recordOutput("DesiredAngle", desiredAngle);
                Logger.recordOutput("PidOutput", Constants.DriveConstants.MAX_ANGULAR_RATE.times(pidOutputAngle));
            }

            @Override
            public SystemState nextState() {
                // TODO
                if(getInstance().m_autoAIMButton.getAsBoolean()){
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
                Translation2d translationDiff = Constants.ClimbConstants.CLIMB_POS.minus(currentTranslation2d);
                double angleCos = translationDiff.getAngle().getCos();
                double angleSin = translationDiff.getAngle().getSin();
                double desiredAngle = 0;
                double pidOutputAngle = getInstance().rotationPIDController.calculate(currentRotation, desiredAngle);
                double pidOutput = getInstance().translationPIDController.calculate(translationDiff.getNorm(), 0);
                getInstance().m_drivetrain.setControl(
                    getInstance().m_drive
                        .withVelocityX(
                            pidOutput * angleCos)
                        .withVelocityY(
                            pidOutput * angleSin)
                        .withRotationalRate(
                                (pidOutputAngle))
                );

                Logger.recordOutput("TranslationDiff", translationDiff);
                Logger.recordOutput("DesiredAngle", desiredAngle);
                Logger.recordOutput("PidOutput", Constants.DriveConstants.MAX_ANGULAR_RATE.times(pidOutputAngle));
            }

            @Override
            public SystemState nextState() {
                // TODO
                if(getInstance().m_climbAlignButton.getAsBoolean()){
                    return CLIMB_ALIGN;
                }
                return DRIVER_CONTROL;
            }
        }
    }
    private static DriveSubsystem m_driveSubsystemInstance;
    private CommandSwerveDrivetrain m_drivetrain;
    private SwerveRequest.FieldCentric m_drive;
    private DoubleSupplier m_driveRequest;
    private DoubleSupplier m_strafeRequest;
    private DoubleSupplier m_rotateRequest;
    private BooleanSupplier m_autoAIMButton;
    private BooleanSupplier m_climbAlignButton;
    private PIDController rotationPIDController;
    private PIDController translationPIDController;

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
        
        rotationPIDController = new PIDController(Constants.DriveConstants.TURN_P,Constants.DriveConstants.TURN_I,Constants.DriveConstants.TURN_D   );
        rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
        translationPIDController = new PIDController(3,7,0);

    }

    @Override
    public void periodic() {
        setPerspective();
        Logger.recordOutput(getName() + "/Pose", m_drivetrain.getState().Pose);
        Logger.recordOutput(getName() +"/leftJoystickX", m_strafeRequest);
        Logger.recordOutput(getName() +"/leftJoystickY", m_driveRequest);
        Logger.recordOutput(getName() +"/AutoAIMButton", m_autoAIMButton);
        Logger.recordOutput(getName() +"/CurrentState", m_driveSubsystemInstance.getState().toString());
        Logger.recordOutput(getName() +"/HubPos", Constants.HubConstants.HUB_POS);
        Logger.recordOutput(getName() +"/ClimbAlignButton", m_climbAlignButton);
    }

    public static DriveSubsystem getInstance(){
        if(m_driveSubsystemInstance == null){
            m_driveSubsystemInstance = new DriveSubsystem();
        }
        return m_driveSubsystemInstance;
    }

    public void setPerspective(){
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                 m_drivetrain.setOperatorPerspectiveForward(
                CommandSwerveDrivetrain.kRedAlliancePerspectiveRotation);
            }
            if (ally.get() == Alliance.Blue) {
                m_drivetrain.setOperatorPerspectiveForward(
                CommandSwerveDrivetrain.kBlueAlliancePerspectiveRotation);
            }
        }

    }
    public void configureBindings(BooleanSupplier autoAIMButton, BooleanSupplier climbAlignButton, DoubleSupplier strafeRequest,DoubleSupplier driveRequest,DoubleSupplier rotateRequest){
        m_autoAIMButton = autoAIMButton;
        m_climbAlignButton = climbAlignButton;
        m_strafeRequest = strafeRequest;
        m_driveRequest = driveRequest;
        m_rotateRequest = rotateRequest;
    }
    @Override
    public void close() {
        // TODO
    }
}