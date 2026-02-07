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
                        .times(-s_rotateRequest.getAsDouble())
                        .times(Constants.DriveConstants.FAST_SPEED_SCALAR);
                s_drivetrain.setControl(
                    s_drive
                        .withVelocityX(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-Math.pow(s_strafeRequest.getAsDouble(), 1))
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                        .withVelocityY(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-Math.pow(s_driveRequest.getAsDouble(), 1))
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                        .withRotationalRate(
                            rotationRate)
                );

                Logger.recordOutput("controlRotationRate", rotationRate);
            }

            @Override
            public SystemState nextState() {
                // TODO
                if(s_autoAIMButton.getAsBoolean()){
                    return AUTO_AIM;
                }
                if(s_climbAlignButton.getAsBoolean()){
                    return CLIMB_ALIGN;
                }
                return DRIVER_CONTROL;
            }
        },
        AUTO_AIM {
            @Override 
            public void execute(){
                Pose2d currentPose2d = s_drivetrain.getState().Pose;
                Translation2d currentTranslation2d = currentPose2d.getTranslation();
                double currentRotation = currentPose2d.getRotation().getRadians();
                Translation2d translationDiff = Constants.HubConstants.HUB_POS.minus(currentTranslation2d);
                double desiredAngle = Math.atan2(translationDiff.getY(),translationDiff.getX()); 
                double pidOutputAngle = rotationPIDController.calculate(currentRotation,desiredAngle);

                s_drivetrain.setControl(
                    s_drive
                        .withVelocityX(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-Math.pow(s_strafeRequest.getAsDouble(), 1))
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                        .withVelocityY(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-Math.pow(s_driveRequest.getAsDouble(), 1))
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
                if(s_autoAIMButton.getAsBoolean()){
                    return AUTO_AIM;
                }
                return DRIVER_CONTROL;
            }
        },
        CLIMB_ALIGN {
            @Override 
            public void execute(){
                Pose2d currentPose2d = s_drivetrain.getState().Pose;
                Translation2d currentTranslation2d = currentPose2d.getTranslation();
                double currentRotation = currentPose2d.getRotation().getRadians();
                Translation2d translationDiff = Constants.HubConstants.HUB_POS.minus(currentTranslation2d);
                double desiredAngle = Math.atan2(translationDiff.getY(),translationDiff.getX()); 
                double pidOutputAngle = rotationPIDController.calculate(currentRotation,desiredAngle);

                s_drivetrain.setControl(
                    s_drive
                        .withVelocityX(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-Math.pow(s_strafeRequest.getAsDouble(), 1))
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                        .withVelocityY(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-Math.pow(s_driveRequest.getAsDouble(), 1))
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
                if(s_climbAlignButton.getAsBoolean()){
                    return CLIMB_ALIGN;
                }
                return DRIVER_CONTROL;
            }
        }
    }
    private static DriveSubsystem s_driveSubsystemInstance;
    private static CommandSwerveDrivetrain s_drivetrain;
    private static SwerveRequest.FieldCentric s_drive;
    private static DoubleSupplier s_driveRequest;
    private static DoubleSupplier s_strafeRequest;
    private static DoubleSupplier s_rotateRequest;
    private static BooleanSupplier s_autoAIMButton;
    private static BooleanSupplier s_climbAlignButton;
    private static PIDController rotationPIDController;
    private static PIDController translationPIDController;

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
        
        rotationPIDController = new PIDController(7, 0.65, 0.2);
        rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
        translationPIDController = new PIDController(getErrorCount(), getMaxRetries(), getErrorCount());

    }

    @Override
    public void periodic() {
        setPerspective();
        Logger.recordOutput(getName() + "/Pose", s_drivetrain.getState().Pose);
        Logger.recordOutput(getName() +"/leftJoystickX", s_strafeRequest);
        Logger.recordOutput(getName() +"/leftJoystickY", s_driveRequest);
        Logger.recordOutput(getName() +"/AutoAIMButton", s_autoAIMButton);
        Logger.recordOutput(getName() +"/CurrentState", s_driveSubsystemInstance.getState().toString());
        Logger.recordOutput(getName() +"/HubPos", Constants.HubConstants.HUB_POS);
        Logger.recordOutput(getName() +"/ClimbAlignButton", s_climbAlignButton);
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
                 s_drivetrain.setOperatorPerspectiveForward(
                CommandSwerveDrivetrain.kRedAlliancePerspectiveRotation);
            }
            if (ally.get() == Alliance.Blue) {
                s_drivetrain.setOperatorPerspectiveForward(
                CommandSwerveDrivetrain.kBlueAlliancePerspectiveRotation);
            }
        }

    }
    public void configureBindings(BooleanSupplier autoAIMButton, BooleanSupplier climbAlignButton, DoubleSupplier strafeRequest,DoubleSupplier driveRequest,DoubleSupplier rotateRequest){
        s_autoAIMButton = autoAIMButton;
        s_climbAlignButton = climbAlignButton;
        s_strafeRequest = strafeRequest;
        s_driveRequest = driveRequest;
        s_rotateRequest = rotateRequest;
    }
    @Override
    public void close() {
        // TODO
    }
}