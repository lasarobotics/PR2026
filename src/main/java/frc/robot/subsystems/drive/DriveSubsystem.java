package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
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
            public void initialize() {
                // TODO
            }

            @Override
            public void execute() {
                // TODO
            }

            @Override
            public SystemState nextState() {
                if (!DriverStation.isAutonomous()) {
                    return DRIVER_CONTROL;
                }
                if (s_autoAimButton.getAsBoolean()) {
                    return AUTO_AIM;
                }
                if (s_climbAlignButton.getAsBoolean()) {
                    return CLIMB_ALIGN;
                }
                return this;
            }
        },
        DRIVER_CONTROL {
            @Override
            public void execute() {
                s_drivetrain.setControl(s_drive
                .withVelocityX(Constants.DriveConstants.MAX_SPEED
                    .times(-s_strafeRequest.getAsDouble())
                    .times(Constants.DriveConstants.s_driveSpeedScaler))
                .withVelocityY(Constants.DriveConstants.MAX_SPEED
                    .times(-s_driveRequest.getAsDouble())
                    .times(Constants.DriveConstants.s_driveSpeedScaler))
                .withRotationalRate(Constants.DriveConstants.MAX_ANGULAR_RATE
                    .times(-s_rotateRequest.getAsDouble())
                    .times(Constants.DriveConstants.s_driveSpeedScaler)));
            }

            @Override
            public SystemState nextState() {
                if (s_autoAimButton.getAsBoolean()) {
                    return AUTO_AIM;
                }
                if (s_climbAlignButton.getAsBoolean()) {
                    return CLIMB_ALIGN;
                }
                return this;
            }
        },
        AUTO_AIM {
            @Override
            public void initialize() {
                s_autoAimController.enableContinuousInput(-Math.PI, Math.PI);
                s_autoAimController.setConstraints(Constants.DriveConstants.TURN_CONSTRAINTS);
                s_autoAimMoveController.setConstraints(Constants.DriveConstants.DRIVE_CONSTRAINTS);
            }

            @Override 
            public void execute() {
                SwerveDriveState currentState = s_drivetrain.getState();
                Translation2d currentPoseTranslation2d = currentState.Pose.getTranslation();
                double currentAngle = currentState.Pose.getRotation().getRadians();
                Translation2d hubTranslation2d;
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    hubTranslation2d = Constants.DriveConstants.HUB_TRANSLATION_COORDINATES_BLUE;
                } else {
                    hubTranslation2d = Constants.DriveConstants.HUB_TRANSLATION_COORDINATES_RED;
                }
                Translation2d translationDiff = currentPoseTranslation2d.minus(hubTranslation2d);
                double angleGoal = Math.atan2(translationDiff.getY(), translationDiff.getX());
                double pidOutputAngle = s_autoAimController.calculate(currentAngle, angleGoal);
                double distancePID = 0;
                if (currentPoseTranslation2d.getDistance(hubTranslation2d) > Constants.DriveConstants.AUTO_SHOOT_MAX_DISTANCE) {
                    double currentDistance = currentPoseTranslation2d.getDistance(hubTranslation2d);
                    double distanceToMove = currentDistance - Constants.DriveConstants.AUTO_SHOOT_MAX_DISTANCE;
                    distancePID = s_autoAimMoveController.calculate(currentDistance, currentDistance-distanceToMove);
                }

                s_drivetrain.setControl(s_drive
                .withVelocityX(distancePID * translationDiff.getAngle().getCos())
                .withVelocityY(distancePID * translationDiff.getAngle().getSin())
                .withRotationalRate(pidOutputAngle));

            }

            @Override
            public SystemState nextState() {
                if (DriverStation.isAutonomous()) {
                    return AUTO;
                }
                if (Math.abs(s_strafeRequest.getAsDouble()) > Constants.DriveConstants.DEADBAND_SCALAR
                        || Math.abs(s_driveRequest.getAsDouble()) > Constants.DriveConstants.DEADBAND_SCALAR
                        || Math.abs(s_rotateRequest.getAsDouble()) > Constants.DriveConstants.DEADBAND_SCALAR) {
                    if (!DriverStation.isAutonomous()) {
                        return DRIVER_CONTROL;
                    }
                }
                if (s_climbAlignButton.getAsBoolean()) {
                    return CLIMB_ALIGN;
                }
                return this;
            }
        },
        CLIMB_ALIGN {
            @Override
            public void initialize() {
                s_climbAutoMoveController.setConstraints(Constants.DriveConstants.DRIVE_CONSTRAINTS);
                s_climbAutoAlignController.enableContinuousInput(-Math.PI, Math.PI);
                s_climbAutoAlignController.setConstraints(Constants.DriveConstants.TURN_CONSTRAINTS);
            }

            @Override
            public void execute() {
                SwerveDriveState currentState = s_drivetrain.getState();
                Translation2d currentPoseTranslation2d = currentState.Pose.getTranslation();
                double currentAngle = currentState.Pose.getRotation().getRadians();
                Translation2d towerTranslation2d;
                int shortestDistanceIndex = 0;
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    for (int i = 1; i < Constants.DriveConstants.BLUE_TOWER_COORDINATES.length; i++) {
                        Translation2d lastTranslation = Constants.DriveConstants.BLUE_TOWER_COORDINATES[i-1];
                        Translation2d currentTranslation = Constants.DriveConstants.BLUE_TOWER_COORDINATES[i];
                        if (currentPoseTranslation2d.getDistance(currentTranslation) < currentPoseTranslation2d.getDistance(lastTranslation)) {
                            shortestDistanceIndex = i;
                        }
                    }
                    towerTranslation2d = Constants.DriveConstants.BLUE_TOWER_COORDINATES[shortestDistanceIndex];
                } else {
                    for (int i = 1; i < Constants.DriveConstants.RED_TOWER_COORDINATES.length; i++) {
                        Translation2d lastTranslation = Constants.DriveConstants.RED_TOWER_COORDINATES[i-1];
                        Translation2d currentTranslation = Constants.DriveConstants.RED_TOWER_COORDINATES[i];
                        if (currentPoseTranslation2d.getDistance(currentTranslation) < currentPoseTranslation2d.getDistance(lastTranslation)) {
                            shortestDistanceIndex = i;
                        }
                    }
                    towerTranslation2d = Constants.DriveConstants.RED_TOWER_COORDINATES[shortestDistanceIndex];
                }
                Translation2d translationDiff = currentPoseTranslation2d.minus(towerTranslation2d);
                double angleGoal = Math.atan2(translationDiff.getY(), translationDiff.getX());
                double pidOutputHeading = s_climbAutoAlignController.calculate(currentAngle, angleGoal);
                double pidDistance = currentPoseTranslation2d.getDistance(towerTranslation2d);
                double pidOutputDirection = s_climbAutoMoveController.calculate(pidDistance, 0);
                s_drivetrain.setControl(s_drive
                .withVelocityX(pidOutputDirection * translationDiff.getAngle().getCos())
                .withVelocityY(pidOutputDirection * translationDiff.getAngle().getSin())
                .withRotationalRate(pidOutputHeading));
            }

            @Override
            public SystemState nextState() {
                if (DriverStation.isAutonomous()) {
                    return AUTO;
                }
                if (Math.abs(s_strafeRequest.getAsDouble()) > Constants.DriveConstants.DEADBAND_SCALAR
                        || Math.abs(s_driveRequest.getAsDouble()) > Constants.DriveConstants.DEADBAND_SCALAR
                        || Math.abs(s_rotateRequest.getAsDouble()) > Constants.DriveConstants.DEADBAND_SCALAR) {
                    if (!DriverStation.isAutonomous()) {
                        return DRIVER_CONTROL;
                    }
                }
                if (s_autoAimButton.getAsBoolean()) {
                    return AUTO_AIM;
                }
                return this;
            }
        },
    }

    private static CommandSwerveDrivetrain s_drivetrain;
    private static SwerveRequest.FieldCentric s_drive;
    private static SwerveRequest.RobotCentric s_driveRobotCentric;

    private static DoubleSupplier s_driveRequest = () -> 0;
    private static DoubleSupplier s_strafeRequest = () -> 0;
    private static DoubleSupplier s_rotateRequest = () -> 0;

    private static ProfiledPIDController s_autoAimController;
    private static ProfiledPIDController s_autoAimMoveController;
    private static ProfiledPIDController s_climbAutoAlignController;
    private static ProfiledPIDController s_climbAutoMoveController;

    private static BooleanSupplier s_climbAlignButton;
    private static BooleanSupplier s_autoAimButton;

    public DriveSubsystem() {
        super(DriveStates.DRIVER_CONTROL);
        s_drivetrain = TunerConstants.createDrivetrain();

        s_drive = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.DriveConstants.MAX_SPEED.times(Constants.DriveConstants.DEADBAND_SCALAR))
            .withRotationalDeadband(Constants.DriveConstants.MAX_ANGULAR_RATE.times(0.1))
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            // TODO specify OperatorPerspective in SwerveDrivetrain obj
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
    }

    public void bindControls(
        DoubleSupplier driveRequest, 
        DoubleSupplier strafeRequest, 
        DoubleSupplier rotateRequest,
        BooleanSupplier climbAlignButton,
        BooleanSupplier autoAimButton) {
        
        s_driveRequest = driveRequest;
        s_strafeRequest = strafeRequest;
        s_rotateRequest = rotateRequest;
        s_climbAlignButton = climbAlignButton;
        s_autoAimButton = autoAimButton;
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/buttons/climb", s_climbAlignButton.getAsBoolean());
        Logger.recordOutput(getName() + "/buttons/aim", s_autoAimButton.getAsBoolean());
    }

    @Override
    public void close() {
        s_drivetrain.close();
    }
}
