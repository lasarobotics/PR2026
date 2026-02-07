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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
            }

            @Override 
            public void execute() {
                SwerveDriveState currentState = s_drivetrain.getState();
                Translation2d currentPoseTranslation2d = currentState.Pose.getTranslation();
                double currentAngle = currentState.Pose.getRotation().getRadians();

                Logger.recordOutput("/Auto_Aim/currentAngle", currentAngle);

                Translation2d hubTranslation2d;
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    hubTranslation2d = Constants.DriveConstants.HUB_TRANSLATION_COORDINATES_BLUE;
                } else {
                    hubTranslation2d = Constants.DriveConstants.HUB_TRANSLATION_COORDINATES_RED;
                }
                Translation2d translationDiff = currentPoseTranslation2d.minus(hubTranslation2d);
                double angleGoal = Math.atan2(translationDiff.getY(), translationDiff.getX());

                Logger.recordOutput("/Auto_Aim/angleGoal", angleGoal);

                double pidOutputHeading = s_autoAimController.calculate(currentAngle, angleGoal);

                Logger.recordOutput("/Auto_Aim/pidAngle", pidOutputHeading);
                var directionOfTravel = new Rotation2d();
                var outputVelocity = 0.0;
                double distancePID = 0;
                if (currentPoseTranslation2d.getDistance(hubTranslation2d) > Constants.DriveConstants.AUTO_SHOOT_MAX_DISTANCE) {
                    double currentDistance = currentPoseTranslation2d.getDistance(hubTranslation2d);
                    Translation2d newPosition = hubTranslation2d.minus(currentPoseTranslation2d);
                    directionOfTravel = newPosition.getAngle();
                    outputVelocity = Math.abs(s_autoAimMoveController.calculate(currentDistance, 0.0)) + 0.2;
                    Logger.recordOutput("/Auto_Aim/pidDistance", distancePID);
                }

                s_drivetrain.setControl(s_drive
                .withVelocityX(outputVelocity * directionOfTravel.getCos())
                .withVelocityY(outputVelocity * directionOfTravel.getSin())
                .withRotationalRate(Constants.DriveConstants.MAX_ANGULAR_RATE.times(pidOutputHeading)));

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
                s_climbAutoAlignController.enableContinuousInput(-Math.PI, Math.PI);
            }

            @Override

            public void execute() {
                SwerveDriveState currentState = s_drivetrain.getState();
                Translation2d currentPoseTranslation2d = currentState.Pose.getTranslation();
                double currentAngle = currentState.Pose.getRotation().getRadians();

                Logger.recordOutput("/Climb_Align/currentAngle", currentAngle);

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

                Logger.recordOutput("/Climb_Align/angleGoal", angleGoal);

                double pidOutputHeading = s_climbAutoAlignController.calculate(currentAngle, angleGoal);

                 Logger.recordOutput("/Climb_Align/pidHeading", pidOutputHeading);

                double pidOutputDirection = s_climbAutoMoveController.calculate(translationDiff.getNorm(), 0);

                Logger.recordOutput("/Climb_Align/pidDistance", pidOutputDirection);

                s_drivetrain.setControl(s_drive
                .withVelocityX(pidOutputDirection * translationDiff.getAngle().getCos())
                .withVelocityY(pidOutputDirection * translationDiff.getAngle().getSin())
                .withRotationalRate(Constants.DriveConstants.MAX_ANGULAR_RATE.times(pidOutputHeading)));
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

    private static DoubleSupplier s_driveRequest;
    private static DoubleSupplier s_strafeRequest;
    private static DoubleSupplier s_rotateRequest ;

    private static PIDController s_autoAimController;
    private static PIDController s_autoAimMoveController;
    private static PIDController s_climbAutoAlignController;
    private static PIDController s_climbAutoMoveController;

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
        
        s_autoAimController = new PIDController(
            Constants.DriveConstants.TURN_P,
            Constants.DriveConstants.TURN_I,
            Constants.DriveConstants.TURN_D);
        s_autoAimMoveController = new PIDController(
            Constants.DriveConstants.DRIVE_P,
            Constants.DriveConstants.DRIVE_I,
            Constants.DriveConstants.DRIVE_D);
        s_climbAutoAlignController = new PIDController(
            Constants.DriveConstants.TURN_P,
            Constants.DriveConstants.TURN_I,
            Constants.DriveConstants.TURN_D);
        s_climbAutoMoveController = new PIDController(
            Constants.DriveConstants.DRIVE_P,
            Constants.DriveConstants.DRIVE_I,
            Constants.DriveConstants.DRIVE_D);
    }

    public void configureBindings(
        BooleanSupplier climbAlignButton,
        BooleanSupplier autoAimButton,
        DoubleSupplier strafeRequest,
        DoubleSupplier driveRequest,
        DoubleSupplier rotateRequest) {
        s_climbAlignButton = climbAlignButton;
        s_autoAimButton = autoAimButton;
        s_strafeRequest = strafeRequest;
        s_driveRequest = driveRequest;
        s_rotateRequest = rotateRequest;
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/buttons/climb", s_climbAlignButton.getAsBoolean());
        Logger.recordOutput(getName() + "/buttons/aim", s_autoAimButton.getAsBoolean());
        Logger.recordOutput(getName() + "/Pose", s_drivetrain.getState().Pose);
        // Logger.recordOutput(getName(), null);
    }

    @Override
    public void close() {
        s_drivetrain.close();
    }
}
