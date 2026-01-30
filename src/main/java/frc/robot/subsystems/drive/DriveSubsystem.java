package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.opencv.core.Mat;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

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
                if (s_shouldAimAlign) {
                    return AUTO_AIM;
                }
                if (s_shouldClimbAlign) {
                    return CLIMB_ALIGN;
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
        AUTO_AIM {
            @Override
            public void initialize() {
                s_autoAimController.enableContinuousInput(-Math.PI, Math.PI);
                s_autoAimController.setConstraints(Constants.DriveConstants.TURN_CONSTRAINTS);
            }

            @Override
            public void execute() {

                SwerveDriveState currentState = s_drivetrain.getState();
                Translation2d currentPoseTranslation2d = currentState.Pose.getTranslation();
                double currentAngle = currentState.Pose.getRotation().getRadians();
                Translation2d hubTranslation2d = Constants.DriveConstants.HUB_TRANSLATION_COORDINATES;
                Translation2d translationDiff = currentPoseTranslation2d.minus(hubTranslation2d);
                double angleGoal = Math.atan2(translationDiff.getY(), translationDiff.getX());
                double pidOutput = s_autoAimController.calculate(currentAngle, angleGoal);

                // TODO 
                // Create turning logic

            }

            @Override
            public SystemState nextState() {
                // TODO
                return this;
            }
        },
        CLIMB_ALIGN {
            @Override
            public void initialize() {
                // TODO
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
                // TODO
                // CALCULATE FOR THE DISTANCE AS WELL
                // double pidOutputDirection = s_climbAutoMoveController.calculate(translationDiff, 0);
            }

            @Override
            public SystemState nextState() {
                // TODO
                return this;
            }
        },
    }

    private static boolean s_shouldAimAlign = false;
    private static boolean s_shouldClimbAlign = false;
    private static CommandSwerveDrivetrain s_drivetrain;
    private static SwerveRequest.FieldCentric s_drive;
    private static SwerveRequest.RobotCentric s_driveRobotCentric;
    private static FieldCentricWithPose s_autoDrive;

    private static boolean s_isAutoAimed;
    private static boolean s_isClimbAligned;

    private static DoubleSupplier s_driveRequest = () -> 0;
    private static DoubleSupplier s_strafeRequest = () -> 0;
    private static DoubleSupplier s_rotateRequest = () -> 0;

    private static ProfiledPIDController s_autoAimController;
    private static ProfiledPIDController s_climbAutoAlignController;
    private static ProfiledPIDController s_climbAutoMoveController;

    private BooleanSupplier m_climbAlignButton;
    private BooleanSupplier m_autoAimButton;

    public DriveSubsystem() {
        super(DriveStates.DRIVER_CONTROL);
        s_drivetrain = TunerConstants.createDrivetrain();

    }

    public void configureBindings(
        BooleanSupplier climbAlignButton,
        BooleanSupplier autoAimButton) {
            m_climbAlignButton = climbAlignButton;
            m_autoAimButton = autoAimButton;
        }

    @Override
    public void periodic() {

    }

    @Override
    public void close() {

    }
}
