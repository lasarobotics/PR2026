package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class DriveSubsystem extends StateMachine{
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
                if(!s_isClimbing){
                AngularVelocity rotationRate = Constants.DriveConstants.MAX_ANGULAR_RATE
                        .times(-s_rotateRequest.getAsDouble())
                        .times(Constants.DriveConstants.FAST_SPEED_SCALAR);
                s_drivetrain.setControl(
                    s_drive
                        .withVelocityX(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-s_strafeRequest.getAsDouble())
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                        .withVelocityY(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-s_driveRequest.getAsDouble())
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                        .withRotationalRate(
                            rotationRate)
                );
                
                Logger.recordOutput("controlRotationRate", rotationRate);
                }
            }

            @Override
            public SystemState nextState() {
                // TODO
                if(getInstance().m_autoAimButton.getAsBoolean()){
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
                if(!s_isClimbing){
                Pose2d currentPose2d = s_drivetrain.getState().Pose;
                Translation2d currentTranslation2d = currentPose2d.getTranslation();
                double currentRotation = currentPose2d.getRotation().getRadians();
                Translation2d translationDiff = s_hubPos.minus(currentTranslation2d);
                double desiredRotation = Math.atan2(translationDiff.getY(),translationDiff.getX()); 
                double pidOutputAngle = getInstance().m_rotationPIDController.calculate(currentRotation, desiredRotation);

                double pidInput = Constants.DriveConstants.MAX_ANGULAR_RATE.times(pidOutputAngle).in(RadiansPerSecond);
                pidInput = pidInput > 0 ? Math.min(pidInput, 8.0) : Math.max(pidInput, -8.0);
                Rotation2d currentAngle = new Rotation2d(currentRotation);
                Rotation2d desiredAngle = new Rotation2d(desiredRotation);
                pidInput = currentAngle.getMeasure().isNear(desiredAngle.getMeasure(), Degrees.of(1.0)) ? 0 : pidInput;
                s_drivetrain.setControl(
                    s_drive
                        .withVelocityX(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-s_strafeRequest.getAsDouble())
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                        .withVelocityY(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-s_driveRequest.getAsDouble())
                                .times(Constants.DriveConstants.FAST_SPEED_SCALAR))
                        .withRotationalRate(pidInput)
                );


                Logger.recordOutput("TranslationDiff", translationDiff);
                Logger.recordOutput("DesiredAngle", desiredAngle);
                Logger.recordOutput("PidInput", pidInput);
                }
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
            public void execute(){// TODO remove if not used, at least remove the keybind
                if(!s_isClimbing){
                Pose2d currentPose2d = s_drivetrain.getState().Pose;
                Translation2d currentTranslation2d = currentPose2d.getTranslation();
                double currentRotation = currentPose2d.getRotation().getRadians();
                Translation2d translationDiff = Constants.ClimbConstants.CLIMB_POS.minus(currentTranslation2d);
                double angleCos = translationDiff.getAngle().getCos()*-1;
                double angleSin = translationDiff.getAngle().getSin()*-1;
                double desiredAngle = 0;
                double pidOutputAngle = Constants.DriveConstants.FAST_SPEED_SCALAR* getInstance().m_rotationPIDController.calculate(currentRotation, desiredAngle);
                double pidOutput = Constants.DriveConstants.FAST_SPEED_SCALAR *0.5* getInstance().m_translationPIDController.calculate(translationDiff.getNorm(), 0);
                // double pidInput = Constants.DriveConstants.MAX_ANGULAR_RATE.times(pidOutputAngle).in(RadiansPerSecond);
                // pidInput = pidInput > 0 ? Math.min(pidInput, 8.0) : Math.max(pidInput, -8.0);
                s_drivetrain.setControl(
                    s_drive
                        .withVelocityX(
                            pidOutput * angleCos)
                        .withVelocityY(
                            pidOutput * angleSin)
                        .withRotationalRate(
                                (pidOutputAngle))
                );
                Logger.recordOutput("ClimbPOS", Constants.ClimbConstants.CLIMB_POS);
                Logger.recordOutput("TranslationDiff", translationDiff);
                Logger.recordOutput("DesiredAngle", desiredAngle);
                Logger.recordOutput("PidOutputAngle", Constants.DriveConstants.MAX_ANGULAR_RATE.times(pidOutputAngle));
                Logger.recordOutput("PidOutput", Constants.DriveConstants.FAST_SPEED_SCALAR * pidOutput);
            }
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
    private static DriveSubsystem s_driveSubsystemInstance;
    private static CommandSwerveDrivetrain s_drivetrain;
    private static SwerveRequest.FieldCentric s_drive;
    private static DoubleSupplier s_driveRequest;
    private static DoubleSupplier s_strafeRequest;
    private static DoubleSupplier s_rotateRequest;
    private BooleanSupplier m_resetPoseButton;
    private BooleanSupplier m_autoAimButton;
    private BooleanSupplier m_climbAlignButton;
    private PIDController m_rotationPIDController;
    private PIDController m_translationPIDController;
    private static Translation2d s_hubPos;
    private static boolean s_isClimbing;
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
        
        m_rotationPIDController = new PIDController(Constants.DriveConstants.TURN_P,Constants.DriveConstants.TURN_I,Constants.DriveConstants.TURN_D   );
        m_rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_translationPIDController = new PIDController(3,7,0);

    }

    public double getDistanceToHub() {
        Translation2d differenceFromHub = s_hubPos.minus(s_drivetrain.getState().Pose.getTranslation());
        double distanceToHub = Math.sqrt(Math.pow(differenceFromHub.getX(), 2) + Math.pow(differenceFromHub.getY(), 2));
        Logger.recordOutput( getName() + "/DistanceToHubAtVarSpeed", distanceToHub);
        return distanceToHub;
    }
    @Override
    public void periodic() {
        setPerspective();
        Logger.recordOutput(getName() + "/Pose", s_drivetrain.getState().Pose);
        Logger.recordOutput(getName() +"/leftJoystickX", s_strafeRequest);
        Logger.recordOutput(getName() +"/leftJoystickY", s_driveRequest);
        Logger.recordOutput(getName() +"/AutoAimButton", m_autoAimButton);
        Logger.recordOutput(getName() +"/CurrentState", s_driveSubsystemInstance.getState().toString());
        Logger.recordOutput(getName() +"/HubPos", s_hubPos);
        Logger.recordOutput(getName() +"/ClimbAlignButton", m_climbAlignButton);
        Logger.recordOutput(getName() +"/ResetPoseButton", m_resetPoseButton);
        Logger.recordOutput(getName() + "/DistanceToHub", s_hubPos.minus(s_drivetrain.getState().Pose.getTranslation()));

        if (m_resetPoseButton.getAsBoolean())
        {
            this.resetPose();
        }

        LimelightHelpers.PoseEstimate limelightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if (limelightEstimate != null && limelightEstimate.tagCount > 0) {
            s_drivetrain.setVisionMeasurementStdDevs(
                    VecBuilder.fill(0.3, 0.3, 9999999));
            s_drivetrain.addVisionMeasurement(limelightEstimate.pose, Utils.fpgaToCurrentTime(limelightEstimate.timestampSeconds));
            Logger.recordOutput(getName() +"/LimeLight Pose", limelightEstimate.pose);
        }
        if(limelightEstimate != null){
            Logger.recordOutput(getName() +"/TagCount", limelightEstimate.tagCount);
        }

        if(DriverStation.isDisabled()){
                LimelightHelpers.PoseEstimate limelightEstimateDisabled = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
            if (limelightEstimateDisabled != null && limelightEstimateDisabled.tagCount > 0) {
                s_drivetrain.setVisionMeasurementStdDevs(
                        VecBuilder.fill(0.1, 0.1, 0.1));
                s_drivetrain.addVisionMeasurement(limelightEstimateDisabled.pose, Utils.fpgaToCurrentTime(limelightEstimate.timestampSeconds));
                Logger.recordOutput(getName() +"/LimeLight Pose", limelightEstimateDisabled.pose);
            }
            if(limelightEstimateDisabled != null){
                Logger.recordOutput(getName() +"/TagCount", limelightEstimateDisabled.tagCount);
            }
        }
    }

    public static DriveSubsystem getInstance(){
        if(s_driveSubsystemInstance == null){
            s_driveSubsystemInstance = new DriveSubsystem();
        }
        return s_driveSubsystemInstance;
    }

    public void resetPose(){
        s_drivetrain.resetPose();
    }

    public void setPerspective(){
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                 s_drivetrain.setOperatorPerspectiveForward(
                CommandSwerveDrivetrain.kRedAlliancePerspectiveRotation);
                s_hubPos = Constants.HubConstants.RED_HUB_POS;
            }
            if (ally.get() == Alliance.Blue) {
                s_drivetrain.setOperatorPerspectiveForward(
                CommandSwerveDrivetrain.kBlueAlliancePerspectiveRotation);
                s_hubPos = Constants.HubConstants.BLUE_HUB_POS;
            }
        }

    }
    public static void postClimbZero(){
        if(s_isClimbing){
            s_drivetrain.setControl(
                    s_drive
                        .withVelocityX(
                            0)
                        .withVelocityY(
                            0)
                        .withRotationalRate(
                                0));
            s_isClimbing = false;
        }
    }
    public static void wheelPushTower(){
        s_drivetrain.setControl(
                    s_drive
                        .withVelocityX(
                            0)
                        .withVelocityY(
                            0)
                        .withRotationalRate(
                                0));
            s_isClimbing = true;
    }
    public Pose2d getPose(){
        return s_drivetrain.getState().Pose;
    }

    public void configureBindings(
        BooleanSupplier autoAimButton, 
        BooleanSupplier climbAlignButton, 
        DoubleSupplier strafeRequest, 
        DoubleSupplier driveRequest, 
        DoubleSupplier rotateRequest, 
        BooleanSupplier resetPoseButton)
    {
        m_autoAimButton = autoAimButton;
        m_climbAlignButton = climbAlignButton;
        m_resetPoseButton = resetPoseButton;
        s_strafeRequest = strafeRequest;
        s_driveRequest = driveRequest;
        s_rotateRequest = rotateRequest;
    }

}
