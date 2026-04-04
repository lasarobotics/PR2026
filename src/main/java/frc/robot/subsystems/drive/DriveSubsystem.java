package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.NativeSwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
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
                AngularVelocity rotationRate = Constants.DriveConstants.MAX_ANGULAR_RATE
                        .times(-s_rotateRequest.getAsDouble())
                        .times(s_currentSpeedScalar);
                if(!s_isClimbing){
                s_drivetrain.setControl(
                    s_drive
                        .withVelocityX(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-s_strafeRequest.getAsDouble() * Math.abs(s_strafeRequest.getAsDouble()))
                                .times(s_currentSpeedScalar))
                        .withVelocityY(
                            Constants.DriveConstants.MAX_SPEED
                                .times(-s_driveRequest.getAsDouble() * Math.abs(s_driveRequest.getAsDouble()))
                                .times(s_currentSpeedScalar))
                        .withRotationalRate(
                            rotationRate)
                );

                Logger.recordOutput("controlRotationRate", rotationRate);
                }
                else { }
            }

            @Override
            public SystemState nextState() {
                // TODO
                if (DriverStation.isAutonomous()) {
                    return AUTO;
                }
                if(getInstance().m_autoAimButton.getAsBoolean()){
                    return AUTO_AIM;
                }
                if(s_isReadyToClimb){
                    return CLIMB_ALIGN;
                }
                return DRIVER_CONTROL;
            }
        },
        AUTO_AIM {
            @Override 
            public void execute(){
                if(!s_isClimbing){
                    double pidOutputAngle = getInstance().m_auto_aimrotationPIDController.calculate(s_currentRotation, s_desiredRotation);

                    double pidInput = Constants.DriveConstants.MAX_ANGULAR_RATE.times(pidOutputAngle).in(RadiansPerSecond);
                    pidInput = pidInput > 0 ? Math.min(pidInput, 8.0) : Math.max(pidInput, -8.0);
                    pidInput = Math.abs(s_currentRotation - s_desiredRotation) < Math.PI / 180 ? 0 : pidInput;
                    s_drivetrain.setControl(
                        s_drive
                            .withVelocityX(
                                Constants.DriveConstants.MAX_SPEED
                                    .times(-s_strafeRequest.getAsDouble())
                                    .times(s_currentSpeedScalar))
                            .withVelocityY(
                                Constants.DriveConstants.MAX_SPEED
                                    .times(-s_driveRequest.getAsDouble())
                                    .times(s_currentSpeedScalar))
                            .withRotationalRate(pidInput)
                    );


                    Logger.recordOutput("TranslationDiff", s_translationDiff);
                    Logger.recordOutput("DesiredAngle", s_desiredRotation);
                    Logger.recordOutput("PidInput", pidInput);
                }
            }

            @Override
            public SystemState nextState() {
                if (DriverStation.isAutonomous()) {
                    return AUTO;
                }
                if (getInstance().m_autoAimButton.getAsBoolean() && s_isAutoAimed && Math.abs(s_strafeRequest.getAsDouble()) < 0.1 && Math.abs(s_driveRequest.getAsDouble()) < 0.1)
                {
                    return PARK;
                }
                else if (getInstance().m_autoAimButton.getAsBoolean())
                {
                    return AUTO_AIM;
                }
                return DRIVER_CONTROL;
            }
        },
        CLIMB_ALIGN{
            @Override
            public void execute(){
                    double currentRotation = s_drivetrain.getState().Pose.getRotation().getRadians();
                    double pidOutputAngle = getInstance().m_auto_aimrotationPIDController.calculate(currentRotation, DriverStation.getAlliance().get() == Alliance.Blue ? 0 : Math.PI);
                    double pidInput = Constants.DriveConstants.MAX_ANGULAR_RATE.times(pidOutputAngle).in(RadiansPerSecond);
                    pidInput = pidInput > 0 ? Math.min(pidInput, 8.0) : Math.max(pidInput, -8.0);
                    s_drivetrain.setControl(
                        s_drive
                            .withVelocityX(
                                Constants.DriveConstants.MAX_SPEED
                                    .times(-s_strafeRequest.getAsDouble())
                                    .times(s_currentSpeedScalar))
                            .withVelocityY(
                                Constants.DriveConstants.MAX_SPEED
                                    .times(-s_driveRequest.getAsDouble())
                                    .times(s_currentSpeedScalar))
                            .withRotationalRate(pidInput)
                    );
            }
            @Override
            public SystemState nextState(){
                if (DriverStation.isAutonomous()) {
                    return AUTO;
                }
                if(!s_isReadyToClimb){
                    return DRIVER_CONTROL;
                }
                return CLIMB_ALIGN;
            }
        },
        FULL_CLIMB_ALIGN {
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
                if (DriverStation.isAutonomous()) {
                    return AUTO;
                }
                if(getInstance().m_climbAlignButton.getAsBoolean()){
                    return FULL_CLIMB_ALIGN;
                }
                
                return DRIVER_CONTROL;
            }
        },
        // Brian Park state
        PARK{
            @Override
            public void initialize()
            {
                s_drivetrain.applyRequest(() -> getInstance().m_idle);
                getInstance().m_module0DrivePos = s_drivetrain.getModule(0).getDriveMotor().getPosition().getValueAsDouble();
                getInstance().m_module1DrivePos = s_drivetrain.getModule(1).getDriveMotor().getPosition().getValueAsDouble();
                getInstance().m_module2DrivePos = s_drivetrain.getModule(2).getDriveMotor().getPosition().getValueAsDouble();
                getInstance().m_module3DrivePos = s_drivetrain.getModule(3).getDriveMotor().getPosition().getValueAsDouble();
                
            }

            @Override
            public void execute(){
             
                s_drivetrain.setControl(new ParkRequest());

            }

            @Override
            public SystemState nextState(){
                if (DriverStation.isAutonomous())
                {
                    return AUTO;
                }
                if (getInstance().m_autoAimButton.getAsBoolean())
                {
                    if(getInstance().s_isAutoAimed && Math.abs(s_strafeRequest.getAsDouble()) < 0.1 && Math.abs(s_driveRequest.getAsDouble()) < 0.1)
                    {
                        return PARK;
                    }
                    return AUTO_AIM;
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
    private PIDController m_auto_aimrotationPIDController;
    private PIDController m_translationPIDController;
    private static Translation2d s_hubPos = Constants.HubConstants.BLUE_HUB_POS;
    private static boolean s_isClimbing;
    private static boolean s_isReadyToClimb;
    private static final SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
    private static double s_currentSpeedScalar;
    final SwerveRequest.Idle m_idle = new SwerveRequest.Idle();
    private double m_module0DrivePos;
    private double m_module1DrivePos;
    private double m_module2DrivePos;
    private double m_module3DrivePos;
    private static boolean s_isAutoAimed;
    private static Pose2d s_currentPose2d;
    private static Translation2d s_currentTranslation2d;
    private static double s_currentRotation;
    private static Translation2d s_translationDiff;
    private static double s_desiredRotation;


    public DriveSubsystem() {
        super(DriveStates.AUTO);
        s_drivetrain = TunerConstants.createDrivetrain();
        setPerspective();
        isReadyToClimb(false);
        s_drive =
            new SwerveRequest.FieldCentric()
                .withDeadband(Constants.DriveConstants.MAX_SPEED.times(Constants.DriveConstants.DEADBAND_SCALAR))
                .withRotationalDeadband(Constants.DriveConstants.MAX_ANGULAR_RATE.times(0.1)) // Add a
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
        
        m_rotationPIDController = new PIDController(Constants.DriveConstants.TURN_P,Constants.DriveConstants.TURN_I,Constants.DriveConstants.TURN_D   );
        m_rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_auto_aimrotationPIDController = new PIDController(Constants.DriveConstants.AUTOAIMTURN_P,Constants.DriveConstants.AUTOAIMTURN_I,Constants.DriveConstants.AUTOAIMTURN_D);
        m_auto_aimrotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_translationPIDController = new PIDController(3,7,0);
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
        Logger.recordOutput(getName() + "/DistanceToHub", getDistanceToHub());
        Logger.recordOutput(getName() + "/isReadyToClimb", s_isReadyToClimb);

        if (m_resetPoseButton.getAsBoolean())
        {
            this.resetPose();
        }

        LimelightHelpers.PoseEstimate limelightEstimate = getFilteredPoseEstimate();
        if (limelightEstimate != null && limelightEstimate.tagCount > 0) {
            if(!DriverStation.isDisabled()){
                s_drivetrain.setVisionMeasurementStdDevs(
                        VecBuilder.fill(0.5, 0.5, 9999999));
                s_drivetrain.addVisionMeasurement(limelightEstimate.pose.toPose2d(), Utils.fpgaToCurrentTime(limelightEstimate.timestampSeconds));
                Logger.recordOutput(getName() +"/LimeLight Pose", limelightEstimate.pose);
        }
            else{
                s_drivetrain.setVisionMeasurementStdDevs(
                        VecBuilder.fill(3, 3, 3));
                s_drivetrain.addVisionMeasurement(limelightEstimate.pose.toPose2d(), Utils.fpgaToCurrentTime(limelightEstimate.timestampSeconds));
            }
            Logger.recordOutput(getName() +"/LimeLight Pose", limelightEstimate.pose);
        }
        if(limelightEstimate != null){
            Logger.recordOutput(getName() +"/TagCount", limelightEstimate.tagCount);
        }
    
        s_currentPose2d = s_drivetrain.getState().Pose;
        s_currentTranslation2d = s_currentPose2d.getTranslation();
        s_currentRotation = s_currentPose2d.getRotation().getRadians();
        s_translationDiff = s_hubPos.minus(s_currentTranslation2d);
        // sotm if we ever want
        // Translation2d velocityTranslation = new
        // Translation2d(s_drivetrain.getState().Speeds.vxMetersPerSecond *
        // Constants.DriveConstants.FUEL_AIR_TIME,
        // s_drivetrain.getState().Speeds.vyMetersPerSecond *
        // Constants.DriveConstants.FUEL_AIR_TIME).rotateBy(new
        // Rotation2d(currentRotation));
        // Translation2d futurePose = currentTranslation2d.plus(velocityTranslation);
        // Logger.recordOutput(getInstance().getName() + "/futurePose", new
        // Pose2d(futurePose,new Rotation2d(0)));
        // Translation2d translationDiff = s_hubPos.minus(futurePose);
        s_desiredRotation = Math.atan2(s_translationDiff.getY(), s_translationDiff.getX());

        if (Math.abs(s_currentRotation - s_desiredRotation) < 4 * Math.PI / 180) 
        {
            s_isAutoAimed = true;
        }
        else
        {
            s_isAutoAimed = false;
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
    public double getDistanceToHub() {
        Translation2d differenceFromHub = s_hubPos.minus(s_drivetrain.getState().Pose.getTranslation());
        double distanceToHub = Math.sqrt(Math.pow(differenceFromHub.getX(), 2) + Math.pow(differenceFromHub.getY(), 2));
        return distanceToHub;
    }
     public double getPredictedDistanceToHub() {
        SwerveDriveState currentState = s_drivetrain.getState();
        Translation2d velocityTranslation = new Translation2d(currentState.Speeds.vxMetersPerSecond * Constants.DriveConstants.FUEL_AIR_TIME, currentState.Speeds.vyMetersPerSecond * Constants.DriveConstants.FUEL_AIR_TIME).rotateBy(currentState.Pose.getRotation());
        Translation2d futurePose = currentState.Pose.getTranslation().plus(velocityTranslation);

        Translation2d differenceFromHub = s_hubPos.minus(futurePose);
        double distanceToHub = Math.sqrt(Math.pow(differenceFromHub.getX(), 2) + Math.pow(differenceFromHub.getY(), 2));
        return distanceToHub;
    }
    public boolean awayFromTower(){
        if( Math.abs(Constants.DriveConstants.CENTER_XPOS - DriveSubsystem.getInstance().getPose().getX()) < Constants.DriveConstants.STOW_DISTANCE_REQUIREMENT)
            return true;
        return false;
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
        s_drivetrain.setControl(pointRequest.withModuleDirection(Rotation2d.fromDegrees(0)));
        s_isClimbing = true;
    }

    public Pose2d getPose(){
        return s_drivetrain.getState().Pose;
    }

    public static void isReadyToClimb(boolean ready)
    {
        if (ready)
        {
            s_isReadyToClimb = true;
            s_currentSpeedScalar = Constants.DriveConstants.SLOW_SPEED_SCALAR;
            return;
        }
        s_isReadyToClimb = false;
        s_currentSpeedScalar = Constants.DriveConstants.FAST_SPEED_SCALAR;
    }

    private LimelightHelpers.PoseEstimate getFilteredPoseEstimate() {
        LimelightHelpers.PoseEstimate pose_estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (pose_estimate == null) {
            return null;
        }

        if (Math.abs(s_drivetrain.getState().Speeds.omegaRadiansPerSecond) > 2 * Math.PI) {
            return null;
        }

        if (pose_estimate.tagCount == 0) {
            return null;
        }

        if (Double.isNaN(pose_estimate.pose.getX()) || Double.isNaN(pose_estimate.pose.getY()) || Double.isNaN(pose_estimate.pose.toPose2d().getRotation().getDegrees())) {
            return null;
        }

        // filtering for unreasonable poses
        // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
        // welded perimeter field is slightly larger
        // 16.540988 meters x
        // 8.069326 meters y
        // bump is 16 cm off the ground
        // and anything above 25cm is probably insane airtime & unreliable
        if (
        pose_estimate.pose.getX() < 0         ||
        pose_estimate.pose.getX() > 16.540988 ||
        pose_estimate.pose.getY() < 0         ||
        pose_estimate.pose.getY() > 8.069326  ||
        pose_estimate.pose.getZ() < -0.05     ||
        pose_estimate.pose.getZ() > 0.25
        )
        {
            return null;
        }

        // aggressive filtering for one tag
        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
        if (pose_estimate.tagCount == 1 && pose_estimate.rawFiducials.length == 1) {
            RawFiducial tag = pose_estimate.rawFiducials[0];
            // ignore anything that has too high ambiguity
            if (tag.ambiguity > Constants.DriveConstants.SINGLE_TAG_AMBIGUITY_CUTOFF) {
                return null;
            }
            // we outright reject anything further than a certain distance
            if (tag.distToCamera > Constants.DriveConstants.SINGLE_TAG_DISTANCE_CUTOFF) {
                return null;
            }
        }

        return pose_estimate;
    }

    public static class ParkRequest implements SwerveRequest{
        public StatusCode apply(SwerveControlParameters paraeters, SwerveModule<?, ?, ?>... modulestoApply){
            // front left
            s_drivetrain.getModule(0).apply(new PositionVoltage(getInstance().m_module0DrivePos).withSlot(1), new PositionVoltage(0.25));
            // front right
            s_drivetrain.getModule(1).apply(new PositionVoltage(getInstance().m_module1DrivePos).withSlot(1), new PositionVoltage(0));
            // back left
            s_drivetrain.getModule(2).apply(new PositionVoltage(getInstance().m_module2DrivePos).withSlot(1), new PositionVoltage(0));
            // back right
            s_drivetrain.getModule(3).apply(new PositionVoltage(getInstance().m_module3DrivePos).withSlot(1), new PositionVoltage(0.25));

            return StatusCode.OK;
        }
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
