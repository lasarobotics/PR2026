package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.BooleanSupplier;
import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

public class FuelManager extends StateMachine {

  public enum FuelManagerStates implements SystemState {
    NOTHING {
      @Override
      public SystemState nextState() {
        return this;
      }
    },
    REST {
      @Override
      public void initialize() {
        getInstance().m_shootMotorLeader.set(0);
        getInstance().m_middleMotor.set(0);
        getInstance().m_intakeMotor.set(0);
        getInstance().m_agitationMotor.set(0);
      }

      @Override
      public void execute() {
        getInstance()
            .setHopperPoint(
                m_whichJiggle,
                Constants.FuelManagerConstants.HOPPER_DEPLOY_POINT,
                Constants.FuelManagerConstants.HOPPER_DEPLOY_POINT);
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isAutonomous() && s_autonStateRequest != null) {
          return s_autonStateRequest;
        }
        if (getInstance().m_hopperStowButton.getAsBoolean() && getInstance().m_hopperStowDebounceCounter == 0)
          getInstance().m_shouldStow = !getInstance().m_shouldStow;
        if (getInstance().m_unclogButton.getAsBoolean()
            || getInstance().m_shooterBeamBreak.getIsDetected().getValue()) {
          return UNCLOG;
        }
        if (getInstance().m_intakeButton.getAsBoolean()) {
          return INTAKE;
        }
        if (getInstance().m_shootButton.getAsBoolean()) {
          return SHOOT;
        }
        if (getInstance().m_staticShootButton.getAsBoolean()) {
          return STATIC_SHOOT;
        }
        return REST;
      }
    },
    INTAKE {
      @Override
      public void initialize() {
        getInstance()
            .m_middleMotor
            .setControl(
                getInstance()
                    .m_motorVelocityVoltage
                    .withVelocity(
                        Constants.FuelManagerConstants
                            .MIDDLE_MOTOR_INTAKE_SPEED)); // TODO add vraible speed
        getInstance()
            .m_intakeMotor
            .setControl(
                getInstance()
                    .m_motorVelocityVoltage
                    .withVelocity(Constants.FuelManagerConstants.INTAKE_MOTOR_SPEED));
        getInstance()
            .m_agitationMotor
            .setControl(
                getInstance()
                    .m_motorVelocityVoltage
                    .withVelocity(Constants.FuelManagerConstants.AGITATION_MOTOR_SPEED));
      }

      @Override
      public void execute() {
        getInstance()
            .setHopperPoint(
                m_whichJiggle,
                Constants.FuelManagerConstants.HOPPER_DEPLOY_POINT,
                Constants.FuelManagerConstants.HOPPER_DEPLOY_POINT);  
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isAutonomous() && s_autonStateRequest != null) {
          return s_autonStateRequest;
        }
        if (getInstance().m_hopperStowButton.getAsBoolean() && getInstance().m_hopperStowDebounceCounter == 0)
          getInstance().m_shouldStow = !getInstance().m_shouldStow;
        if (getInstance().m_unclogButton.getAsBoolean()
            || getInstance().m_shooterBeamBreak.getIsDetected().getValue()) {
          return UNCLOG;
        }
        if (getInstance().m_intakeButton.getAsBoolean()) {
          return INTAKE;
        }
        return REST;
      }
    },
    UNCLOG {
      @Override
      public void initialize() {
        getInstance()
            .m_middleMotor
            .setControl(
                getInstance()
                    .m_motorVelocityVoltage
                    .withVelocity(
                        Constants.FuelManagerConstants
                            .MIDDLE_MOTOR_INTAKE_SPEED)); // TODO add vraible speed
        getInstance()
            .m_intakeMotor
            .setControl(
                getInstance()
                    .m_motorVelocityVoltage
                    .withVelocity(Constants.FuelManagerConstants.INTAKE_UNCLOG_SPEED));
        getInstance().m_shootMotorLeader.setControl(new VoltageOut(1));
        getInstance()
            .m_agitationMotor
            .setControl(
                getInstance()
                    .m_motorVelocityVoltage
                    .withVelocity(Constants.FuelManagerConstants.AGITATION_MOTOR_SPEED));
      }

      @Override
      public void execute() {
        getInstance()
            .setHopperPoint(
                m_whichJiggle,
                Constants.FuelManagerConstants.HOPPER_DEPLOY_POINT,
                Constants.FuelManagerConstants.HOPPER_DEPLOY_POINT);
        if (getInstance().m_hopperIntervalCounter++
            == Constants.FuelManagerConstants.HOPPER_JIGGLE_INTERVAL_LENGTH) {
          getInstance().m_hopperIntervalCounter = 0;
          getInstance()
              .setHopperPoint(
                  m_whichJiggle,
                  Constants.FuelManagerConstants.HOPPER_JIGGLE_POINT,
                  Constants.FuelManagerConstants.HOPPER_DEPLOY_POINT);
        }
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isAutonomous() && s_autonStateRequest != null) {
          return s_autonStateRequest;
        }
        if (getInstance().m_hopperStowButton.getAsBoolean() && getInstance().m_hopperStowDebounceCounter == 0)
          getInstance().m_shouldStow = !getInstance().m_shouldStow;
        if (getInstance().m_unclogButton.getAsBoolean()
            || getInstance().m_shooterBeamBreak.getIsDetected().getValue()) {
          return UNCLOG;
        }
        return REST;
      }
    },
    SHOOT {
      @Override
      public void initialize() {
        getInstance().m_shootSpeed =
            getInstance()
                .getSpeed(
                    (s_DriveSubsystemInstance
                        .getDistanceToHub())); // use predictedDistanceToHub for sotm
        getInstance()
            .m_shootMotorLeader
            .setControl(
                getInstance().m_shooterVelocityDutyCycle.withVelocity(getInstance().m_shootSpeed));
      }

      @Override
      public void execute() {
        getInstance().m_shootSpeed =
            getInstance()
                .getSpeed(
                    (s_DriveSubsystemInstance
                        .getDistanceToHub())); // use predictedDistanceToHub for sotm
        getInstance()
            .m_shootMotorLeader
            .setControl(
                getInstance().m_shooterVelocityDutyCycle.withVelocity(getInstance().m_shootSpeed));
        if (Math.abs(
                getInstance().m_shootMotorLeader.getRotorVelocity().getValueAsDouble()
                    - getInstance().m_shootSpeed)
            <= Math.abs(getInstance().m_shootSpeed)
                * Constants.FuelManagerConstants.SHOOTER_WITHIN_RANGE_COEFFICIENT) {
          getInstance()
              .m_intakeMotor
              .setControl(
                  getInstance()
                      .m_motorVelocityVoltage
                      .withVelocity(Constants.FuelManagerConstants.INTAKE_MOTOR_SPEED));
          getInstance()
              .m_middleMotor
              .setControl(
                  getInstance()
                      .m_motorVelocityVoltage
                      .withVelocity(
                          Constants.FuelManagerConstants
                              .MIDDLE_MOTOR_SHOOT_SPEED)); // TODO add vraible speed
        }
        if (getInstance().m_thumpIntervalCounter
            <= Constants.FuelManagerConstants.THUMPER_INTERVAL_LENGTH) {
          getInstance()
              .m_agitationMotor
              .setControl(
                  getInstance()
                      .m_motorVelocityVoltage
                      .withVelocity(Constants.FuelManagerConstants.AGITATION_MOTOR_SPEED));
        } else {
          if (getInstance().m_thumpIntervalCounter
              >= Constants.FuelManagerConstants.THUMPER_TOTAL_LENGTH) {
            getInstance().m_thumpIntervalCounter = 0;
          }
          getInstance()
              .m_agitationMotor
              .setControl(getInstance().m_motorVelocityVoltage.withVelocity(0));
        }
        getInstance().m_thumpIntervalCounter++;

        if (getInstance().m_hopperIntervalCounter++
            == Constants.FuelManagerConstants.HOPPER_JIGGLE_INTERVAL_LENGTH) {
          getInstance().m_hopperIntervalCounter = 0;
          getInstance()
              .setHopperPoint(
                  m_whichJiggle,
                  Constants.FuelManagerConstants.HOPPER_DEPLOY_POINT,
                  Constants.FuelManagerConstants.HOPPER_JIGGLE_POINT);
        }
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isAutonomous() && s_autonStateRequest != null) {
          return s_autonStateRequest;
        }
        if (getInstance().m_hopperStowButton.getAsBoolean() && getInstance().m_hopperStowDebounceCounter == 0)
          getInstance().m_shouldStow = !getInstance().m_shouldStow;
        if (getInstance().m_shootButton.getAsBoolean()) {
          return SHOOT;
        }
        return REST;
      }
    },
    STATIC_SHOOT {
      @Override
      public void initialize() {
        getInstance()
            .m_shootMotorLeader
            .setControl(
                getInstance()
                    .m_shooterVelocityDutyCycle
                    .withVelocity(Constants.FuelManagerConstants.SHOOT_MOTOR_SPEED));
      }

      @Override
      public void execute() {
        if (Math.abs(
                getInstance().m_shootMotorLeader.getRotorVelocity().getValueAsDouble()
                    - Constants.FuelManagerConstants.SHOOT_MOTOR_SPEED)
            <= Math.abs(Constants.FuelManagerConstants.SHOOT_MOTOR_SPEED)
                * Constants.FuelManagerConstants.SHOOTER_WITHIN_RANGE_COEFFICIENT) {
          getInstance()
              .m_intakeMotor
              .setControl(
                  getInstance()
                      .m_motorVelocityVoltage
                      .withVelocity(Constants.FuelManagerConstants.INTAKE_MOTOR_SPEED));
          getInstance()
              .m_middleMotor
              .setControl(
                  getInstance()
                      .m_motorVelocityVoltage
                      .withVelocity(
                          Constants.FuelManagerConstants
                              .MIDDLE_MOTOR_SHOOT_SPEED)); // TODO add vraible speed
        }
        if (getInstance().m_thumpIntervalCounter
            <= Constants.FuelManagerConstants.THUMPER_INTERVAL_LENGTH) {
          getInstance()
              .m_agitationMotor
              .setControl(
                  getInstance()
                      .m_motorVelocityVoltage
                      .withVelocity(Constants.FuelManagerConstants.AGITATION_MOTOR_SPEED));
        } else {
          if (getInstance().m_thumpIntervalCounter
              >= Constants.FuelManagerConstants.THUMPER_TOTAL_LENGTH) {
            getInstance().m_thumpIntervalCounter = 0;
          }
          getInstance()
              .m_agitationMotor
              .setControl(getInstance().m_motorVelocityVoltage.withVelocity(0));
        }
        getInstance().m_thumpIntervalCounter++;

        if (getInstance().m_hopperIntervalCounter++
            == Constants.FuelManagerConstants.HOPPER_JIGGLE_INTERVAL_LENGTH) {
          getInstance().m_hopperIntervalCounter = 0;
          getInstance()
              .setHopperPoint(
                  m_whichJiggle,
                  Constants.FuelManagerConstants.HOPPER_DEPLOY_POINT,
                  Constants.FuelManagerConstants.HOPPER_JIGGLE_POINT);
        }
      }

      @Override
      public SystemState nextState() {
        if (DriverStation.isAutonomous() && s_autonStateRequest != null) {
          return s_autonStateRequest;
        }
        if (getInstance().m_hopperStowButton.getAsBoolean() && getInstance().m_hopperStowDebounceCounter == 0 && getInstance().m_hopperStowDebounceCounter == 0)
          getInstance().m_shouldStow = !getInstance().m_shouldStow;
        if (getInstance().m_staticShootButton.getAsBoolean()) {
          return STATIC_SHOOT;
        }
        return REST;
      }
    }
  }

  private static FuelManager s_FuelManagerInstance;
  private static DriveSubsystem s_DriveSubsystemInstance;
  private final TalonFX m_intakeMotor;
  private final TalonFX m_shootMotorLeader;
  private final TalonFX m_shootMotorFollower;
  private final TalonFX m_middleMotor;
  private final TalonFX m_agitationMotor;
  private final TalonFX m_hopperMotor;
  private final CANrange m_shooterBeamBreak;
  private BooleanSupplier m_intakeButton;
  private BooleanSupplier m_shootButton;
  private BooleanSupplier m_staticShootButton;
  private BooleanSupplier m_unclogButton;
  private BooleanSupplier m_hopperStowButton;
  private VelocityDutyCycle m_shooterVelocityDutyCycle;
  private VelocityVoltage m_motorVelocityVoltage;
  private double m_shootSpeed;
  private static SystemState s_autonStateRequest;
  private int m_thumpIntervalCounter;
  private static boolean m_whichJiggle;
  private int m_hopperIntervalCounter;
  private int m_hopperStowDebounceCounter;
  private MotionMagicVoltage m_hopperPositionVoltage;
  private boolean m_shouldStow;

  private FuelManager() {
    super(FuelManagerStates.REST);
    m_thumpIntervalCounter = 0;
    m_hopperIntervalCounter = 0;
    m_shouldStow = false;
    s_autonStateRequest = null;
    m_hopperStowDebounceCounter = 0;
    s_DriveSubsystemInstance = DriveSubsystem.getInstance();
    m_intakeMotor = new TalonFX(Constants.FuelManagerConstants.INTAKE_MOTOR_ID);
    m_shootMotorLeader = new TalonFX(Constants.FuelManagerConstants.SHOOT_MOTOR_LEADER_ID);
    m_shootMotorFollower = new TalonFX(Constants.FuelManagerConstants.SHOOT_MOTOR_FOLLOWER_ID);
    m_middleMotor = new TalonFX(Constants.FuelManagerConstants.MIDDLE_MOTOR_ID);
    m_agitationMotor = new TalonFX(Constants.FuelManagerConstants.AGITATION_MOTOR_ID);
    m_hopperMotor = new TalonFX(Constants.FuelManagerConstants.HOPPER_MOTOR_ID);
    m_shooterBeamBreak = new CANrange(Constants.FuelManagerConstants.BEAM_BREAK_ID);
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    m_hopperMotor.setPosition(0);

    shooterConfig.Slot0.withKP(999999999);
    shooterConfig.MotorOutput.withPeakForwardDutyCycle(0.0).withPeakReverseDutyCycle(-1.0);
    m_shooterVelocityDutyCycle = new VelocityDutyCycle(0);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0.withKP(0.55).withKI(0).withKD(0.01).withKS(0.2).withKV(0.1);
    m_motorVelocityVoltage = new VelocityVoltage(0);

    m_intakeMotor.getConfigurator().apply(motorConfig); // TODO add individual configs
    m_middleMotor.getConfigurator().apply(motorConfig);

    m_shootMotorFollower.getConfigurator().apply(shooterConfig);
    m_shootMotorLeader.getConfigurator().apply(shooterConfig);

    m_shootMotorFollower.setControl(
        new Follower(m_shootMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    TalonFXConfiguration agitatorConfig = new TalonFXConfiguration();
    agitatorConfig.Slot0.withKP(0.6).withKV(0.1);
    m_agitationMotor.getConfigurator().apply(agitatorConfig);

    TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
    hopperConfig.Slot0.withKP(10).withKD(.1);
    hopperConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    hopperConfig
        .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(50)
        .withSupplyCurrentLimit(30)
        .withSupplyCurrentLowerLimit(40);
    m_hopperMotor.getConfigurator().apply(hopperConfig);
    m_whichJiggle = true;
    m_hopperPositionVoltage = new MotionMagicVoltage(Radians.zero());
  }

  public static FuelManager getInstance() {
    if (s_FuelManagerInstance == null) {
      s_FuelManagerInstance = new FuelManager();
    }
    return s_FuelManagerInstance;
  }

  public double getSpeed(double totalDistance) {
    double aValue = 1.85;
    double bValue = -18.2;
    double cValue = -34.1;
    return 1.035 * ((aValue * Math.pow(totalDistance, 2)) + (bValue * totalDistance) + cValue);
  }

  public static void autonStateRequester(SystemState request) {
    s_autonStateRequest = request;
  }

  public void configureBindings(
      BooleanSupplier intakeButton,
      BooleanSupplier shootButton,
      BooleanSupplier staticShootButton,
      BooleanSupplier unclogButton,
      BooleanSupplier hopperStowButton) {
    m_intakeButton = intakeButton;
    m_shootButton = shootButton;
    m_staticShootButton = staticShootButton;
    m_unclogButton = unclogButton;
    m_hopperStowButton = hopperStowButton;
  }

  public void setHopperPoint(
      boolean whichJiggle, final PositionVoltage truePoint, final PositionVoltage falsePoint) {
    ++getInstance().m_hopperStowDebounceCounter;
    if (getInstance().m_hopperStowDebounceCounter >= Constants.FuelManagerConstants.HOPPER_STOW_DEBOUNCE) {
      getInstance().m_hopperStowDebounceCounter = 0;
    }
    if (ClimbSubsystem.getInstance().getIsExtended() || ClimbSubsystem.getInstance().getIsClimbing() || getInstance().m_shouldStow) {
      getInstance().m_hopperMotor.setControl(Constants.FuelManagerConstants.HOPPER_STOW_POINT);
      return;
    }

    if (whichJiggle) {
      getInstance().m_hopperMotor.setControl(truePoint);
    } else {
      getInstance().m_hopperMotor.setControl(falsePoint);
    }
    m_whichJiggle = !m_whichJiggle;
  }

  @Override
  public void periodic() {
    Logger.recordOutput(getName() + "/Intake Button", m_intakeButton.getAsBoolean());
    Logger.recordOutput(getName() + "/Shoot Button", m_shootButton.getAsBoolean());
    Logger.recordOutput(getName() + "/Current State", getInstance().getState().toString());
    Logger.recordOutput(
        getName() + "/Intake Motor Speed", m_intakeMotor.getRotorVelocity().getValueAsDouble());
    Logger.recordOutput(
        getName() + "/Indexer Speed", m_middleMotor.getRotorVelocity().getValueAsDouble());
    Logger.recordOutput(
        getName() + "/SHOOTER Speed", m_shootMotorLeader.getRotorVelocity().getValueAsDouble());
    Logger.recordOutput(getName() + "/Desired Shooter Speed", getInstance().m_shootSpeed);
    Logger.recordOutput(
        getName() + "/BeamBreak", getInstance().m_shooterBeamBreak.getIsDetected().getValue());
    Logger.recordOutput(
        getName() + "/HopperPosition",
        getInstance().m_hopperMotor.getPosition().getValueAsDouble());
    // Distance from Hub: x:2.4, y:1.55, Speed:-74.5
    // Distance from Hub: x:0.2, y:-1.82, Speed: 67
    // Distance from Hub: x:-1.83, y:-3.24, Speed: -87.5
    // Distance from Hub: x: .014, y: 1.475, Speed: -65.4
  }
}
