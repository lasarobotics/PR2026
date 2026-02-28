// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FuelManager;
import frc.robot.subsystems.drive.DriveSubsystem;

import org.lasarobotics.fsm.SystemState;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final FuelManager FUEL_MANAGER = FuelManager.getInstance();
  public final ClimbSubsystem CLIMB_SUBSYSTEM = ClimbSubsystem.getInstance();
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final DriveSubsystem DRIVE_SUBSYSTEM = DriveSubsystem.getInstance();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController PRIMARY_CONTROLLER =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    FUEL_MANAGER.configureBindings(PRIMARY_CONTROLLER.leftTrigger(),PRIMARY_CONTROLLER.rightBumper(), PRIMARY_CONTROLLER.leftBumper());
    CLIMB_SUBSYSTEM.configureBindings(PRIMARY_CONTROLLER.povUp(),PRIMARY_CONTROLLER.povLeft(),PRIMARY_CONTROLLER.povRight(), PRIMARY_CONTROLLER.povDown(),PRIMARY_CONTROLLER.x(), PRIMARY_CONTROLLER.b());
    DRIVE_SUBSYSTEM.configureBindings(
      PRIMARY_CONTROLLER.a(),
      PRIMARY_CONTROLLER.y(),
      () -> PRIMARY_CONTROLLER.getLeftY(), // drive x
      () -> PRIMARY_CONTROLLER.getLeftX(), // drive y
      () -> PRIMARY_CONTROLLER.getRightX(),// rotate x
      PRIMARY_CONTROLLER.start()); // reset pose
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    return autoChooser.getSelected();
  }

  public Command getPathCommand() 
  {
    try 
    {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Testing Path");
      return AutoBuilder.followPath(path);
    }
    catch (Exception e) {System.out.println(e);}
    return null;
  }

  public Command L1_Climb()
  {
    return Commands.startEnd(() -> CLIMB_SUBSYSTEM.autonStateRequester(true), 
      () -> CLIMB_SUBSYSTEM.autonStateRequester(false),
      CLIMB_SUBSYSTEM
    ).until(() -> !DriverStation.isAutonomous());
  }

  public Command Start_Intake(){
    return new InstantCommand(() -> FUEL_MANAGER.autonStateRequester(FuelManager.FuelManagerStates.INTAKE));
  }

  public Command Fuel_Rest(){
    return new InstantCommand(() -> FUEL_MANAGER.autonStateRequester(null));
  }

  public Command Start_Shoot(){
    return new InstantCommand(() -> FUEL_MANAGER.autonStateRequester(FuelManager.FuelManagerStates.SHOOT));
  }


}
