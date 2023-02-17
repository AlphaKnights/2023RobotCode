// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PistonState;
import frc.robot.commands.ChangePistonState;
import frc.robot.commands.NavXZeroCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
  //Robot's Commands
  private final NavXZeroCommand m_zeroCommand = new NavXZeroCommand();
  private final ChangePistonState m_halfClawStateCommand = new ChangePistonState(PistonState.HALF, m_clawSubsystem);
  private final ChangePistonState m_offClawStateCommand = new ChangePistonState(PistonState.OFF, m_clawSubsystem);
  private final ChangePistonState m_openClawStateCommand = new ChangePistonState(PistonState.OPEN, m_clawSubsystem);
  private final ChangePistonState m_closeClawStateCommand = new ChangePistonState(PistonState.CLOSED, m_clawSubsystem);

  // The driver's controller - driver drives the robot
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    JoystickButton m_zeroButton = new JoystickButton(m_driverController, OIConstants.kNavXZeroButton);
  // The operator's controller - operator controls movement of the arm and elevator
  Joystick m_operatorLeftJoystick = new Joystick(OIConstants.kLeftJoystickControllerPort);//Arm
  JoystickButton m_halfClawButton = new JoystickButton(m_driverController, OIConstants.clawHalfShutButton);
  JoystickButton m_fullClawButton = new JoystickButton(m_driverController, OIConstants.clawOpenButton);
  JoystickButton m_closeClawButton = new JoystickButton(m_driverController, OIConstants.clawShutButton);
  JoystickButton m_offClawButton = new JoystickButton(m_driverController, OIConstants.clawOffButton);
  Joystick m_operatorRightJoystick = new Joystick(OIConstants.kRightJoystickControllerPort);//Elevator
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_zeroButton.onTrue(m_zeroCommand);
    m_halfClawButton.onTrue(m_halfClawStateCommand);
    m_fullClawButton.onTrue(m_openClawStateCommand);
    m_closeClawButton.onTrue(m_closeClawStateCommand);
    m_offClawButton.onTrue(m_offClawStateCommand);
    // Configure default commands
    m_elevatorSubsystem.setDefaultCommand(new RunCommand(() -> m_elevatorSubsystem.setPower(m_operatorRightJoystick.getY()*m_operatorRightJoystick.getThrottle())));
    m_armSubsystem.setDefaultCommand(new RunCommand(() -> m_armSubsystem.setPower(m_operatorLeftJoystick.getY()*m_operatorLeftJoystick.getThrottle())));
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}