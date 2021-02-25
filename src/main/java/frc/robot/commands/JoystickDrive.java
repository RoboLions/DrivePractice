/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class JoystickDrive extends CommandBase {
  /**
   * Creates a new JoystickDrive.
   */
    private final DriveSubsystem driveSubsystem;
    private final static XboxController driverController = RobotContainer.driverController;

  public JoystickDrive(DriveSubsystem drivetrain) {
    driveSubsystem = drivetrain;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("JOYSTICK TIME!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = -driverController.getY(Hand.kLeft);
    double rotate = driverController.getX(Hand.kRight);

    if ((throttle > 0 && throttle < 0.25) || (throttle < 0 && throttle > -0.25)) {
      throttle = 0;
    } else {
      throttle = throttle;
    }

    if ((rotate>0 && rotate<0.25) || (rotate<0 && rotate>-0.25)) {
      rotate = 0;
    } else {
      rotate = rotate;
    }

    if (driverController.getTriggerAxis(Hand.kRight) > 0.25) {
      throttle = Math.signum(throttle)*0.75;
    } else if (driverController.getAButton()) {
      throttle = throttle*1.1;
    } else {
      throttle = throttle*0.8;
    }

    SmartDashboard.putNumber("throttle", throttle);
    SmartDashboard.putNumber("rotate", rotate);
    SmartDashboard.putNumber("left", throttle+rotate);
    SmartDashboard.putNumber("right", throttle-rotate);

    SmartDashboard.putNumber("Front Left", driveSubsystem.leftFrontMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Front Right", driveSubsystem.rightFrontMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Back Left", driveSubsystem.leftBackMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Back Right", driveSubsystem.rightBackMotor.getSelectedSensorVelocity());

    driveSubsystem.drive(throttle, rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
