/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import com.ctre.phoenix.sensors.PigeonIMU;


public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   */

  private static final WPI_TalonFX leftMotor = RobotMap.leftDriveMotor;
  private static final WPI_TalonFX rightMotor = RobotMap.rightDriveMotor;

  private static XboxController driveController = Robot.m_robotContainer.driverController;

  public static final int MOTOR_ENCODER_CODES_PER_REV = 2048;
  public static final double DIAMETER_INCHES = 5.0;
  private static final double IN_TO_M = .0254;

  public static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M;
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

  private static final double GEAR_RATIO = 12.75;

  private static final double TICKS_PER_METER = (MOTOR_ENCODER_CODES_PER_REV * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);
  private static final double METERS_PER_TICKS = 1/TICKS_PER_METER;

  private static final int timeoutMs = 10;

  private final PigeonIMU imu = RobotMap.drive_imu;

  public boolean state_flag_motion_profile = true;

  public DriveSubsystem() {
    leftMotor.setNeutralMode(NeutralMode.Coast);
    rightMotor.setNeutralMode(NeutralMode.Coast);

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);

    leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    leftMotor.configVelocityMeasurementWindow(10);
    leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

    rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    rightMotor.configVelocityMeasurementWindow(10);
    rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);
  }

  public double getYaw() {
    double[] ypr = new double[3];
    imu.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public double getPitch() {
    double[] ypr = new double[3];
    imu.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public double getRoll() {
    double[] ypr = new double[3];
    imu.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public void ZeroYaw() {
    imu.setYaw(0, timeoutMs);
    imu.setFusedHeading(0, timeoutMs);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setModePercentVoltage() {
    leftMotor.set(ControlMode.PercentOutput, 0);
    rightMotor.set(ControlMode.PercentOutput, 0);
  }

  public static void drive(double throttle, double rotate) {
    leftMotor.set(throttle + rotate);
    rightMotor.set(throttle - rotate);
  }

  public void stop() {
    drive(0,0);
  }

  public static double getLeftEncoderPosition() {
    return leftMotor.getSelectedSensorPosition();
  }

  public static double getRightEncoderPosition() {
    return rightMotor.getSelectedSensorPosition();
  }

  public double distanceTravelledinTicks() {
    return(getLeftEncoderPosition() + getRightEncoderPosition()) / 2;
  }

  public double getLeftEncoderVelocityMetersPerSecond() {
    double leftVelocityMPS = (leftMotor.getSelectedSensorVelocity()*10);
    leftVelocityMPS = leftVelocityMPS*METERS_PER_TICKS;
    return(leftVelocityMPS);
  }

  public double getRightEncoderVelocityMetersPerSecond() {
    double rightVelocityMPS = (rightMotor.getSelectedSensorVelocity()*10);
    rightVelocityMPS = rightVelocityMPS*METERS_PER_TICKS;
    return rightVelocityMPS;
  }

  public double leftDistanceTravelledinMeters() {
    double left_dist = getLeftEncoderPosition()*METERS_PER_TICKS;
    return left_dist;
  }

  public double rightDistanceTravelledinMeters() {
    double right_dist = getRightEncoderPosition()*METERS_PER_TICKS;
    return right_dist;
  }

  public double distanceTravelledinMeters() {
    double distanceTravelled = (leftDistanceTravelledinMeters() + rightDistanceTravelledinMeters())/ 2;
    return distanceTravelled;
  }

  public void resetEncoders() {
    leftMotor.setSelectedSensorPosition(0);
    rightMotor.setSelectedSensorPosition(0);
  }


}
