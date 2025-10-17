// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive robotDrive;
  private final XboxController controller;

  private final PWMSparkMax leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax rightMotor = new PWMSparkMax(1);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotor.setInverted(true);

    robotDrive = new DifferentialDrive(leftMotor::set, rightMotor::set);
    controller = new XboxController(0);

    SendableRegistry.addChild(robotDrive, leftMotor);
    SendableRegistry.addChild(robotDrive, rightMotor);
  }

  @Override
  public void teleopPeriodic() {
    double leftStickY = -controller.getLeftY();
    double leftStickX = controller.getLeftX();

    if (leftStickY < 0) {
      leftStickX = -leftStickX;
    }

    double leftPower = leftStickY+leftStickX;
    double rightPower = leftStickY-leftStickX;

    if (Math.abs(leftPower) > 1 || Math.abs(rightPower) > 1) {
      double greatest = Math.max(Math.abs(leftPower), Math.abs(rightPower));
      leftPower /= greatest;
      rightPower /= greatest;
    }

    // Convert leftPower from 0-1 to 0.5-1
    // If the power is less than 0.15, then don't move
    leftPower = (leftPower / 2 + 0.5) * (Math.abs(leftPower) > 0.15 ? 1 : 0);
    rightPower = (rightPower / 2 + 0.5) * (Math.abs(rightPower) > 0.15 ? 1 : 0);

    // Slow mode
    if (controller.getRightTriggerAxis() > 0.3) {
      leftPower /= 2;
      rightPower /= 2;
    }

    robotDrive.tankDrive(leftPower, rightPower);

    SmartDashboard.putNumber("Left Power", leftPower);
    SmartDashboard.putNumber("Right Power", rightPower);
  }
}
