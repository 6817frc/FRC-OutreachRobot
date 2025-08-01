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
  private final DifferentialDrive m_robotDrive;
  private final XboxController m_controller;

  private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    m_controller = new XboxController(0);

    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
  }

  @Override
  public void teleopPeriodic() {
    double m_leftStickY = m_controller.getLeftY();
    double m_leftStickX = m_controller.getLeftX();

    double leftPower = -m_leftStickY+m_leftStickX;
    double rightPower = -m_leftStickY-m_leftStickX;

    leftPower /= 1.5;
    rightPower /= 1.5;

    if (m_controller.getRightTriggerAxis() > 0.3) {
      leftPower /= 1.5;
      rightPower /= 1.5;
    }

    m_robotDrive.tankDrive(leftPower, rightPower);

    SmartDashboard.putNumber("Left Power", leftPower);
    SmartDashboard.putNumber("Right Power", rightPower);
  }
}
