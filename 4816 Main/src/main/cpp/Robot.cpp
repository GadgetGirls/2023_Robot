// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCenter, kAutoNameCenter);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  SpeedThrottlePercentage = .5;
  TurboEnabled = 0;
  inches_per_rotation = 1.91; // derived from measurement

  frc::CameraServer::StartAutomaticCapture();
  leftLeadMotor.RestoreFactoryDefaults();
  rightLeadMotor.RestoreFactoryDefaults();
  leftFollowMotor.RestoreFactoryDefaults();
  rightFollowMotor.RestoreFactoryDefaults();
  rightLeadMotor.SetInverted(true);
  followTelescopeElevatorMotor.SetInverted(true); // This should be the right motor!
  
 /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
     * the Follow() method on the SPARK MAX you want to configure as a follower, and by passing
     * as a parameter the SPARK MAX you want to configure as a leader.
     */
    leftFollowMotor.Follow(leftLeadMotor);
    rightFollowMotor.Follow(rightLeadMotor);
    followTelescopeElevatorMotor.Follow(leadTelescopeElevatorMotor);


  // WF- gyro.GetAngle() returns the yaw axis.  Since we want to measure
  // robot tilt in autonomous mode we want to set the yaw to this axis.
  // We determined through experimentation that the X axis on the gyro
  // measures our tilt.
  rioGyro.SetYawAxis(frc::ADIS16470_IMU::kZ);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  rotate_180_done = 0;
  drive_done = 0;
  initial_tilt_detected = 0;
  pid_rotation.Reset();
  pid_drive.Reset();
  rioGyro.Reset();
  pid_out = 0;
  last_pid_out = 0;
  damping_factor = 1.0;

  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCenter) {
    // Center Auto goes here.  Empty for now.
  } else {
    // Default Auto goes here. Empty for now.
  }
}

void Robot::AutonomousPeriodic() {
  // Use this for debug purposes.  Remove before competition
   frc::SmartDashboard::PutNumber("GyroYaw", robot_angle);

 // Dump initial payload code should go here once we have an arm to work with
  payload_dump_done = 1; // FIX: Remove this once we have arm code written
  
  if (m_autoSelected == kAutoNameCenter) {
    // This program is executed if the robot starts off in the middle position.
    // After dumping the initial payload we want to go into reverse until we detect an angle
    // Once we detect an angle we want to keep going until we are at 0 degrees. 
   
    if (payload_dump_done) {
      x_complementary_angle =  double(rioGyro.GetXComplementaryAngle());
      if (x_complementary_angle < -8) initial_tilt_detected = 1;
      if (!initial_tilt_detected) {robotDrive.ArcadeDrive(-0.5,0);} // Back up until we are on the ramp
      else {
        // If there is a direction change then slowly damp the throttle
        if ((last_pid_out < 0 && pid_out > 0) || ((last_pid_out > 0) || (pid_out < 0))) {
          damping_factor = damping_factor - 0.1;
        } 
        if (x_complementary_angle < 2)  {robotDrive.ArcadeDrive(0,0);}
        else if (x_complementary_angle < -2) {robotDrive.ArcadeDrive(-0.25 * damping_factor,0);} // keep backing up
        else if (x_complementary_angle > 2)  {robotDrive.ArcadeDrive(0.25 * damping_factor,0);}
        last_pid_out = pid_out;
      }
    }
  } else {
    // If robot is not in the middle then drop the payload to score, back up
    // then turn 180 degrees and stop.  At that point we wait until autonomous 
    // period is over.
    
    if (!drive_done && payload_dump_done) {
      leftLeadEncoderValue = leftLeadEncoder.GetPosition();
      rightLeadEncoderValue = rightLeadEncoder.GetPosition();
      pid_out = pid_drive.Calculate(abs(leftLeadEncoderValue), 110);
      if (pid_out > 0.5) pid_out = 0.5; // Clamp the max throttle value
      robotDrive.ArcadeDrive(-pid_out, 0); // We want to go in reverse!
      if (abs(leftLeadEncoderValue) >= 108) drive_done = 1;
    }
    else if (!rotate_180_done && payload_dump_done) {
    pid_out = pid_rotation.Calculate(robot_angle,180); 
    if (pid_out > 0.5) pid_out = 0.5;
    robotDrive.ArcadeDrive(0, pid_out);
    if (abs(robot_angle) >= 178) {rotate_180_done = 1;}
    }
    else {robotDrive.ArcadeDrive(0,0);}
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  frc::SmartDashboard::PutNumber("GyroYaw", robot_angle);

  // Elevator Telescope Control
  if (elevatorExtendLimitSwitch0.Get() && (elevatorController.GetRightY() > 0)) leadTelescopeElevatorMotor.Set(0);
  else if (elevatorRetractLimitSwitch0.Get() && (elevatorController.GetRightY() < 0)) leadTelescopeElevatorMotor.Set(0);
  else if ((elevatorController.GetRightY() < xbox_drift_threshold) && (elevatorController.GetRightY() > -xbox_drift_threshold)) leadTelescopeElevatorMotor.Set(0);
  else leadTelescopeElevatorMotor.Set(elevatorController.GetRightY());
  
  // Elevator Tilt Control
  if (elevatorUprightLimitSwitch0.Get() && (elevatorController.GetLeftY() > 0)) leadTelescopeElevatorMotor.Set(0);
  else if ((elevatorController.GetLeftY() < xbox_drift_threshold) && (elevatorController.GetLeftY() > -drift_tolerance)) tiltElevatorMotor.Set(0);
  else tiltElevatorMotor.Set(elevatorController.GetLeftY());

  // Claw Pneumatics.  Assuming Forward is closed and reverse is open
  if (elevatorController.GetLeftBumperPressed()) {
    leftDoubleSolenoid0.Set(frc::DoubleSolenoid::kReverse);
    rightDoubleSolenoid0.Set(frc::DoubleSolenoid::kReverse);

  } else  if (elevatorController.GetRightBumperPressed()) {
    leftDoubleSolenoid0.Set(frc::DoubleSolenoid::kForward);
    rightDoubleSolenoid0.Set(frc::DoubleSolenoid::kForward);
  }

  // Drive Control
  TurboEnabled = driveStick.GetRawAxis(3);
  if (TurboEnabled < 0) {robotDrive.ArcadeDrive(-driveStick.GetY(), -driveStick.GetX());}
  else { robotDrive.ArcadeDrive(-driveStick.GetY() * SpeedThrottlePercentage, -driveStick.GetX() * SpeedThrottlePercentage);}

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
