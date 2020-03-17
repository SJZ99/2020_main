/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TrajectoryDepend extends SubsystemBase {
  /**
   * Creates a new TrajectoryDepend.
   */
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TrajectoryFoller.wheelPitch);      // 輪子寬度
  DifferentialDriveOdometry odmetry = new DifferentialDriveOdometry(Robot.gyro.getHeading());

  public TrajectoryDepend() {

  }
  /**
   * set odmetry,let the starting position 
   * of the robot be the starting point of the trajectory
   * 
   * @param pose2d the trajectory origin
   */
  public void setOdmetry(Pose2d pose2d){
    odmetry.resetPosition(pose2d, pose2d.getRotation());
  }
  
 /**
   * encoder velocity to chassis speed
   * 
   * @return current chassis speed
   */
  public DifferentialDriveWheelSpeeds getSpeed() {
    return new DifferentialDriveWheelSpeeds(
      Robot.drive.getleftVelocity(),
      Robot.drive.getrightVelocity()
      );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
