/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  /**
   * Creates a new Gyro.
   */
  static AHRS ahrs = new AHRS(SPI.Port.kMXP);

  public Gyro() {
    ahrs.reset();
  }
  /**
   * reset the Gyro
   */
  public void reset() {
    ahrs.reset();
  }
  /**
   * for trajectory follow
   * 
   * @return counterclockwise is positive
   */
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-ahrs.getAngle());
  }
  /**
   * get Yaw
   * 
   * @return current Yaw, clockwise is positive
   */
  public float getYaw(){
    return ahrs.getYaw();
  }
  /**
   * get angle
   * 
   * @return Angle clockwise is positive
   */
  public double getAngle(){
    return ahrs.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
