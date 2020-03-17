/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  WPI_TalonFX rightmotor        = new WPI_TalonFX(Constants.MotorConstants.krightmotor);
  WPI_TalonFX rightmotorSlave   = new WPI_TalonFX(Constants.MotorConstants.krightmotorS);
  WPI_TalonFX leftmotor         = new WPI_TalonFX(Constants.MotorConstants.kleftmotor);
  WPI_TalonFX leftmotorSlave   = new WPI_TalonFX(Constants.MotorConstants.kleftmotorS);

  DifferentialDrive drive = new DifferentialDrive(leftmotor, rightmotor);
  public DriveTrain() {
    setmotor();

  }
  /**
   * set motor, contain: follow, sensor, Voltage
   */
  public void setmotor(){
    rightmotor.configFactoryDefault();
    rightmotorSlave.configFactoryDefault();
    leftmotor.configFactoryDefault();
    leftmotorSlave.configFactoryDefault();

    rightmotor.setSelectedSensorPosition(0);
    leftmotor.setSelectedSensorPosition(0);

    rightmotorSlave.follow(rightmotor);
    leftmotorSlave.follow(leftmotor);

    rightmotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0 ,0);
    leftmotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0 ,0);
    // rightmotorSlave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0 ,0);
    // leftmotorSlave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0 ,0);

    rightmotor.setInverted(Constants.MotorConstants.isRightMotorInvert);
    leftmotor.setInverted(Constants.MotorConstants.isLeftMotorInvert);

    rightmotorSlave.setInverted(InvertType.FollowMaster);
    leftmotorSlave.setInverted(InvertType.FollowMaster);

    rightmotor.setSensorPhase(Constants.MotorConstants.isRightPhaseInvert);
    leftmotor.setSensorPhase(Constants.MotorConstants.isLeftPhaseInvert);
    // rightmotorSlave.setSensorPhase(Constants.MotorConstants.isRightPhaseInvert);
    // leftmotorSlave.setSensorPhase(Constants.MotorConstants.isLeftPhaseInvert);

    rightmotor.configVoltageCompSaturation(10);
    leftmotor.configVoltageCompSaturation(10);
    rightmotor.enableVoltageCompensation(true);
    leftmotor.enableVoltageCompensation(true);
    leftmotorSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,255);
    leftmotorSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,255);
       
    rightmotorSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,255);
    rightmotorSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,255);
  }
  /**
   * get velocity
   * @return left motor velocity(wheel)
   */
  public double getleftVelocity(){
    return leftmotor.getSelectedSensorVelocity(0) * Constants.MotorConstants.distantsPerPulse;
  }
  /**
   * get velocity
   * @return right motor velocity(wheel)
   */
  public double getrightVelocity(){
    return rightmotor.getSelectedSensorVelocity(0) * Constants.MotorConstants.distantsPerPulse;
  }
  /**
   * get position 
   * @return left motor position
   */
  public double getleftPosition(){
    return leftmotor.getSelectedSensorPosition(0) * Constants.MotorConstants.distantsPerPulse;
  }
  /**
   * get position 
   * @return right motor position
   */
  public double getrightPosition(){
    return rightmotor.getSelectedSensorPosition(0) * Constants.MotorConstants.distantsPerPulse;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
