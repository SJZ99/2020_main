/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  WPI_TalonFX rightmotor        = new WPI_TalonFX(Constants.MotorConstants.krightmotor);
  WPI_TalonFX rightmotorSlave   = new WPI_TalonFX(Constants.MotorConstants.krightmotorS);
  WPI_TalonFX leftmotor         = new WPI_TalonFX(Constants.MotorConstants.kleftmotor);
  WPI_TalonFX leftmotorSlave   = new WPI_TalonFX(Constants.MotorConstants.kleftmotorS);

  private double m_quickStopAccumulator = 0,leftout=0,rightout=0;

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

   /**
   * Curvature drive method for differential drive platform.
   *
   * <p>The rotation argument controls the curvature of the robot's path rather than its rate of
   * heading change. This makes the robot more controllable at high speeds. Also handles the
   * robot's quick turn functionality - "quick turn" overrides constant-curvature turning for
   * turn-in-place maneuvers.
   *
   * @param xSpeed      The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation   The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                    positive.
   * @param isQuickTurn If set, overrides constant-curvature turning for
   *                    turn-in-place maneuvers.
   */
  @SuppressWarnings({"ParameterName", "PMD.CyclomaticgitComplexity"})
  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    if(Math.abs(zRotation)<0.05){
      zRotation =0;
    }
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    double angularPower;
    boolean overPower;
    double m_quickStopAlpha =0.1;
    if (isQuickTurn) {
      if (Math.abs(xSpeed) < 0.1) {
        m_quickStopAccumulator = (1 - 0.1) * m_quickStopAccumulator
            + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
      }
      
      overPower = true;
      angularPower = zRotation;
    } else {
      overPower = false;
      angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) {
        m_quickStopAccumulator -= 1;
      } else if (m_quickStopAccumulator < -1) {
        m_quickStopAccumulator += 1;
      } else {
        m_quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = xSpeed + angularPower;
    double rightMotorOutput = xSpeed - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }

    leftout=  leftMotorOutput;
    rightout = rightMotorOutput;
    
  }
  public void curveDrive(double xSpeed, double zRotation, boolean isQuickTurn){
    
    curvatureDrive(xSpeed, zRotation, isQuickTurn);

    rightmotor.set(ControlMode.PercentOutput, rightout);
    leftmotor.set(ControlMode.PercentOutput, leftout);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
