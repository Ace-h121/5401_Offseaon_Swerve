// Copyrnght (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  //two motor on the shooter
  CANSparkMax leadMotor;
  CANSparkMax followMotor;

  RelativeEncoder encoder;

  SparkPIDController pidController;


  /** Creates a new Shooter. */
  public Shooter() {
    leadMotor = new CANSparkMax(Constants.ShooterConstants.LEAD_ID, MotorType.kBrushless);
    followMotor = new CANSparkMax(Constants.ShooterConstants.FOLLOWER_ID, MotorType.kBrushless);

    encoder = leadMotor.getEncoder();

    leadMotor.setInverted(true);

    //the true is used to set it inverted to the lead motor
    followMotor.follow(leadMotor, true);
    pidController = leadMotor.getPIDController();

    pidController.setFF(Constants.ShooterConstants.kF);
    pidController.setP(Constants.ShooterConstants.kP);
    pidController.setI(Constants.ShooterConstants.kI);
    pidController.setD(Constants.ShooterConstants.kD);

  }

  public void setVelocity(double velocity){
    pidController.setReference(velocity, ControlType.kVelocity);

  }

  public double getVelocity(){
    return encoder.getVelocity();
  }

  public void stop(){
    leadMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
