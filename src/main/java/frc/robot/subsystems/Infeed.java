// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//WPI Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//  Shuffle Board Imports
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;

//Rev Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

//File Imports
import frc.robot.Constants;

//Unused Imports that might be used in future
//import edu.wpi.first.math.controller.PIDController;

public class Infeed extends SubsystemBase {
  //Creates CANSparkMax's
  private CANSparkMax pivotMotor;
  private CANSparkMax intakeMotor1;

  //Creates Encoders
  private RelativeEncoder pivotEncoder;
  private RelativeEncoder intakeEncoder;

  //Creates PIDController
  private SparkPIDController pivotPID;
  private SparkPIDController intakePID;

  //Limit Switch
  private DigitalInput limitSwitch;

  private GenericEntry testing;

  /** Creates a new Infeed. */
  public Infeed() {
    //Initalizes CANSparkMax Motors
    pivotMotor = new CANSparkMax(Constants.InfeedConstants.PIVOT_ID, MotorType.kBrushless);
    intakeMotor1 = new CANSparkMax(Constants.InfeedConstants.INTAKE1_ID, MotorType.kBrushless);

    //Defines PID controller
    pivotPID = pivotMotor.getPIDController();
    intakePID = intakeMotor1.getPIDController();

    //Gets Encoders for Pivot Motor
    pivotEncoder = pivotMotor.getEncoder();
    intakeEncoder = intakeMotor1.getEncoder();

    //Start position of Pivot Motor
    pivotEncoder.setPosition(0);

    //Resets Motors to factory defaults
    pivotMotor.restoreFactoryDefaults();
    intakeMotor1.restoreFactoryDefaults();

    pivotMotor.setSmartCurrentLimit(50);

    //Sets idle mode for when motors aren't being directly used
    pivotMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor1.setIdleMode(IdleMode.kBrake);

    //Creates PID for Pivot
    pivotPID.setP(Constants.InfeedConstants.pivotP);
    pivotPID.setI(Constants.InfeedConstants.pivotI);
    pivotPID.setD(Constants.InfeedConstants.pivotD);
    pivotPID.setIZone(Constants.InfeedConstants.pivotILimit);

    //Creates PID for Intake
    intakePID.setFF(Constants.InfeedConstants.kF);
    intakePID.setP(Constants.InfeedConstants.kP);
    intakePID.setI(Constants.InfeedConstants.kI);
    intakePID.setD(Constants.InfeedConstants.kD);

    //Create limit switch
    limitSwitch = new DigitalInput(9);

  }
  //Gets position of pivotMotor
  public double getPosition(){
    return pivotEncoder.getPosition();
  }
  //Sets point of pivot motor
  public void setPoint(double position){
    pivotPID.setReference(position, ControlType.kPosition);
  }
  //Makes Infeed take in a Note
  public void intake(){
    intakeMotor1.set(Constants.InfeedConstants.INTAKE_SPEED);
  }
  //Makes Infeed expel Note
  public void expel(){
    intakeMotor1.set(Constants.InfeedConstants.EXPEL_SPEED);
  }
  //Stop Infeed Motor
  public void stopIntake(){
    intakeMotor1.set(0);
  }
  //Rotates the pivot Motor to ground
  public void rotatetoGround(){
    //If pivot motor position is greater than out_position pivot motor stops
    if(pivotEncoder.getPosition() > Constants.InfeedConstants.OUT_POSITION){
        pivotMotor.set(0);
    } else {
      //Other wise pivot motor moves at -35% speed
      pivotMotor.set(Constants.InfeedConstants.PIVOT_TO_GROUND_SPEED);
    }

  }
  //Rotates the pivot motor to Shooter
  public void rotatetoShooter(){
    //If pivot motor position is greater than in_position pivot motor moves back at -20% speed
    if (pivotEncoder.getPosition() > Constants.InfeedConstants.IN_POSITION){
      pivotMotor.set(Constants.InfeedConstants.OVERSHOOT_FIX_SPEED);
    }
    else{
      //Other wise pivot motor moves at 35% speed
      pivotMotor.set(Constants.InfeedConstants.PIVOT_TO_SHOOTER_SPEED);
    }
  }
  //Stops pivot motor
  public void stopPivot(){
    pivotMotor.set(0);
  }
  //Resets enocoder position to 0
  public void resetEncoder(){
    pivotEncoder.setPosition(0);
  }

  public boolean getLimitSwitch(){
    return limitSwitch.get();
  }

    public boolean getLimitSwitchReverse(){
    return !limitSwitch.get();
  }

  public void setVelocity(double velocity){
    intakePID.setReference(velocity, ControlType.kVelocity);
  }
  public double getVelocity(){
    return intakeEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Creates SmartDashBoard pivotMotor Encoder Value
    SmartDashboard.putNumber("pivot ", getPosition());
    SmartDashboard.putBoolean("Has Note", getLimitSwitchReverse());
    SmartDashboard.putNumber(("Infeed Speed"), getVelocity());

  }
}
