// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Sucks note in using infeed */

package frc.robot.Commands;

import edu.wpi.first.wpilibj.DigitalInput;
//WPI imports
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
//File Impots
import frc.robot.subsystems.Infeed;

public class Intake extends Command {
  //Declares Variables
  Infeed intake;
  boolean endCommand = false;


  /** Creates a new Intake. */
  public Intake(Infeed m_intake) {
    //Makes local variable equal to global variable
    intake = m_intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getLimitSwitch()){
      intake.intake();
      endCommand = true;
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
    
  }
}