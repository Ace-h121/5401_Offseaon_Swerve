// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**Removes note from infeed */

package frc.robot.Commands;

//WPI Imports
import edu.wpi.first.wpilibj2.command.Command;

//Subsystem Imports
import frc.robot.subsystems.Infeed;

public class Expel extends Command {
  //Declares Variables
  private Infeed expel;

  private boolean endCommand = false;

  /** Creates a new Expel. */
  public Expel(Infeed m_expel) {
    //Makes local variable equal to global variable
    expel = m_expel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(expel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    expel.expel();
    endCommand = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}