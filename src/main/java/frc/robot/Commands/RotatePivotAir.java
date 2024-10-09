// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**Moves pivot into air so infeed touches nothing */

package frc.robot.Commands;

//WPI Imports
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

//File Imports
import frc.robot.subsystems.Infeed;

public class RotatePivotAir extends Command {
  //Declaring variables
  Infeed infeed;
  boolean endCommand = false;

  /** Creates a new Pivot. */
  public RotatePivotAir(Infeed m_rotatetoground) {
    //Makes local variable equal to global variable
    infeed = m_rotatetoground;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(infeed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    infeed.setPoint(Constants.InfeedConstants.AIR_POSITION);
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
