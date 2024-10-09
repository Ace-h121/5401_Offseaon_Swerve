// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;

public class LimitSwitchCommand extends Command {
  Infeed infeed;
  Shooter shooter;
  boolean endCommand = false;

  /** Creates a new Pivot. */
  public LimitSwitchCommand(Infeed m_stoppivot, Shooter m_shooter) {
    //Makes local variable equal to global variable
    infeed = m_stoppivot;
    shooter = m_shooter;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(infeed, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!(infeed.getVelocity() < 0)){
    infeed.stopIntake();
    shooter.stop();
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
