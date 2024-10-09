// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Infeed;

public class AmpShot extends Command {
  private Infeed infeed;
  private boolean endCommand = false;

  /** Creates a new SpeakerShot. */
  public AmpShot(Infeed m_infeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    infeed = m_infeed;
    addRequirements(infeed);
  }

  // Called when the command is initially scheduled.
	public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    infeed.setVelocity(Constants.InfeedConstants.AMP_RPM);

  }
  // Called once the command ends or is interrupted.

  @Override
  public void end(boolean interrupted) {}

  //Returns true when the command should end.

  @Override
  public boolean isFinished() {
    return endCommand;
  }
}
