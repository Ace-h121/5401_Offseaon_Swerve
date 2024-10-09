// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
// WPI imports
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.Constants.ClimberConstants;
// Subsystem imports
import frc.robot.subsystems.Climber;

public class ClimberMove extends Command {
  private Climber climber;

  private String side;
  private double speed;

  private XboxController controller = Controls.xbox_operator;
  /** Creates a new Climber. */
  public ClimberMove(Climber m_climber, String m_side) {
    climber = m_climber;
    side = m_side;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (side == "Left"){
      speed = controller.getLeftY();
    }
    else{
      speed = controller.getRightY();
    }
    if (speed > Constants.ClimberConstants.CONTROLLER_DEADZONE || speed < -ClimberConstants.CONTROLLER_DEADZONE){
      climber.climb(speed);
    } else {
      climber.climb(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}