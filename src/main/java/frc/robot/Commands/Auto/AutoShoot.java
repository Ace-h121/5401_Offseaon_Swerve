// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.Expel;
import frc.robot.Commands.RotatePivotShooter;
import frc.robot.Commands.SpeakerShot;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends SequentialCommandGroup {
  private Infeed infeed;
  private Shooter shooter; 

  /** Creates a new AutoShoot. */
  public AutoShoot(Infeed m_infeed, Shooter m_shooter) {
    infeed = m_infeed;
    shooter = m_shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, infeed);
    addCommands(
      new SpeakerShot(shooter), 
      
      new Expel(infeed)
    );

      
  }


}
