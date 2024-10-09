// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Commands.Expel;
import frc.robot.Commands.Intake;
import frc.robot.Commands.RotatePivotAir;
import frc.robot.Commands.RotatePivotGround;
import frc.robot.Commands.RotatePivotSafe;
import frc.robot.Commands.RotatePivotShooter;
import frc.robot.Commands.SpeakerShot;
import frc.robot.Commands.StopAll;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceAuto extends SequentialCommandGroup {
  Infeed infeed;
  Shooter shooter;
  CommandSwerveDrivetrain drivebase;


  /** Creates a new TwoPieceAuto. */
  public TwoPieceAuto(Infeed m_infeed, Shooter m_shooter, CommandSwerveDrivetrain m_drivebase) {
    infeed = m_infeed;
    shooter = m_shooter;
    drivebase = m_drivebase;

    // Add your commands in the addCommands() call, e.g.

    addRequirements(drivebase,infeed,shooter);
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShoot(infeed, shooter),
      new WaitCommand(.35),
      new StopAll(infeed, shooter),
      new RotatePivotGround(infeed),
      new ParallelCommandGroup( drivebase.getAutoCommand(Constants.Trajectorys.leaveStart), new Intake(infeed)).until(infeed::getLimitSwitchReverse),
      new StopAll(infeed, shooter),
      new ParallelCommandGroup(drivebase.getAutoCommand(Constants.Trajectorys.backToSpeaker), new RotatePivotShooter(infeed), new SpeakerShot(shooter)),
      new Expel(infeed)
    );
  }
}
