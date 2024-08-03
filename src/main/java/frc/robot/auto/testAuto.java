// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class testAuto extends SequentialCommandGroup {
  CommandSwerveDrivetrain drivetrain;
  /** Creates a new testAuto. */
  public testAuto(CommandSwerveDrivetrain m_Drivetrain) {
    
    drivetrain = m_Drivetrain;
    
    // An example trajectory to follow.  All units in meters.
    Trajectory trajectory = Constants.Trajectorys.sCurveTrajectory;
        
    //pass the auto and the drivebase into a factory function
    SwerveControllerCommand sCurve = drivetrain.getAutoCommand(drivetrain, trajectory);

    SwerveControllerCommand rSCurve = drivetrain.getAutoCommand(drivetrain, Constants.Trajectorys.rSCurveTrajectory);

            

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());        
    addCommands(sCurve,rSCurve);
    
  }
}
