// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class testAuto extends SequentialCommandGroup {
  SwerveControllerCommand sCurve;
  CommandSwerveDrivetrain drivetrain;
  /** Creates a new testAuto. */
  public testAuto(CommandSwerveDrivetrain m_Drivetrain) {
    
    drivetrain = m_Drivetrain;
    
    // An example trajectory to follow.  All units in meters.
    Trajectory trajectory = Constants.Trajectorys.sCurveTrajectory;
        
    //pass the auto and the drivebase into a factory function
     sCurve = drivetrain.getAutoCommand(trajectory);

    SwerveControllerCommand rSCurve = drivetrain.getAutoCommand(Constants.Trajectorys.rSCurveTrajectory);

            

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());        
    addCommands(sCurve,rSCurve);
    
    
  }

}
