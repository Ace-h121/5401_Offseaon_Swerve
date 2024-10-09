// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//3rd Party 
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

//WPI
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.AmpShot;
import frc.robot.Commands.ClimberMove;
import frc.robot.Commands.Expel;
import frc.robot.Commands.Intake;
import frc.robot.Commands.RotatePivotAir;
import frc.robot.Commands.RotatePivotGround;
import frc.robot.Commands.RotatePivotShooter;
import frc.robot.Commands.ShooterBack;
import frc.robot.Commands.SpeakerShot;
import frc.robot.Commands.StopAll;
//Commands
import frc.robot.Commands.Auto.AutoDefenseShot;
import frc.robot.Commands.Auto.AutoShoot;
import frc.robot.Commands.Auto.DriveToPositionCommand;
import frc.robot.Commands.Auto.FourPieceAuto;
import frc.robot.Commands.Auto.JustShoot;
import frc.robot.Commands.Auto.LimitSwitchCommand;
import frc.robot.Commands.Auto.OnePieceAuto;
import frc.robot.Commands.Auto.SideAuto;
import frc.robot.Commands.Auto.ThreePieceAuto;
import frc.robot.Commands.Auto.TwoPieceAuto;
import frc.robot.subsystems.CANdleSystem;
//Subsystems
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;

//Constants
import frc.robot.generated.TunerConstants;



public class RobotContainer {


  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    private final CommandXboxController operator = Controls.operator;
    private final CommandXboxController driver = Controls.driver;

    /* Auto chooser 
     * TODO: Change to choose your own auto
     */
    private final SendableChooser<Command> chooser = new SendableChooser<>();

    /*Intake */
    private final Infeed infeed = new Infeed();
    /*Shooter */
    private final Shooter shooter = new Shooter();
    /*Climbers */
    private final Climber leftClimber = new Climber(Constants.ClimberConstants.LEFTCLIMBER_ID, false, "Left", 8);
    private final Climber rightClimber = new Climber(Constants.ClimberConstants.RIGHTCLIMBER_ID, true, "Right", 7);

    /*CANDLE */
    private final CANdleSystem candle = new CANdleSystem(operator.getHID());
    

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  //trigger for limit switch
  private final Trigger hasNote = new Trigger(infeed::getLimitSwitch);

  private void configureBindings() {
    
    
    drivetrain.setDefaultCommand(
       // Drivetrain will execute this command periodically
       
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            
            
            
        ));

    leftClimber.setDefaultCommand(new ClimberMove(leftClimber, "Left"));
    rightClimber.setDefaultCommand(new ClimberMove(rightClimber, "Right"));

        /*
     * Low Power Shot - Bottom DPAD 
     * Speaker Shot - Up DPAD
     * Left climber - left joysticks
     * right climber - right joystick
     * Expel - Right Trigger
     * Intake - Left Trigger
     * Rotate Pivot Air - B
     * Rotate Pivot Grouns - Y
     * Rotate Pivot Shooter - A
     * Stop All - Left Bumber
     */

    /** Intake Commands */
    //If "Right Trigger" pressed/held on operator controller expel command used (removes note from infeed)
    operator.rightTrigger().onTrue(new Expel(infeed));
    //If "Left Trigger" pressed/held on operator controller intake command used (sucks note into infeed)
    operator.leftTrigger().onTrue(new Intake(infeed));
    //If "Left Bumper" pressed/held on operator controller stop intake command used (stops infeed)
    operator.leftBumper().onTrue(new StopAll(infeed, shooter));

    /** Pivot Commands */
    //If "Y" pressed/held on operator controller rotatepivotground command used (moves intake to ground)
    operator.y().whileTrue(new SequentialCommandGroup(new RotatePivotGround(infeed), new Intake(infeed)));
    //If "A" pressed/held on operator controller rotatepivotshooter command used (moves intake to shooter)
    operator.a().whileTrue(new RotatePivotShooter(infeed));
    //If "B" pressed/held on operator controller rotatepivotAir command used (moves intake to air)
    operator.b().whileTrue(new RotatePivotAir(infeed));
    operator.x().whileTrue(new ParallelCommandGroup(new ShooterBack(shooter), new Intake(infeed)));

    //Auto Shooter Methods.
    operator.povUp().onTrue(new ParallelCommandGroup(new AutoShoot(infeed, shooter)));
    operator.povDown().onTrue(new ParallelCommandGroup(new AmpShot(infeed)));
    operator.povLeft().onTrue(new SpeakerShot(shooter));

    hasNote.onFalse(new SequentialCommandGroup(new ParallelCommandGroup(new RotatePivotShooter(infeed)), new LimitSwitchCommand(infeed, shooter)));
    
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
    joystick.x().whileTrue(new DriveToPositionCommand(drivetrain, ()-> new Pose2d(2, 3, new Rotation2d(0)), false)); 

    
    /* 
    //binding the sysid gears, once again comment out when not in use. 
    joystick.x().onTrue(drivetrain.applyRequest(()-> translation.withVolts(Units.Volts.ofBaseUnits(6))));
    joystick.a().whileTrue(drivetrain.applyRequest(()-> rotate.withVolts(Units.Volts.ofBaseUnits(6))));
    joystick.b().whileTrue(drivetrain.applyRequest(()-> steer.withVolts(Units.Volts.ofBaseUnits(6))));
*/

  
    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    //drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    chooseAuto();
    configureBindings();
  }

  public void chooseAuto(){
    

    chooser.addOption("One Piece", new OnePieceAuto(infeed, shooter, drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    chooser.addOption("Two Piece", new TwoPieceAuto(infeed, shooter, drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    chooser.addOption("Three Piece Amp", new ThreePieceAuto(infeed, shooter, drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    chooser.addOption("Four Piece", new FourPieceAuto(infeed, shooter, drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    chooser.addOption("Peddie Edition", new JustShoot(infeed, shooter).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    chooser.addOption("Do Nothing", Commands.print("Oops"));


    Shuffleboard.getTab("Autonomous").add(chooser);

  }

  public static void endgameRumble(){
    if (DriverStation.isEnabled()){
      if (DriverStationJNI.getMatchTime() <= 25 && DriverStationJNI.getMatchTime() >= 1 ){
        Controls.xbox_driver.setRumble(RumbleType.kBothRumble, 1);
        Controls.xbox_operator.setRumble(RumbleType.kBothRumble, 1);
      }
      else {
        Controls.xbox_driver.setRumble(RumbleType.kBothRumble, 0);
        Controls.xbox_operator.setRumble(RumbleType.kBothRumble, 0);
      }
    }
  }


  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
