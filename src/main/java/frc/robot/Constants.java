// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Commands.Auto.SideAuto;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public final class Constants {
    public static class ControlConstants{
        public static final int XBOX_CONTROLLER_DRIVER = 0;
        public static final int XBOX_CONTROLLER_OPERATOR =1;
        public static final double AXIS_THRESHOLD = 0.05;
        public static final double SPIN_SENSITIVITY = 0.8;
        
        }

    public static class PhotonConstants{
        public static final double CAMERA_ANGLE = 42.5;
        public static final double CAMERA_HEIGHT = 24.375;
    }
    
    //Controls Constants
    public static class Controls{
        // Defines Driver Controller port number
        public static final int XBOX_CONTROLLER_DRIVER = 0;
        // Defines Driver Operator port number
        public static final int XBOX_CONTROLLER_OPERATOR = 1;
    
        public static final double AXIS_THRESHOLD = 0.05;
        public static final double SPIN_SENSITIVITY = 0.8;
        public static final double DEFAULT_SENSITIVTY = 0.8;
        }
    
    //Drive Motors Constants
    public static class DriveMotors{
        // Defined Motor IDs

    
        // Maxium amount of AMPS allowed to the motors
        public static final int CURRENT_LIMIT = 30;
    
        // PID Constants
        public static final double KP = 2;
        public static final double KI = 0.5;
        public static final double KD = 0; 
        public static final double IZONE = .25;
    
        // Sensitivity Constants
        public static final double PERCISION_SENSITIVITY = 0.4;
        public static final double DEFAULT_SENSITIVTY = 1.0;
    
        // Define stopping
        public static final double POWER_STOP = 0;
    
        // Inverse Direction of Drive Motor
        public static final int INVERSE_DIRECTION = -1;
    
        // Straight Direction
        public static final double STRAIGHT_DIRECTION = 1.0;
        public static final double ANGULAR_KP = 0.02;
        public static final double ANGULAR_KI = 0;
        public static final double ANGULAR_KD = 0; 
    }
    
    //Infeed Constants
    public static class InfeedConstants{
        //Id of infeeds CANSparkMax motors
        public static final int INTAKE1_ID = 14;
        public static final int PIVOT_ID = 13;

        //PID values
        public static final double pivotP = .1;
        public static final double pivotI = 0.00005;
        public static final double pivotD = 9.5;
        public static final double pivotILimit = .2;

        //TODO Tune infeed speed PID
        public static final double kF = 0.00017;
        public static final double kP = 0.00015;
        public static final double kI = 0;
        public static final double kD = 0;

        //set pointa
        public static final double IN_POSITION = -.1;
        public static final double SAFE_POSITION = -2;

        public static final double OUT_POSITION = -14.3;
        public static final double AIR_POSITION = -7.17
        ; //Up -6.5 Faing out -8

        /*  Infeed speed constants  */
        //Remove note to shoot into Amp
        public static final double AMP_SPEED = -0.55;
        public static final double AMP_RPM = -4480; //TODO needs to be tuned to the right rpm
        //Take in note speed
        public static final double INTAKE_SPEED = 0.4;
        //Remove note from infeed speed
        public static final double EXPEL_SPEED = -.8; // was 4.2 changed for hatboro
        //Move Pivot to floor speed
        public static final double PIVOT_TO_GROUND_SPEED = -0.35;
        //Move Pivot to shooter speed
        public static final double PIVOT_TO_SHOOTER_SPEED = 0.35;
        //Fix Pivot position if overshot speed
        public static final double OVERSHOOT_FIX_SPEED = -0.20;
    }


    public final class ShooterConstants{
        public static final int LEAD_ID = 12;
        public static final int FOLLOWER_ID = 15;

        public static final double kF = .00017;
        public static final double kP = .00015;
        public static final double kI = 0;
        public static final double kD = .00000;

        public final class SpeedConstants{
            public static final double SHOOTER_RPM = 5500;
            public static final double Defense_RPM = 1850;
            public static final double TRAP_RPM = 4000;
            public static final double BACK_RPM = 1800;
        }
    }


    public static class ClimberConstants {
        //IDs of climber CANSparkMax motors
        public static final int LEFTCLIMBER_ID = 10;
        public static final int RIGHTCLIMBER_ID = 9;

        // Encoder ranges
        public static final int climberEncoderMax = 25;
        public static final int climberEncoderMin = 0;

        // Climb motor speed
        public static final double climberUpSpeed = 1;
        public static final double climberDownSpeed = -1;
        public static final double CONTROLLER_DEADZONE = .05;

    }

    public static final class Swerve{
        public static final double trackWidth = Units.inchesToMeters(24); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(24); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = 4 *Math.PI;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    }

    public static final class AutoConstants{

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 2;
        public static final double kPYController = 2;
        public static final double kPThetaController = 5;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);


        public static final ProfiledPIDController thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

            
        
    };

    public static final class Trajectorys {
        //config to ensure the robot doesnt try to move through itself
        public static final  TrajectoryConfig config = new TrajectoryConfig(
                    TunerConstants.kSpeedAt12VoltsMps,
                    //TODO: Fix this to be the actual constant. 
                    3.23)
                .setKinematics(Constants.Swerve.swerveKinematics);


        public final static Trajectory sCurveTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0,0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1,1), new Translation2d(-1, 3)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 5, new Rotation2d(Units.degreesToRadians(90))),
                config);

        public final static Trajectory rSCurveTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0,5, new Rotation2d(Units.degreesToRadians(90))),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-1,3), new Translation2d(1, 1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 0, new Rotation2d(0)),
                config);
        
        public final Trajectory sCurveTrajectory2 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0,5, new Rotation2d(Units.degreesToRadians(90))),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-1,3), new Translation2d(1, 1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 0, new Rotation2d(0)),
                config);

        public static final Trajectory leaveStart = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)),
            List.of(new Translation2d(.7, 0)),
            new Pose2d(1.3, 0, new Rotation2d(0)),
            config.setReversed(false));
        public static final Trajectory backToSpeaker = TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.3, 0, new Rotation2d(0)),
            List.of(new Translation2d(.6, 0)),
            new Pose2d(0, 0, new Rotation2d(0)),
            config.setReversed(true));

        public static final Trajectory getNote1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)),
            List.of(new Translation2d(.25, -.45)),
            new Pose2d(1.2, -1.35, new Rotation2d(0)),
            config.setReversed(false));

        public static final Trajectory getNote2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)),
            List.of(new Translation2d(.25, .45)),
            new Pose2d(1.2, 1.35, new Rotation2d(0)),
            config.setReversed(false));

        public static final Trajectory SideAuto = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)),
            List.of(new Translation2d(.5, .25)),
            new Pose2d(1.3, .30, new Rotation2d(0)),
            config);
        
        }
    }

