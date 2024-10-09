// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// 24 1/2


package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Photonvision extends SubsystemBase{
  /** Creates a new NetworkTables. */
  PhotonCamera camera;


  public Photonvision(String cameraName) {
    camera = new PhotonCamera(cameraName);

    camera.setPipelineIndex(0);

    
  }

  public PhotonCamera getCamera(){
    return camera;
  }

 
  public boolean hasTarget(){
    var result = camera.getLatestResult();
    return result.hasTargets();
   }
 
   double getAprilTagID(){
     var result = camera.getLatestResult();
     if(hasTarget()){
       return result.getBestTarget().getFiducialId();
     }
     return 0;
   }
 
   double getTargetYaw(){
 
     var result = camera.getLatestResult();
     if(hasTarget()){
       return result.getBestTarget().getYaw();
     }
     return 0;
   }
 
   double getTargetArea(){
     var result = camera.getLatestResult();
     if(hasTarget()){
       return result.getBestTarget().getArea();
     }
     return 0;
   }

  /**
  @return returns value in meters
  */
  public double getTargetHeight(int id){
    switch(id){
      case 1: return 122/100;
      case 2: return 122/100;
      case 3: return 1.37;
      case 4: return 1.37;
      case 5: return 122/100;
      case 6: return 122/100;
      case 7: return 132/100;
      case 8: return 132/100;
      case 9: return 122/100;
      case 10: return 122/100;
      case 11: return 121/100;
      case 12: return 121/100;
      case 13: return 121/100;
      case 14: return 121/100;
      case 15: return 121/100;
      case 16: return 121/100;

      default: return 0;
    }
    
  }

  public double getDistance(){
     var result = camera.getLatestResult();
     
     PhotonTrackedTarget target = null;

    if (result.hasTargets()) {
      // First calculate range
      var targetList = result.getTargets();
      target = removeBadTargets(targetList);

      }
      if (target != null){
      double range =
        PhotonUtils.calculateDistanceToTargetMeters(
          Units.inchesToMeters(Constants.PhotonConstants.CAMERA_HEIGHT),
          getTargetHeight(target.getFiducialId()),
          Units.degreesToRadians(Constants.PhotonConstants.CAMERA_ANGLE),
          Units.degreesToRadians(target.getPitch())
          );
         return range;
      }
        
      return 0;
  }

  public PhotonTrackedTarget removeBadTargets(List<PhotonTrackedTarget> targetList){
    PhotonTrackedTarget target = null;
      for (PhotonTrackedTarget m_target : targetList){
        if (m_target.getFiducialId() != 3 || m_target.getFiducialId() != 8){
          target = m_target;
          break;
        }
      
      }
      
      return target;

  }




  public double getYaw(){
    var result = camera.getLatestResult();
    var targets = result.getTargets();
    var target = removeBadTargets(targets);
    if (target != null){
      return target.getYaw();
    }
    return 0;
  }

  public void shuffleBoardTabs(){
  }



  @Override
  public void periodic(){
    shuffleBoardTabs();
  }
}
