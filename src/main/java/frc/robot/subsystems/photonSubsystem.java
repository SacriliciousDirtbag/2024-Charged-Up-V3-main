package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.targeting.PhotonPipelineResult;

//PHOTONVISION IMPORTS
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.*;

import java.io.IOException;
import java.nio.channels.Pipe;
import java.sql.Driver;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class photonSubsystem extends SubsystemBase{
  static PhotonCamera camera;
  static SwerveDrivePoseEstimator poseEstimator;
  PhotonPipelineResult lf; //latest frame
  PhotonTrackedTarget ct; //current target
  Transform3d pti; //idk what this is arlen
  final double chog = Units.inchesToMeters(20.25); //camera height off ground
  final double thog = Units.inchesToMeters(34); //target height off ground
  final double cpr = 0; //camera pitch radians
  double distance;


  Pose3d robotPose;
  Pose2d robotPose1;

  Pose2d goalPose;


  Optional<PhotonTrackedTarget> targetOpt;


  private PhotonTrackedTarget lastTarget;

   private static final int TAG_TO_CHASE = 3;
    private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(2, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, -90));


  Supplier<Pose2d> poseSupplier;
  Pose2d robotPose2d;
  
  public photonSubsystem(Swerve s_Swerve, Supplier<Pose2d> ps)
  {
    this.poseSupplier = ps;
    camera = new PhotonCamera( "PhotonCamera1");
    poseEstimator =  new SwerveDrivePoseEstimator(
    Constants.Swerve.swerveKinematics, 
    s_Swerve.getYaw(),
    s_Swerve.getModulePositions(), 
    s_Swerve.getPose()
    );
  }

  @Override
  public void periodic()
  {
    lastTarget = null;
    robotPose1 = poseSupplier.get();
    SmartDashboard.putNumber("Robot Pose2D Rotation-Value", getRobot1Pose().getRotation().getDegrees());
    
    robotPose2d = poseSupplier.get();
        robotPose = new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(), 
            0.0, 
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getDegrees())
        );

        var photonRes = camera.getLatestResult();
        if (photonRes.hasTargets()) {
          // Find the tag we want to chase
          targetOpt = photonRes.getTargets().stream().filter(t -> t.getFiducialId() == TAG_TO_CHASE).findFirst();;
          //     .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
          //     .findFirst();
          if (targetOpt.isPresent()) {
            var target = targetOpt.get();
            // This is new target data, so recalculate the goal
            lastTarget = target;
            
            // Transform the robot's pose to find the camera's pose
            var cameraPose = robotPose.transformBy(Constants.cameraSettings.ROBOT_TO_CAMERA);
    
            // Trasnform the camera's pose to the target's pose
            var camToTarget = target.getBestCameraToTarget();
            var targetPose = cameraPose.transformBy(camToTarget);
            
            // Transform the tag's pose to set our goal
            goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

            SmartDashboard.putNumber("Robot Pose2D X-Value", getRobotPose().getX());
            SmartDashboard.putNumber("Robot1 Pose2D Y-Value", getRobotPose().getY());
            SmartDashboard.putNumber("Robot1 Pose2D Rot-Value", getRobot1Pose().getRotation().getDegrees());

          }
        }
}

  public Pose2d getGoalPose()
  {
    return goalPose;
  }

  public Pose3d getRobotPose()
  {
    return robotPose;
  }

  public Pose2d getRobot1Pose()
  {
    if(robotPose1 == null){
      return new Pose2d();
    }else{
    return robotPose1;
  }
  }

  public double getYaw()
  {
    if(ct != null)
    {
      return ct.getYaw();
    }
    return 0.0;
  }

  public double getDistance()
  {
    if(ct != null)
    {
      return distance;
    }
    return 0.0;
  }

  public Pose2d getRobotPose2d()
  {
     if(robotPose2d == null){
      return new Pose2d();
    }else{
    return robotPose2d;
  }
  }

  public static PhotonCamera getCamera(){
    return camera;
  }
  
}
