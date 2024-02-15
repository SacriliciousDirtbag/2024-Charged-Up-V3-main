package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.core.TreeNode;

import frc.robot.Constants;
import frc.robot.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.photonSubsystem;

import frc.robot.Constants.cameraSettings;


public class PhotonSwerve extends Command{
    PIDController movementController;
    PhotonCamera camera;
    
    Pose2d pose2d;
    Pose3d pose3d;

    private static final int TAG_TO_CHASE = 2;
    private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(1.5, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));

    PIDController xPidController;
    PIDController yPidController;
    PIDController thController;

    Supplier<Pose2d> poseSupplier;

    private PhotonTrackedTarget lastTarget;

    public PhotonSwerve(PhotonCamera camera, PIDController xController,
     PIDController yController, PIDController thController, Supplier<Pose2d> ps)  
    {
        this.camera = camera;
        this.poseSupplier = ps;

        this.xPidController = xController;
        this.yPidController = yController;
        this.thController = thController;

        this.xPidController.setTolerance(0.2);
        this.yPidController.setTolerance(0.2);
        this.thController.setTolerance(Units.degreesToRadians(3));
        this.thController.enableContinuousInput(Math.PI, Math.PI);
    }

    

    @Override
    public void initialize() 
    {
        lastTarget = null;
        var robotPose = poseSupplier.get();
        thController.reset();
        xPidController.reset();
        yPidController.reset();
    }

    @Override 
    public void execute()
    {
        var robotPose2d = poseSupplier.get();
        var robotPose = new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(), 
            0.0, 
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getDegrees())
        );

        var photonRes = camera.getLatestResult();
        if (photonRes.hasTargets()) {
          // Find the tag we want to chase
          var targetOpt = photonRes.getTargets().stream()
              .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
              .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
              .findFirst();
          if (targetOpt.isPresent()) {
            var target = targetOpt.get();
            // This is new target data, so recalculate the goal
            lastTarget = target;
            
            // Transform the robot's pose to find the camera's pose
            var cameraPose = robotPose.transformBy(cameraSettings.ROBOT_TO_CAMERA);
    
            // Trasnform the camera's pose to the target's pose
            var camToTarget = target.getBestCameraToTarget();
            var targetPose = cameraPose.transformBy(camToTarget);
            
            // Transform the tag's pose to set our goal
            var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
    
            // Drive
            xPidController.setSetpoint(goalPose.getX());
            yPidController.setSetpoint(goalPose.getY());
            thController.setSetpoint(goalPose.getRotation().getRadians());
          }
        }
    }
}
