package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.sim.ChassisReference;
import com.fasterxml.jackson.core.TreeNode;

import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;

import java.util.stream.IntStream;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.photonSubsystem;

import frc.robot.Constants.cameraSettings;


public class PhotonSwerve extends Command{
    PIDController movementController;
    photonSubsystem camera;
    
    Pose2d robPose2d;
    Pose3d robotPose;

    //Tag ID
    private static final int TAG_TO_CHASE = 3;
    private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(1.5, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));

    ProfiledPIDController xPidController;
    ProfiledPIDController yPidController;
    ProfiledPIDController thController;

    Supplier<Pose2d> poseSupplier;

    private PhotonTrackedTarget lastTarget;
    private Swerve swerve;

    public PhotonSwerve(photonSubsystem camera, ProfiledPIDController xController,
     ProfiledPIDController yController, ProfiledPIDController thController, Supplier<Pose2d> ps, Swerve Swerve)  
    {
        this.camera = camera;
        this.poseSupplier = ps;
        this.swerve = Swerve;

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
        thController.reset(robotPose.getRotation().getRadians());
        xPidController.reset(robotPose.getX());
        yPidController.reset(robotPose.getY());
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
            xPidController.setGoal(goalPose.getX());
            yPidController.setGoal(goalPose.getY());
            thController.setGoal(goalPose.getRotation().getRadians());

            double xSpeed = xPidController.calculate(robotPose.getX());
            if (xPidController.atGoal()) {
              xSpeed = 0;
            }
      
            double ySpeed = yPidController.calculate(robotPose.getY());
            if (yPidController.atGoal()) {
              ySpeed = 0;
            }
      
            double omegaSpeed = thController.calculate(robotPose2d.getRotation().getRadians());
            if (thController.atGoal()) {
              omegaSpeed = 0;
            }
      


            ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,omegaSpeed,robotPose2d.getRotation());
            SwerveModuleState[] modStates =  Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);


                 // Set the swerve module states
    if (desiredChassisSpeeds != null) {
        var currentStates = swerve.getModulePositions();
        var desiredStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
  
        if(desiredChassisSpeeds.vxMetersPerSecond == 0.0 && desiredChassisSpeeds.vyMetersPerSecond == 0.0
            && desiredChassisSpeeds.omegaRadiansPerSecond == 0.0) {
          // Keep the wheels at their current angle when stopped, don't snap back to straight
          IntStream.range(0, currentStates.length).forEach(i -> desiredStates[i].angle = currentStates[i].angle);
        }
  
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.AutoConstants.kMaxSpeedMetersPerSecond);
        swerve.setModStates(modStates);
        // SwerveModulePosition[] newMods =  swerve.getModulePositions();
      }
      desiredChassisSpeeds = null;
          }

          
        }


     
    }
}
