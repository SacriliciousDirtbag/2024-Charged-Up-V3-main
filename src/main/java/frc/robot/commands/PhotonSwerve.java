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

    ProfiledPIDController xPidController;
    ProfiledPIDController yPidController;
    ProfiledPIDController thController;

    Supplier<Pose2d> poseSupplier;

    private PhotonTrackedTarget lastTarget;
    private Swerve swerve;

    Pose2d robotPose1;
    Pose2d goalPose;
    Pose3d robotPose;

    public PhotonSwerve(photonSubsystem camera, ProfiledPIDController xController,
     ProfiledPIDController yController, ProfiledPIDController thController, Swerve Swerve)  
    {
        this.camera = camera;
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
        robotPose1 = camera.getRobot1Pose();
        thController.reset(robotPose1.getRotation().getRadians());
        xPidController.reset(robotPose1.getX());
        yPidController.reset(robotPose1.getY());
    }

    @Override 
    public void execute()
    {
        goalPose = camera.getGoalPose();
        robotPose = camera.getRobotPose();
        robotPose1 = camera.getRobotPose2d();
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

        double omegaSpeed = thController.calculate(robotPose1.getRotation().getRadians());
        if (thController.atGoal()) {
            omegaSpeed = 0;
        }


        ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,omegaSpeed,robotPose1.getRotation());

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
            swerve.setModStates(desiredStates);
            // SwerveModulePosition[] newMods =  swerve.getModulePositions();
        }
        desiredChassisSpeeds = null;
    } 

    @Override
    public void end(boolean isFinished)
    {
        SwerveModuleState[] stop = new SwerveModuleState[4];
        for(int i = 0; i < stop.length; i++)
        {
            stop[i] = new SwerveModuleState();
        }
        swerve.setModStates(stop);
    }
}


