package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
   // public SwerveDriveOdometry swerveOdometry;
    private SwerveDrivePoseEstimator swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private Pigeon2 gyro;
    private final Field2d m_field = new Field2d();
    private PhotonVision s_PhotonVision;

    public Swerve(PhotonVision pv) {




        s_PhotonVision = pv;

        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);


        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);

      //  swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
       swerveOdometry = new SwerveDrivePoseEstimator(
                            Constants.Swerve.swerveKinematics, 
                            getGyroYaw(), 
                            getModulePositions(),
                            new Pose2d(),
                            stateStdDevs, 
                            visionStdDevs);



        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    


    public void driveRobotRelative(ChassisSpeeds chassisCSpeeds){
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisCSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
       return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }


    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        swerveOdometry.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        swerveOdometry.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
       
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void setFieldPosition(double xsetpoint, double ysetpoint){
        
        swerveOdometry.resetPosition(getGyroYaw(),getModulePositions(), new Pose2d(xsetpoint,ysetpoint,getGyroYaw()) );
    }

    public double getDistanceToSpeaker(){
      var alliance = DriverStation.getAlliance();
      double targetX;
      double targetY;
      double currentX;
      double currentY;
      double distance;
      // no inverts
      if (alliance.isPresent() && alliance.get() == Alliance.Red){
        targetX = 16.5;
        targetY = 5.5;
        // placeholders
      } else {
        targetX = 0;
        targetY = 5.5;
        // targetY same as red
      }
      currentX = getPose().getX();
      currentY = getPose().getY();
      
      distance = Math.pow((currentX - targetX) ,2) + Math.pow((currentY - targetY) ,2);
      distance = Math.sqrt(distance);
      return distance;
    }

    public double getAngleToSpeaker(){
        var alliance = DriverStation.getAlliance();
        double targetX;
        double targetY;
        double currentX;
        double currentY;
        double distance;
        double currentAngle;
        double targetAngle;
        currentX = getPose().getX();
        currentY = getPose().getY();
         if (alliance.isPresent() && alliance.get() == Alliance.Red){
            targetX = 16.542;
            targetY = 5.56;
            if(targetY-currentY>0){
                targetAngle = 90-Math.toDegrees(Math.atan((targetX-currentX)/(targetY-currentY)));
            } else if(targetY-currentY<0){
                targetAngle = -90+Math.toDegrees(Math.atan((targetX-currentX)/(currentY-targetY)));
            } else {
                targetAngle = 0;
            }
        // placeholders
      } else {
            targetX = 0;
            targetY = 5.5;
             if(targetY-currentY>0){
                targetAngle = 90+Math.toDegrees(Math.atan((currentX-targetX)/(targetY-currentY)));
            } else if(targetY-currentY<0){
                targetAngle = -90-Math.toDegrees(Math.atan((currentX-targetX)/(currentY-targetY)));
            } else {
                targetAngle = 180;
            }
        // targetY same as red
      }
      return(targetAngle);
    }
    /*gmake function get rotation output 
    take angle & return output(what we should send output of rotation between -1&1)
    */
    public double getRotationOutput(double targetAngle){
        double currentAngle = swerveOdometry.getEstimatedPosition().getRotation().getDegrees();
        double output;
        double error;

        error = targetAngle - currentAngle; 

        if (error<-180) {
            error = error + 360;
        }
        if (error>180) {
            error = error - 360;
        }
        
        output = error*Constants.AUTOROTATE_P;
        if (output>Constants.AUTOROTATE_MAX) {
            output = Constants.AUTOROTATE_MAX;
        }
         if (output<-Constants.AUTOROTATE_MAX) {
            output = -Constants.AUTOROTATE_MAX;
        }

        if (Math.abs(error)>Constants.AUTOROTATE_TOL) {
            if (output>0 && output<Constants.AUTOROTATE_MIN) {
                output = Constants.AUTOROTATE_MIN;
            }
            if (output<0 && output>-Constants.AUTOROTATE_MIN) {
                output = -Constants.AUTOROTATE_MIN;
            }

        }
        SmartDashboard.putNumber("output", output);
        SmartDashboard.putNumber("error", error);
            return(output);
    }


    public boolean aimedAtSpeaker(){
        double currentAngle = swerveOdometry.getEstimatedPosition().getRotation().getDegrees();
        double targetAngle = getAngleToSpeaker();
        double error;

        error = targetAngle - currentAngle; 

        if (error<-180) {
            error = error + 360;
        }
        if (error>180) {
            error = error - 360;
        }
        return(Math.abs(error)< 3);
    }


    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        
        var visionEst = s_PhotonVision.getEstimatedGlobalPose();
        visionEst.ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = s_PhotonVision.getEstimationStdDevs(estPose);
                    
                    addVisionMeasurement(
                        estPose, est.timestampSeconds, estStdDevs);
                        
                    SmartDashboard.putNumber("Est Pose X", estPose.getX());
                    SmartDashboard.putNumber("Est Pose Y", estPose.getY());
                  
        });
        
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
            
        }

        m_field.setRobotPose(swerveOdometry.getEstimatedPosition()); 
        SmartDashboard.putData("Field", m_field);
        
        SmartDashboard.putNumber("DistanceToSpeaker", getDistanceToSpeaker());//??
        SmartDashboard.putNumber("AngleToSpeaker", getAngleToSpeaker());//??
        
        
    }
}