package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {



    //Moving Target Calcs
    public static final double shotVelocity = 12.7;

    //Target locations
    public static final double redSpeakerLocationX = 16.314; //was 16.542
    public static final double redSpeakerLocationY = 5.56;
    public static final double blueSpeakerLocationX = 0.228;
    public static final double blueSpeakerLocationY = 5.56;
    
    //Auto Rotate PID Values
    public static final double autoRotate_P = .01;
    public static final double autoRotate_I = 0;
    public static final double autoRotate_D = 0.00065;
    public static final double autoRotate_ks = 0.028;

    public static final double AUTOROTATE_MAX = .75;
    public static final double AUTOROTATE_MIN = -0.75;
    public static final double AUTOROTATE_TOL = 3;

    //Auto Pitch PID Values
    public static final double defaultPitch = 50; //toggle if Pitch aint workin
    public static final double pitchJogSpeed = .2;
    public static final double pitch_tol = .5;
    public static final double autoPitch_P = 0.045;
    public static final double autoPitch_I = 0;
    public static final double autoPitch_D = 0;
    public static final double autoPitch_ks = 0.011;

    public static final double pitchMaxOutput = .75;
    public static final double pitchMinOutput = -.75;   
    public static final double pitchMaxAngle = 60;
    public static final double pitchMinAngle = 0;
    public static final Rotation2d shooterPitchCancoderCal = Rotation2d.fromDegrees(111);

    //Shooter PID 
    public static final double AutoShooter_P = .5;
    public static final double AutoShooter_I = 1.5;
    public static final double AutoShooter_D = 0.0;
   

    //Auto Note Pickup
    public static final double rotateP = 0.01;
    public static final double strafeP = 0.01;
    public static final double translatePRotateStrat = 0.0178;
    public static final double translatePStrafeStrat = 0.005;


    //Intake
    public static final double intakePercent = 1;
    public static final double feedPercent = 1;

    public static final double ShooterSpeed = 4500;
    public static final double IdleSpeed = 3000;

    public static final double targetSpeedTolerance = 100;
   
    public static final double aim_adjust = 13;


    //Climber
    public static final double ClimberKg = 0;
    public static final double ClimberJogPercent = .2;
    public static final double Climberconfigs_P = .6;
    public static final double Climberconfigs_I = 0;
    public static final double Climberconfigs_D = 0;
    public static final double Climberconfigs_kG = 0;
    public static final double Climberconfigs_kS = 0.05;
    public static final double Climberconfigs_kG_Climb = 0;
    public static final double Climberconfigs_kS_Climb = 0;
    
    public static final double ClimberRotPerDist = .6726;
    public static final double ClimberMAXPos = 845;
    public static final double ClimberMINPos = 339;
    public static final double ClimberUpperLimitPos = 845;
    public static final double ClimberLowerLimitPos = 339;
    public static final double ClimberTolerance = 3;
    public static final double ClimberExtend = 844;
    public static final double ClimberRetracted = 340;

    //AmpLift
    public static final double AmpLiftKg = 0;
    public static final double AmpLiftJogPercent = .1;
    public static final double AmpLiftconfigs_P = 5;
    public static final double AmpLiftconfigs_I = 0;
    public static final double AmpLiftconfigs_D = 0;
    public static final double AmpLiftconfigs_kG = 0;
    public static final double AmpLiftconfigs_kS = .4;

    public static final double AmpLiftRotPerDist = .08733;
    public static final double AmpLiftMAXPos = 870;
    public static final double AmpLiftMINPos = 245;
    public static final double AmpLiftUpperLimitPos = 870;
    public static final double AmpLiftLowerLimitPos = 245;
    public static final double AmpLiftTolerance = 3;

    //AmpShoot
    public static final double AmpRollerShootPercent = 1;
    public static final double AmpRollerShootTimout = 1;


    //Hand Off
    public static final double IntakeHandOffPercent = .5;
    public static final double ShooterHandOffPercent = .2;
    public static final double AmpLiftHandOffPercent = 1;
    public static final double PitchPOS_Handoff = 5;

    //Lob Shot
    public static final double LobShotPitch = 50;
    public static final double LobShotRPM = 4000;
    public static final double LobShotblueRot = 165;
    public static final double LobShotredRot = 45;

    //Post Shot
    public static final double PostShotPitch = 27;
    public static final double PostShotRPM = 4500;
    public static final double PostShotblueRot = -173.8;
    public static final double PostShotredRot = 10;




    //Amp Lift POS Sequentials
    public static final double AmpLiftPOS_HandOff = 257;
    public static final double AmpLiftPOS_Amp = 715;
    public static final double AmpLiftPOS_Trap = 860;
    public static final double AmpLiftPOS_Retracted = 295;

    //pitch tolerace/adjuster //was at 3 now at .5
    public static final double robotAngle_tol = .5;
    public static final double ShooterPitchCalc_A = -0.514;
    public static final double ShooterPitchCalc_B = 6.73;    
    public static final double ShooterPitchCalc_C = -31;    
    public static final double ShooterPitchCalc_D = 77.4;



    public static final double stickDeadband = 0.1;



    public static final class Swerve {
        public static final int pigeonID = 9;
        // changed pigeon code from 1 to 9

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);
        // changed L2 to L1

        /* Drivetrain Constants */
        public static final double trackWidth = .53; //TODO: This must be tuned to specific robot
        // Center to Center distance of left and right modules in meters.
        public static final double wheelBase = .47; //TODO: This must be tuned to specific robot
        // Center to Center distance of front and rear module wheels in meters.

        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

       

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 11; //1 for rio bot 11 for andy bot
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-128.40);// 168.2 rio og
            // changed degree from 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 12; //2 for rio bot 12 for andy bot
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(166.38);// 98.17 rio og
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 13; //3 for rio bot 13 for andy bot
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(97.29);// 43.3 rio og
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 14; //4 for rio bot 14 for andy bot
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-122.696)   ; // -71.63 og

            




































            
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }



}
