package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    /* Controllers */
    private final Joystick leftJoystick = new Joystick(0);
    public final Joystick rightJoystick = new Joystick(1);
    private final Joystick OP_Panel = new Joystick(2);
    /* Drive Controls */

    /* Driver Buttons */

    /* Subsystems */
    
    public static final PhotonVision s_PhotonVision =  new PhotonVision();
    public final static Swerve s_Swerve = new Swerve(s_PhotonVision);
    public final static Shooter s_Shooter = new Shooter();
    public final static Intake s_Intake = new Intake();
    public final static Limelight s_Limelight = new Limelight();
    public final static Climber s_Climber = new Climber();
    public final static AmpLift s_AmpLift = new AmpLift();
    public final static Candle s_Candle = new Candle(); 
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        // changed to negative so 
                        () -> -leftJoystick.getRawAxis(1),
                        () -> -leftJoystick.getRawAxis(0),
                        () -> -rightJoystick.getRawAxis(0),
                        () -> OP_Panel.getRawButton(1),
                        () -> rightJoystick.getRawButton(3)
                        ));

                        //changed drivers to joysticks and axis numbered
                        //deleted robotcentric and joystick thingy
                        // removed the (-) symbols in front of leftJoystick.getRawAxis(1),() -> leftJoystick.getRawAxis(0),() -> rightJoystick.getRawAxis(2),


        NamedCommands.registerCommand("c_IntakeNote", new IntakeNote());
        NamedCommands.registerCommand("c_ShootNoteIntoSpeaker", new ShootNoteIntoSpeaker(() -> false, () -> false, () -> false, () -> false));   
        NamedCommands.registerCommand("c_ShooterMotorsOn", new InstantCommand(() -> s_Shooter.setRPM(Constants.ShooterSpeed, Constants.ShooterSpeed))); 
        NamedCommands.registerCommand("c_AutoNotePickUpStrafe", new AutoNotePickUpStrafe()); 
        NamedCommands.registerCommand("c_AmpLiftToAmpPosition", new AmpLiftSetPOS(Constants.AmpLiftPOS_Amp));
        NamedCommands.registerCommand("c_NoteHandOff", new NoteHandOff());
        NamedCommands.registerCommand("c_RetractAmpLift", new AmpLiftSetPOS(Constants.AmpLiftPOS_Retracted));
        NamedCommands.registerCommand("c_AmpRollerShot", new AmpRollerJog(Constants.AmpRollerShootPercent).withTimeout(Constants.AmpRollerShootTimout));
        
        

        // Configure the button bindings
        configureButtonBindings();
        
        

      //Chooser Options for Path Planner  
      autoChooser.setDefaultOption("A_S1_1_2_3_67", new PathPlannerAuto("A_S1_1_2_3_67")); //A_1_45_2_3(add middle grab early)
      autoChooser.addOption("B_S3_3_2_1_45", new PathPlannerAuto("B_S3_3_2_1_45"));
      autoChooser.addOption("C_S1_1_4_56", new PathPlannerAuto("C_S1_1_4_56")); //C_3_2_45_1(add middle grab early)
      autoChooser.addOption("D_S4_87_76", new PathPlannerAuto("D_S4_87_76"));
      autoChooser.addOption("E_S2_2_56_67", new PathPlannerAuto("E_S2_2_56_67"));
      autoChooser.addOption("F_S3_3_2_56", new PathPlannerAuto("F_S3_3_2_56"));
      autoChooser.addOption("G_S4", new PathPlannerAuto("G_S4"));
      autoChooser.addOption("H_S3_3_2_1_45", new PathPlannerAuto("H_S3_3_2_1_45"));
      SmartDashboard.putData("Autonomous Program", autoChooser);
      //not sure if this right or work
      // some weird ask brett or walker bouts it
    }

    
    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */


        //new JoystickButton(rightJoystick,2).onTrue(new InstantCommand(() -> s_Swerve.setFieldPosition(0,0)));
         new JoystickButton(rightJoystick,1).whileTrue(new ShootNoteIntoSpeaker(() -> OP_Panel.getRawButton(4), 
                                                                                () -> OP_Panel.getRawButton(3),
                                                                                () -> OP_Panel.getRawButton(2),
                                                                                () -> OP_Panel.getRawButton(1))); //for match
        new JoystickButton(rightJoystick,2).whileTrue(new AmpRollerJog(Constants.AmpRollerShootPercent));
       new JoystickButton(rightJoystick, 3).whileTrue(new ShootNotePreset(Constants.PostShotPitch, Constants.PostShotRPM, 
                                                                                        Constants.PostShotredRot, Constants.PostShotblueRot)); 
        new JoystickButton(rightJoystick, 4).whileTrue(new PreSetLobShot(Constants.LobShotPitch, Constants.LobShotRPM, 
                                                                                        Constants.LobShotredRot, Constants.LobShotblueRot));
       new JoystickButton(rightJoystick, 5).onTrue(new InstantCommand(() -> s_Swerve.setAutoRotateConstants()));
        new JoystickButton(rightJoystick, 6).onTrue(new InstantCommand(() -> s_Shooter.setAutoPitchConstants()));
        new JoystickButton(rightJoystick, 7).onTrue(new InstantCommand(() -> s_Shooter.reapplyConfigs()));
         new JoystickButton(rightJoystick, 9).onTrue(new InstantCommand(() -> s_Climber.reapplyConfigs()));
        new JoystickButton(rightJoystick, 10).onTrue(new InstantCommand(() -> s_AmpLift.reapplyConfigs()));
        //new JoystickButton(rightJoystick, 8).whileTrue(new PitchJog(Constants.pitchJogSpeed));
       //new JoystickButton(rightJoystick, 9).whileTrue(new PitchJog(-Constants.pitchJogSpeed));
         new JoystickButton(rightJoystick,11).onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        new JoystickButton(rightJoystick,13).whileTrue(new ShooterCalibration());
        new JoystickButton(rightJoystick,15).onTrue(new ClimberHome());
        new JoystickButton(rightJoystick,16).onTrue(new AmpLiftHome());

      //Jog
        new POVButton(rightJoystick, 0).whileTrue(new ClimberJog(Constants.ClimberJogPercent));
        new POVButton(rightJoystick, 45).whileTrue(new ClimberJog(Constants.ClimberJogPercent));
        new POVButton(rightJoystick, 315).whileTrue(new ClimberJog(Constants.ClimberJogPercent));
        new POVButton(rightJoystick, 180).whileTrue(new ClimberJog(-Constants.ClimberJogPercent));
        new POVButton(rightJoystick, 225).whileTrue(new ClimberJog(-Constants.ClimberJogPercent));
        new POVButton(rightJoystick, 135).whileTrue(new ClimberJog(-Constants.ClimberJogPercent));

  
        //new JoystickButton(OP_Panel, 14).whileTrue(new ShootSpeed());


        //new JoystickButton(rightJoystick, 1).whileTrue(new ShooterCalibration()); //for calibration

        
        //use for finding pitch constants
 

        new JoystickButton(leftJoystick,1).onTrue(new IntakeNote().withTimeout(3));
        new JoystickButton(leftJoystick,2).whileTrue(new EjectIntake());
        // new JoystickButton(leftJoystick, 2).whileTrue(new unlatchBalancer());
        
        // Programming testing buttons
        new JoystickButton(leftJoystick, 7).whileTrue(new RotateToAngle(0));
        new JoystickButton(leftJoystick, 8).whileTrue(new RotateToAngle(90));
        new JoystickButton(leftJoystick, 9).whileTrue(new RotateToAngle(180));
        new JoystickButton(leftJoystick, 10).whileTrue(new RotateToAngle((-90)));
        new JoystickButton(leftJoystick,11).onTrue(new InstantCommand(() -> s_Shooter.setPitch(5)));
        new JoystickButton(leftJoystick,12).onTrue(new InstantCommand(() -> s_Shooter.setPitch(30)));
        new JoystickButton(leftJoystick,13).onTrue(new InstantCommand(() -> s_Shooter.setPitch(55)));


        //Auto Pitch Calibration
        


        
        //new JoystickButton(OP_Panel,1).onTrue(new InstantCommand(() -> s_Shooter.setRPM(Constants.ShooterSpeed, Constants.ShooterSpeed)));
        // new JoystickButton(OP_Panel,2).onTrue(new InstantCommand(() -> s_Shooter.setRPM(Constants.IdleSpeed, Constants.IdleSpeed)));
        // new JoystickButton(OP_Panel,3).onTrue(new InstantCommand(() -> s_Shooter.setRPM(0,0)));

        //toggles
      //  new JoystickButton(OP_Panel,1).onTrue(new InstantCommand(() -> s_Shooter.setRPM(Constants.ShooterSpeed, Constants.ShooterSpeed)));
        new JoystickButton(OP_Panel,2).onTrue(new InstantCommand(() -> s_Shooter.setPitchManualMode(true)));
        new JoystickButton(OP_Panel,2).onFalse(new InstantCommand(() -> s_Shooter.setPitchManualMode(false)));
      //  new JoystickButton(OP_Panel,3).onTrue(new InstantCommand(() -> s_Shooter.setRPM(0,0)));

        //testing
    
         // new JoystickButton(OP_Panel, 6).onTrue(new AmpLiftSetPOS(500));
      
        //new JoystickButton(OP_Panel, 6).onTrue(new ClimberSetPOS_Climb(Constants.ClimberMiddlePos));
          //new JoystickButton(OP_Panel, 7).onTrue(new AmpLiftSetPOS(Constants.AmpLiftPOS_Trap));
        
        // Auto Climb
        new JoystickButton(OP_Panel, 5).onTrue(new AutoTrap());

        //Amp
        new JoystickButton(OP_Panel, 8).onTrue(new NoteHandOff());
        new JoystickButton(OP_Panel, 11).onTrue(new AmpLiftSetPOS(Constants.AmpLiftPOS_Amp));
        new JoystickButton(OP_Panel, 12).onTrue(new RetractAmpLift());

        //Manual climb and amp jog
        new JoystickButton(OP_Panel, 13).onTrue(new ClimberSetPOS_Climb(Constants.ClimberExtend));
        new JoystickButton(OP_Panel, 14).onTrue(new Climb());
        new JoystickButton(OP_Panel, 15).whileTrue(new AmpLiftJog(Constants.AmpLiftJogPercent));
        new JoystickButton(OP_Panel, 16).whileTrue(new AmpLiftJog(-Constants.AmpLiftJogPercent));
      
      
        // new JoystickButton(OP_Panel, 15).whileTrue(new ClimberJog(Constants.ClimberJogPercent));
        // new JoystickButton(OP_Panel, 16).whileTrue(new ClimberJog(-Constants.ClimberJogPercent));

        //new JoystickButton(OP_Panel, 14).onTrue(new NoteHandOff());



    }   

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        // An ExampleCommand will run in autonomous
       // return new exampleAuto(s_Swerve);
    }
}
