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
    private final Joystick rightJoystick = new Joystick(1);
    private final Joystick OP_Panel = new Joystick(2);
    /* Drive Controls */

    /* Driver Buttons */

    /* Subsystems */
    
    private static final PhotonVision s_PhotonVision =  new PhotonVision();
    public final static Swerve s_Swerve = new Swerve(s_PhotonVision);
    public final static Shooter s_Shooter = new Shooter();
    public final static Intake s_Intake = new Intake();
    public final static Limelight s_Limelight = new Limelight();

    
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
                        () -> false,
                        () -> rightJoystick.getRawButton(3)
                        ));

                        //changed drivers to joysticks and axis numbered
                        //deleted robotcentric and joystick thingy
                        // removed the (-) symbols in front of leftJoystick.getRawAxis(1),() -> leftJoystick.getRawAxis(0),() -> rightJoystick.getRawAxis(2),


        NamedCommands.registerCommand("c_IntakeNote", new IntakeNote());
        NamedCommands.registerCommand("c_ShootNoteIntoSpeaker", new ShootNoteIntoSpeaker());   
        NamedCommands.registerCommand("c_ShooterMotorsOn", new InstantCommand(() -> s_Shooter.setRPM(Constants.ShooterSpeed, Constants.ShooterSpeed))); 

        
        

        // Configure the button bindings
        configureButtonBindings();
        
        

      //Chooser Options for Path Planner  
      // autoChooser.setDefaultOption("Default", new PathPlannerAuto("Example 1"));
      autoChooser.setDefaultOption("Default", new PathPlannerAuto("Default"));
       autoChooser.addOption("auto 2", new PathPlannerAuto("Example 2")); //is this needed????
       autoChooser.addOption("Shoot Only", new PathPlannerAuto("Shoot Thing Yay"));
    //    autoChooser.addOption("Middle Shoot", new PathPlannerAuto("Middle shpoot"));
    //    autoChooser.addOption("Middle Shoot", new PathPlannerAuto("Middle shoot"));
    //    autoChooser.addOption("Middle Shoot Ony", new PathPlannerAuto("Wall Shoot Out"));
    //    autoChooser.addOption("Wall Shoot Out", new PathPlannerAuto("Post Shoot"));
       SmartDashboard.putData("Post Shoot", autoChooser);
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
        new JoystickButton(leftJoystick,2).onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        new JoystickButton(rightJoystick,2).onTrue(new InstantCommand(() -> s_Swerve.setFieldPosition(0,0)));
        //new JoystickButton(OP_Panel, 14).whileTrue(new ShootSpeed());

        new JoystickButton(leftJoystick,1).onTrue(new IntakeNote().withTimeout(3));
        //new JoystickButton(leftJoystick,1).whileTrue(new EjectIntake());

         new JoystickButton(rightJoystick,1).whileTrue(new ShootNoteIntoSpeaker()); //for match
        //new JoystickButton(rightJoystick, 1).whileTrue(new ShooterCalibration()); //for calibration
        

        new JoystickButton(OP_Panel,10).onTrue(new InstantCommand(() -> s_Shooter.reapplyConfigs()));
       // new JoystickButton(OP_Panel,5).onTrue(new InstantCommand(() -> s_Shooter.setPitch(5)));
        //new JoystickButton(OP_Panel,5).whileTrue(new ShooterCalibration());;
        //use for finding pitch constants
        // new JoystickButton(OP_Panel,6).onTrue(new InstantCommand(() -> s_Shooter.setPitch(15)));
        // new JoystickButton(OP_Panel,7).onTrue(new InstantCommand(() -> s_Shooter.setPitch(20)));
        // new JoystickButton(OP_Panel,8).onTrue(new InstantCommand(() -> s_Shooter.setPitch(30)));
        new JoystickButton(OP_Panel,1).onTrue(new InstantCommand(() -> s_Shooter.setRPM(Constants.ShooterSpeed, Constants.ShooterSpeed)));
        //P controller calibration
        // new JoystickButton(OP_Panel, 5).whileTrue(new RotateToAngle(0));
        // new JoystickButton(OP_Panel, 6).whileTrue(new RotateToAngle(90));
        // new JoystickButton(OP_Panel, 7).whileTrue(new RotateToAngle(180));
        // new JoystickButton(OP_Panel, 8).whileTrue(new RotateToAngle(270));
        new JoystickButton(OP_Panel, 5).whileTrue(new AutoNotePickUpStrafe());

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
