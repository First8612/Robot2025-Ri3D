// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.DigitalOutput;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link XCaliper}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
  private final Drivetrain m_swerve = new Drivetrain(); 
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Climber m_climber = new Climber();
  private final Arm m_arm = new Arm();
  private final TagLimelight m_tagLimelight = new TagLimelight();
  private final NoteLimelight m_noteLimelight = new NoteLimelight();
  private final LEDs m_leds = new LEDs();
  // private PoseEstimator m_poseEstimator;

  //Commands
  private final AimArm aimArmContinuous = new AimArm(m_arm, m_tagLimelight, true);
  private final AimArm aimArm = new AimArm(m_arm, m_tagLimelight, false);
  private final AimAndShoot aimAndShoot = new AimAndShoot(m_shooter, m_arm, m_tagLimelight);
  private final ArmDown armDown = new ArmDown(m_arm);
  private final AimRotation aimRotate = new AimRotation(m_swerve, m_tagLimelight);
  private final ShootSpeaker shootSpeaker = new ShootSpeaker(m_shooter);
  private final FeedAndShoot feedAndShoot = new FeedAndShoot(m_shooter, m_intake);
  private final IntakeAndFeed intakeAndFeed = new IntakeAndFeed(m_shooter, m_intake);
  private final StopSystems stopSystems = new StopSystems(m_shooter, m_intake, m_arm);
  private final ShooterPrep shooterPrep = new ShooterPrep(m_shooter);

  //Auton chooser initiation
  SendableChooser<Command> m_autonChooser;

  //LED relays
  private DigitalOutput m_greenRelay = new DigitalOutput(2);

  //Controllers
  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);

  //Buttons and axes
  private final int m_armAxis = XboxController.Axis.kRightY.value;
  private final JoystickButton m_climbUpButton = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_climbDownButton = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);
  private final JoystickButton m_shootSpeakerButton = new JoystickButton(m_operatorController, XboxController.Button.kX.value);
  private final JoystickButton m_shootAmpButton = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
  private final JoystickButton m_feedOnlyButton = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
  private final JoystickButton m_shootOnlyButton = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
  private final JoystickButton m_backfeedButton = new JoystickButton(m_operatorController, 8);
  private final POVButton m_shooterPrepButton  = new POVButton(m_operatorController, 90);
  private final POVButton m_armAutoAimAdjustUpButton = new POVButton(m_operatorController, 0);
  private final POVButton m_armAutoAimAdjustDownButton = new POVButton(m_operatorController, 180);
  private final JoystickButton m_aimButton = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton m_resetFieldRelativeButton = new JoystickButton(m_driverController, 7);
  private final JoystickButton m_precisionButton = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_noteAimButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);

  private boolean m_wasAimPressedBefore = false;
  private RotationAimController m_rotationAimController = new RotationAimController(m_tagLimelight);
  private NoteAimController m_noteRotationController = new NoteAimController(m_noteLimelight);
  private ArmAimHelper m_armAimHelper = new ArmAimHelper();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //Commands for PathPlanner
    NamedCommands.registerCommand("stopModules", new InstantCommand(() -> {m_swerve.stopModules();}));
    NamedCommands.registerCommand("stopSystems", stopSystems);
    NamedCommands.registerCommand("shootSpeaker", shootSpeaker);
    NamedCommands.registerCommand("shootAmp", new RunCommand(() -> {m_shooter.shootAmp();}).withTimeout(1.0));
    NamedCommands.registerCommand("aimArm", aimArm);
    NamedCommands.registerCommand("aimAndShoot", aimAndShoot);
    NamedCommands.registerCommand("aimRotate", aimRotate);
    NamedCommands.registerCommand("armDown", armDown);
    NamedCommands.registerCommand("intakeAndFeed", intakeAndFeed);
    NamedCommands.registerCommand("feedAndShoot", feedAndShoot);
    NamedCommands.registerCommand("prepShooter", shooterPrep);

    //Auton things
    final PathPlannerAuto m_pathplanner1 = new PathPlannerAuto("Three-Speaker Auto (Non-Amp)");
    final PathPlannerAuto m_pathplanner2 = new PathPlannerAuto("Three-Speaker Auto (Podium)");
    final PathPlannerAuto m_pathplanner3 = new PathPlannerAuto("Two-Speaker Auto");
    final PathPlannerAuto m_pathplanner4 = new PathPlannerAuto("Three-Speaker Auto (Amp Note)");
    // final PathPlannerAuto m_pathplanner5 = new PathPlannerAuto("Three-Speaker Auto (Center)");
    final PathPlannerAuto m_pathplanner6 = new PathPlannerAuto("Three-Speaker Auto (Amp Side)");
    final PathPlannerAuto m_pathplanner7 = new PathPlannerAuto("Disruption Auto");
    // final PathPlannerAuto m_pathplanner8 = new PathPlannerAuto("Two-Speaker Auto (Podium)");
    final PathPlannerAuto m_pathplanner9 = new PathPlannerAuto("Three-Speaker Auto (Under Stage)");
    // final PathPlannerAuto m_pathplanner10 = new PathPlannerAuto("Two-Speaker Auto (Non-Amp)");
    // final PathPlannerAuto m_pathplanner13 = new PathPlannerAuto("Two-Speaker Toss Auto");
    // final PathPlannerAuto m_pathplanner11 = new PathPlannerAuto("Two-Speaker Auto (Amp Side)");
    final PathPlannerAuto m_pathplanner12 = new PathPlannerAuto("Three-Speaker Auto (Podium, With Run)");
    final PathPlannerAuto m_pathplanner13 = new PathPlannerAuto("Four-Speaker Auto");




    //Auton chooser
    m_autonChooser = new SendableChooser<>();

    //Adding auton routines to chooser
    m_autonChooser.addOption("Two-Speaker Auto (Center)", m_pathplanner3);
    // m_autonChooser.addOption("Two-Speaker Auto (Center, Podium)", m_pathplanner8);
    m_autonChooser.addOption("Three-Speaker Auto (Non-Amp, Center)", m_pathplanner1);
    // m_autonChooser.addOption("Two-Speaker Auto (Non-Amp, Podium)", m_pathplanner10);
    // m_autonChooser.addOption("Two-Speaker Auto With Toss (Non-Amp, Center)", m_pathplanner13);
    m_autonChooser.addOption("Three-Speaker Auto (Center, Podium)", m_pathplanner2);
    m_autonChooser.addOption("Three-Speaker Auto (Center, Podium, With Run)", m_pathplanner12);
    m_autonChooser.addOption("Three-Speaker Auto (Center, Amp Note)", m_pathplanner4);
    // m_autonChooser.addOption("Three-Speaker Auto (Center, Center)", m_pathplanner5);
    m_autonChooser.addOption("Three-Speaker Auto (Center, Center, Under Stage)", m_pathplanner9);
    m_autonChooser.addOption("Four-Speaker Auto (Center, Both)", m_pathplanner13);
    // m_autonChooser.addOption("Two-Speaker Auto (Amp Side, Amp Note)", m_pathplanner11);
    m_autonChooser.addOption("Three-Speaker Auto (Amp Side, Center)", m_pathplanner6);
    m_autonChooser.addOption("Disruption Auto", m_pathplanner7);
    SmartDashboard.putData("Auton Chooser", m_autonChooser);

    SmartDashboard.putData("Rotation Aim Controller", m_rotationAimController.getInternalController());

    m_swerve.setDefaultCommand(new RunCommand(() -> {this.driveWithJoystick(true);}, m_swerve));
  }

  public void teleopInit() {
    m_intake.setDefaultCommand(
      new RunCommand(() -> {
        var intakeSpeed = (m_operatorController.getRawAxis(XboxController.Axis.kRightTrigger.value)
           - m_operatorController.getRawAxis(XboxController.Axis.kLeftTrigger.value));
        m_intake.intake(intakeSpeed);
      },
      m_intake));


    m_arm.setDefaultCommand(
        new RunCommand(() -> {
          m_arm.adjustAim(-m_operatorController.getRawAxis(m_armAxis) * 1.5);
        },
      m_arm));

    m_shooter.setDefaultCommand(new RunCommand(() -> {}, m_shooter));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_climbUpButton.whileTrue(new StartEndCommand(() -> {
      m_climber.climbUp(-1);
    },
    () -> {
      m_climber.stop();
    }, m_climber));

    m_climbDownButton.whileTrue(new StartEndCommand(() -> {
      m_climber.climbDown(1);
    },
    () -> {
      m_climber.stop();
    }, m_climber));
    /*
    m_shootAmpButton.whileTrue(new StartEndCommand(() -> {
      m_shooter.shootAmp();
    },
    () -> {
      m_shooter.stop();
    }));

    m_shooterPrepButton.whileTrue(shooterPrep);

    m_shootSpeakerButton.onTrue(shootSpeaker);

    m_feedOnlyButton.whileTrue(new StartEndCommand(() -> {
      m_shooter.feedOnly();
    },
    () -> {
      m_shooter.stop();
    }));

    m_shootOnlyButton.whileTrue(new StartEndCommand(() -> {
      m_shooter.shootSpeakerOnly();
    },
    () -> {
      m_shooter.stop();
    }));

     m_backfeedButton.whileTrue(new StartEndCommand(() -> {
      m_shooter.backfeed();
    },
    () -> {
      m_shooter.stop();
    }));
    */
     
    m_resetFieldRelativeButton.onTrue(new InstantCommand(() -> {
      m_swerve.resetFieldRelative();
    }));
    
    m_aimButton.whileTrue(aimArmContinuous);

    m_armAutoAimAdjustUpButton.onTrue(new InstantCommand(() -> {
      m_arm.autoAimAdjustUp();
    }));

    m_armAutoAimAdjustDownButton.onTrue(new InstantCommand(() -> {
      m_arm.autoAimAdjustDown();
    }));
  }

  public void autonomousPeriodic() {

  }

  public void autonomousInit() {
    m_swerve.straightenModules();
    m_shooter.setHasNotBackfed();
  }

  public void teleopPeriodic() {

  }

  private void driveWithJoystick(boolean fieldRelative) {
    double xSpeed = Math.pow(MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.02), 3);
    double ySpeed = Math.pow(MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.02), 3);

    double rotWODeadband = -m_driverController.getRightX();
    double rot = Math.pow(MathUtil.applyDeadband(rotWODeadband, 0.02), 3);

    double tagLimelight_tx = m_tagLimelight.getTargetX();
    double noteLimelight_tx = m_noteLimelight.getTargetX();

    if (m_precisionButton.getAsBoolean()) {
      var multiplier = 0.75;
      xSpeed *= multiplier;
      ySpeed *= multiplier;
      rot *= multiplier;
    }

    if (m_aimButton.getAsBoolean() && tagLimelight_tx != 0) {
      if (!m_wasAimPressedBefore) {
        m_rotationAimController.reset();
      }

      rot = m_rotationAimController.calculate();
    }

    if (m_noteAimButton.getAsBoolean() && noteLimelight_tx != 0) {
      rot = m_noteRotationController.calculate();
    }

    m_wasAimPressedBefore = m_aimButton.getAsBoolean();

    SmartDashboard.putNumber("Teleop rot", rot);
    
    m_swerve.drive(xSpeed, ySpeed, rot);
  }

  /**
   * Use this to pass the autonomous command to the main {@link XCaliper} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonChooser.getSelected();
  }

  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_tagLimelight.periodic();
    SmartDashboard.putBoolean("Have Note?", m_intake.hasNote());
    SmartDashboard.putData(CommandScheduler.getInstance());

    var alliance = DriverStation.getAlliance();
    if (m_intake.hasNote()) {
      m_leds.setGreen();
    } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      m_leds.setRed();
    } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
      m_leds.setBlue();
    }

    if (m_intake.hasNote()) {
      m_greenRelay.set(true);
    } else {
      m_greenRelay.set(false);
    }

    SmartDashboard.putNumber("Arm/Auto Aim Setpoint", m_armAimHelper.getArmSetpoint(m_tagLimelight).getFirst());
  }
}
