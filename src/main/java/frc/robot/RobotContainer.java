package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.AbsoluteDriveAdv;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // Use CommandPS4Controller for PS5 controller support with command-based methods
    final CommandPS4Controller driverController = new CommandPS4Controller(0);

    // The robot's subsystems and commands are defined here...

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the rotational velocity 
    // buttons are quick rotation positions to different ways to face
    // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(Constants.Subsytems.SWERVE_SUBSYSTEM,
        () -> -MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        driverController.triangle()::getAsBoolean,
        driverController.cross()::getAsBoolean,
        driverController.square()::getAsBoolean,
        driverController.circle()::getAsBoolean);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = Constants.Subsytems.SWERVE_SUBSYSTEM.driveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getRightX(),
        () -> driverController.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAngularVelocity = Constants.Subsytems.SWERVE_SUBSYSTEM.driveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getRightX() * -1);

    Command driveFieldOrientedDirectAngleSim = Constants.Subsytems.SWERVE_SUBSYSTEM.simDriveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getRawAxis(2));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings()
    {
        if (DriverStation.isTest())
        {
            driverController.circle().whileTrue(Constants.Subsytems.SWERVE_SUBSYSTEM.sysIdDriveMotorCommand());
            driverController.square().whileTrue(Commands.runOnce(Constants.Subsytems.SWERVE_SUBSYSTEM::lock, Constants.Subsytems.SWERVE_SUBSYSTEM).repeatedly());
            driverController.triangle().whileTrue(Constants.Subsytems.SWERVE_SUBSYSTEM.driveToDistanceCommand(1.0, 0.2));
            driverController.options().onTrue(Commands.runOnce(Constants.Subsytems.SWERVE_SUBSYSTEM::zeroGyro));
            driverController.share().whileTrue(Constants.Subsytems.SWERVE_SUBSYSTEM.centerModulesCommand());
            driverController.L1().onTrue(Commands.none());
            driverController.R1().onTrue(Commands.none());
            Constants.Subsytems.SWERVE_SUBSYSTEM.setDefaultCommand(
                !RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);
        }
        else
        {
            driverController.cross().onTrue(Commands.runOnce(Constants.Subsytems.SWERVE_SUBSYSTEM::zeroGyro));
            driverController.square().onTrue(Commands.runOnce(Constants.Subsytems.SWERVE_SUBSYSTEM::addFakeVisionReading));
            driverController.circle().whileTrue(
                Commands.deferredProxy(() -> Constants.Subsytems.SWERVE_SUBSYSTEM.driveToPose(
                    new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                ));
            driverController.triangle().whileTrue(Constants.Subsytems.SWERVE_SUBSYSTEM.aimAtSpeaker(2));
            driverController.options().whileTrue(Commands.none());
            driverController.share().whileTrue(Commands.none());
            driverController.L1().whileTrue(Commands.runOnce(Constants.Subsytems.SWERVE_SUBSYSTEM::lock, Constants.Subsytems.SWERVE_SUBSYSTEM).repeatedly());
            driverController.R1().onTrue(Commands.none());
            Constants.Subsytems.SWERVE_SUBSYSTEM.setDefaultCommand(
                !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return Constants.Subsytems.SWERVE_SUBSYSTEM.getAutonomousCommand("New Auto");
    }

    public void setDriveMode()
    {
        configureBindings();
    }

    public void setMotorBrake(boolean brake)
    {
        Constants.Subsytems.SWERVE_SUBSYSTEM.setMotorBrake(brake);
    }
}
