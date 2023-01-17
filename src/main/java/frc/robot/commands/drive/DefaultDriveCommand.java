package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final XboxController driverController;

    private final double         DRIVE_FILTER_VALUE = 0.075f;

    /**
     * Creates a new ExampleCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public DefaultDriveCommand(XboxController driverController, DriveSubsystem driveSubsystem) {

        this.driverController = driverController;
        this.driveSubsystem   = driveSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Filter out low input values to reduce drivetrain drift
        double  leftY      = (Math.abs(driverController.getLeftY()) < DRIVE_FILTER_VALUE) ? 0.0f : driverController.getLeftY();
        double  leftX      = (Math.abs(driverController.getLeftX()) < DRIVE_FILTER_VALUE) ? 0.0f : driverController.getLeftX();

        // Use Arcade drive where the left Y axis is the speed, and the right X axis is the turn
        // NOTE: the Y axis value coming from the controller is inverted - stick forward gives a
        // negative Y
        double  leftSpeed  = leftY * -1 + leftX;
        double  rightSpeed = leftY * -1 - leftX;

        // Drive slowly unless the right bumper is pressed for speed boost.
        // This is useful for training drivers at low speeds and would
        // likely be removed for competitions
        boolean boost      = false;

        if (driverController.getRightBumper()) {
            boost = true;
        }

        if (!boost) {
            // Drive at half speed unless there is a boost
            driveSubsystem.setMotorSpeeds(leftSpeed / 2, rightSpeed / 2);
        }
        else {
            driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
