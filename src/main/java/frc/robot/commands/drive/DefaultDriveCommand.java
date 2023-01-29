package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GameController;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final GameController gameController;

    /**
     * Creates a new ExampleCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public DefaultDriveCommand(GameController gameController, DriveSubsystem driveSubsystem) {

        this.gameController = gameController;
        this.driveSubsystem = driveSubsystem;

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

        // Use arcade drive
        // Left Y controls velocity and forward/back
        // Right X controls turning

        double speed      = gameController.getLeftY();
        double turn       = gameController.getRightX() * 3.0d / 4.0d; // Turn at a max 3/4 speed.

        double leftSpeed  = speed + turn;
        double rightSpeed = speed - turn;

        // If the magnitude of the speed + turn > 1.0, then the driver will lose some ability
        // to turn the robot. Maintain the ratio / differential of the turn amount when the
        // motor speeds exceed 1.
        if (leftSpeed > 1 || rightSpeed > 1) {
            if (leftSpeed > 1) {
                leftSpeed  = 1;
                rightSpeed = 1 - turn * 2;
            }
            else {
                leftSpeed  = 1 + turn * 2;
                rightSpeed = 1;
            }
        }
        else if (rightSpeed < -1 || leftSpeed < -1) {
            if (leftSpeed < -1) {
                leftSpeed  = -1;
                rightSpeed = -1 - turn * 2;
            }
            else {
                leftSpeed  = -1 + turn * 2;
                rightSpeed = -1;
            }
        }

        // Drive slowly unless one of the bumpers is pressed, then boost the speed to full.
        // This code should likely be removed at competition
        boolean boost = gameController.getRightBumper() || gameController.getLeftBumper();

        if (!boost) {
            // Drive at half speed unless there is a boost
            driveSubsystem.setMotorSpeeds(leftSpeed / 2, rightSpeed / 2);
        }
        else {
            driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("DefaultDriveCommand interrupted");
        }
        else {
            System.out.println("DefaultDriveCommand ended");
        }
    }

}
