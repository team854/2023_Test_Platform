package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    // The motors on the left side of the drive.
    private final TalonSRX leftPrimaryMotor   = new TalonSRX(DriveConstants.LEFT_MOTOR_PORT);
    private final TalonSRX leftFollowerMotor  = new TalonSRX(DriveConstants.LEFT_MOTOR_PORT + 1);

    // The motors on the right side of the drive.
    private final TalonSRX rightPrimaryMotor  = new TalonSRX(DriveConstants.RIGHT_MOTOR_PORT);
    private final TalonSRX rightFollowerMotor = new TalonSRX(DriveConstants.RIGHT_MOTOR_PORT + 1);

    private double         leftSpeed          = 0;
    private double         rightSpeed         = 0;

    private AHRS           navX               = new AHRS();
    private double         offsetX            = 0;
    private double         offsetY            = 0;

    private double         gyroHeadingOffset  = 0;

    private enum GyroAxis {
        YAW, PITCH, ROLL
    };

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        leftPrimaryMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);
        leftFollowerMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);

        leftPrimaryMotor.setNeutralMode(NeutralMode.Brake);
        leftFollowerMotor.setNeutralMode(NeutralMode.Brake);

        leftFollowerMotor.follow(leftPrimaryMotor);


        rightPrimaryMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);
        rightFollowerMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);

        rightPrimaryMotor.setNeutralMode(NeutralMode.Brake);
        rightFollowerMotor.setNeutralMode(NeutralMode.Brake);

        rightFollowerMotor.follow(rightPrimaryMotor);
    }

    /**
     * Calibrate Gyro
     * <p>
     * This routine calibrates the gyro. The robot must not be moved during the
     * calibrate routine which lasts about 10 seconds
     */
    public void calibrateGyro() {

        gyroHeadingOffset = 0;
        navX.calibrate();
    }

    /**
     * Reset Gyro
     * <p>
     * This routine resets the gyro angle to zero.
     * <p>
     * NOTE: This is not the same as calibrating the gyro.
     */
    public void resetGyroHeading() {
        setGyroHeading(0);
    }

    /**
     * Set Gyro Heading
     * <p>
     * This routine sets the gyro heading to a known value.
     */
    public void setGyroHeading(double heading) {

        // Clear the current offset.
        gyroHeadingOffset = 0;

        // Adjust the offset so that the heading is now the current heading.
        gyroHeadingOffset = getHeading() - heading;
    }

    /**
     * Gets the heading of the robot.
     *
     * @return heading in the range of 0 - 360 degrees
     */
    public double getHeading() {

        double gyroAngle = getRawGyroAngle(GyroAxis.YAW);

        // subtract the offset angle that was saved when the gyro
        // was last rest.
        gyroAngle -= gyroHeadingOffset;

        // The angle can be positive or negative and extends beyond 360 degrees.
        double heading = gyroAngle % 360.0;

        if (heading < 0) {
            heading += 360;
        }

        return heading;
    }

    private double getRawGyroAngle(GyroAxis gyroAxis) {

        // NOTE: Pitch and roll may be reversed depending on the orientation
        // of the NavX in the robot.
        switch (gyroAxis) {
        case YAW:
            return navX.getAngle();
        case PITCH:
            return navX.getPitch();
        case ROLL:
            return navX.getRoll();
        default:
            return 0;
        }
    }

    public double getPitch() {
        return getRawGyroAngle(GyroAxis.PITCH);
    }

    public double getRoll() {
        return getRawGyroAngle(GyroAxis.ROLL);
    }

    public double getDistanceCm() {

        // Use the NavX to get the distance of the robot travel in cm
        // NOTE: the NavX returns displacement in cm, so multiply the meters by 100.

        double x = (navX.getDisplacementX() - offsetX) * 100d;
        double y = (navX.getDisplacementY() - offsetY) * 100d;

        // distance travelled is the hypotenuse of x, y
        return Math.round(Math.sqrt(x * x + y * y));
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {

        // NOTE: For the NavX, capture the current reading and use that
        // to offset any future readings.

        offsetX = navX.getDisplacementX();
        offsetY = navX.getDisplacementY();
    }


    /**
     * Set the left and right speed of the primary and follower motors
     *
     * @param leftSpeed
     * @param rightSpeed
     */
    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {

        this.leftSpeed  = leftSpeed;
        this.rightSpeed = rightSpeed;

        leftPrimaryMotor.set(ControlMode.PercentOutput, leftSpeed);
        rightPrimaryMotor.set(ControlMode.PercentOutput, rightSpeed);

        // NOTE: The follower motors are set to follow the primary
        // motors
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Right Motor", rightSpeed);
        SmartDashboard.putNumber("Left  Motor", leftSpeed);

        SmartDashboard.putNumber("Distance (cm)", getDistanceCm());

        SmartDashboard.putData("Gyro", navX);

        SmartDashboard.putNumber("Gyro Heading", getHeading());
        SmartDashboard.putNumber("Gyro Pitch", getPitch());
        SmartDashboard.putNumber("Gyro Roll", getRoll());
    }
}
