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

    // FIXME: Are there encoders
    // leftEncoder, rightEncoder?
    // If there is a NavX we could try to pull the distance estimate off the NavX

    private double         leftSpeed          = 0;
    private double         rightSpeed         = 0;

    private AHRS           navXGyro           = null;

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

        // Setting both encoders to 0
        resetEncoders();

        // Declaring a NavX gyro will start the automatic calibration.
        // Do not move the robot when powering on.
        navXGyro = new AHRS();
    }

    /**
     * Calibrate Gyro
     * <p>
     * This routine calibrates the gyro. The robot must not be moved during the
     * calibrate routine which lasts about 10 seconds
     */
    public void calibrateGyro() {

        gyroHeadingOffset = 0;
        navXGyro.calibrate();
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

        switch (gyroAxis) {
        case YAW:
            return navXGyro.getAngle();
        case PITCH:
            return navXGyro.getPitch();
        case ROLL:
            return navXGyro.getRoll();
        default:
            return 0;
        }
    }

    public double getPitch() {
        return getRawGyroAngle(GyroAxis.PITCH);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderCounts() {
        return (getLeftEncoder() + getRightEncoder()) / 2;
    }

    public double getEncoderDistanceInches() {

        // Note: the NavX can do a distance estimate if encoders are not available

        return getAverageEncoderCounts() * DriveConstants.INCHES_PER_ENCODER_COUNT;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public double getLeftEncoder() {

        // FIXME: Are there encoders?
        return 0; // leftEncoder.getPosition();
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public double getRightEncoder() {

        // FIXME: Are there encoders? What are they?
        return 0; // rightEncoder.getPosition();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        // rightEncoder.setPosition(0);
        // leftEncoder.setPosition(0);

        // FIXME: If using a NavX, pull the distance estimate off the NavX gyro.
        // For the reset routine, just store the current NavX value and
        // then return the delta from that value on all subsequent reads
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

        SmartDashboard.putNumber("Right Encoder", getRightEncoder());
        SmartDashboard.putNumber("Left Encoder", getLeftEncoder());

        SmartDashboard.putNumber("Distance (inches)", getEncoderDistanceInches());

        SmartDashboard.putData("Gyro", navXGyro);

        SmartDashboard.putNumber("Gyro Heading", getHeading());
        SmartDashboard.putNumber("Gyro Pitch", getPitch());
    }
}
