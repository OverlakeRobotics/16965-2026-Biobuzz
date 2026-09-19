package org.firstinspires.ftc.teamcode.system;


import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


// This class contains basic movement for robots with holonomic movement (Mecanum or Omni)
// It includes functions that set velocity to the motors, set target positions to the motors, and
// stop the robot.
// Positive heading is to the left, negative heading is to the right.
@Config
public class BasicHolonomicDrivetrain {
    public static final double MAX_STOP_VELOCITY = 1e-2;
    public static final int MAX_VELOCITY = 2800;
    public static final double FORWARD_COUNTS_PER_INCH = 29.9447; // 29.77497; // 32.49;
    public static final double STRAFE_COUNTS_PER_INCH = 29.7545; // 33.153818; // 33.27;
    public static final double STRAFE_TO_FORWARD_RATIO = STRAFE_COUNTS_PER_INCH / FORWARD_COUNTS_PER_INCH;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    protected double forward;
    protected double strafe;
    protected double turn;
    protected int countsAffectedByTurn;

    protected DriveState currentDriveState;
    public enum DriveState {
        POSITION_DRIVE,
        VELOCITY_DRIVE,
        STOPPED

    }

    public BasicHolonomicDrivetrain(DcMotorEx backLeft, DcMotorEx backRight,
                                    DcMotorEx frontLeft, DcMotorEx frontRight) {
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;

        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);

        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        currentDriveState = DriveState.STOPPED;
    }

    // Behavior: Sets the velocity PIDF coefficients for the drivetrain motors.
    // Parameters:
    //      - double p: The proportional term.
    //      - double i: The integral term.
    //      - double d: The derivative term.
    //      - double f: The feedforward term.
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        backLeft.setVelocityPIDFCoefficients(p, i, d, f);
        backRight.setVelocityPIDFCoefficients(p, i, d, f);
        frontLeft.setVelocityPIDFCoefficients(p, i, d, f);
        frontRight.setVelocityPIDFCoefficients(p, i, d, f);
    }

    // Behavior: Sets the positional P coefficient for the drivetrain motors, which is only used
    //           for autonomous movements.
    // Parameters:
    //      - double p: The proportional term.
    public void setPositionP(double p) {
        backLeft.setPositionPIDFCoefficients(p);
        backRight.setPositionPIDFCoefficients(p);
        frontLeft.setPositionPIDFCoefficients(p);
        frontRight.setPositionPIDFCoefficients(p);
    }

    // Behavior: Sets the velocity of the drive motors to the given velocities.
    // Parameters:
    //      - double backLeftVelocity: The velocity for the back left motor.
    //      - double backRightVelocity: The velocity for the back right motor.
    //      - double frontLeftVelocity: The velocity for the front left motor.
    //      - double frontRightVelocity: The velocity for the front right motor.
    protected void setMotorVelocity(double backLeftVelocity, double backRightVelocity,
                               double frontLeftVelocity, double frontRightVelocity) {
        double max = Math.max(Math.max(Math.abs(frontLeftVelocity), Math.abs(frontRightVelocity)),
                Math.max(Math.abs(backLeftVelocity), Math.abs(backRightVelocity)));

        if (max > MAX_VELOCITY) {
            frontLeftVelocity *= MAX_VELOCITY / max;
            frontRightVelocity *= MAX_VELOCITY / max;
            backLeftVelocity *= MAX_VELOCITY / max;
            backRightVelocity *= MAX_VELOCITY / max;
        }

        backLeft.setVelocity(backLeftVelocity);
        backRight.setVelocity(backRightVelocity);
        frontLeft.setVelocity(frontLeftVelocity);
        frontRight.setVelocity(frontRightVelocity);
    }

    // Behavior: Sets the mode of all the motors to the given mode.
    // Parameters:
    //      - DcMotorEx.RunMode mode: The mode to set the motors to.
    protected void setMotorMode(DcMotorEx.RunMode mode) {
        if (backLeft.getMode() != mode) {
            backLeft.setMode(mode);
            backRight.setMode(mode);
            frontLeft.setMode(mode);
            frontRight.setMode(mode);
        }
    }

    // Behavior: Sets the drive velocity given a forward velocity, a strafe velocity, and a turn
    //           velocity. If velocity exceeds MAX_VELOCITY, it will scale all the velocities
    //           down until the highest velocity is MAX_VELOCITY to make sure the robot can still
    //           maneuver. The directions follow the unit circle.
    // Parameters:
    //      - double forward: The forward velocity. Positive for forwards and negative for backwards.
    //      - double strafe: The strafe velocity. Positive for left and negative for right.
    //      - double turn: The turn velocity. Positive for left and negative for right.
    protected void moveRobot(double forward, double strafe, double turn) {
        double bl = forward + strafe - turn;
        double br = forward - strafe + turn;
        double fl = forward - strafe - turn;
        double fr = forward + strafe + turn;

        setMotorVelocity(bl, br, fl, fr);
    }

    // TODO: The stopped behavior here is a little weird, because it just sets a position drive, but then
    //       the position drive sets it to stopped kinda? I'd check this out and maybe clean it up.
    // Behavior: Drives the robot based on the current state and velocities. This should be called
    //           every loop. The possible states are STOPPED, which sets the velocity to 0,
    //           VELOCITY_DRIVE, which drives based on the current strafe, forward, and turn velocities,
    //           and POSITION_DRIVE, which drives the motors to the current set position.
    public void drive() {
        switch (currentDriveState) {
            case STOPPED:
                setPositionDrive(getBackLeftPosition(), getBackRightPosition(), getFrontLeftPosition(), getFrontRightPosition());
                break;

            case VELOCITY_DRIVE:
                if (Math.abs(forward) <= MAX_STOP_VELOCITY && Math.abs(strafe) <= MAX_STOP_VELOCITY &&
                        Math.abs(turn) <= MAX_STOP_VELOCITY) {
                    setDriveState(DriveState.STOPPED);
                }
                moveRobot(forward, strafe, turn);
                break;

            case POSITION_DRIVE:
                if (!isDriving()) {
                    setDriveState(DriveState.STOPPED);
                    break;
                }
                double backLeftDif = (getBackLeftTarget() - getBackLeftPosition());
                double backRightDif = (getBackRightTarget() - getBackRightPosition());
                double frontLeftDif = (getFrontLeftTarget() - getFrontLeftPosition());
                double frontRightDif = (getFrontRightTarget() - getFrontRightPosition());

                double max = Math.max(Math.max(Math.abs(backLeftDif), Math.abs(backRightDif)),
                        Math.max(Math.abs(frontLeftDif), Math.abs(frontRightDif)));
                backLeftDif /= max;
                backRightDif /= max;
                frontLeftDif /= max;
                frontRightDif /= max;

                double velocity = getPositionDriveVelocity();

                setMotorVelocity(velocity * backLeftDif, velocity * backRightDif,
                        velocity * frontLeftDif, velocity * frontRightDif);
                break;
        }
    }

    // Behavior: Gets the wanted velocity of the robot for position driving. Currently just returns
    //           forward but can be changed/overridden to implement something like a slowdown.
    // Returns: The wanted velocity of the robot in ticks/s.
    protected double getPositionDriveVelocity() {
        return forward;
    }

    // Behavior: Sets the forward, strafe, and turn velocities of the robot to the given values.
    // Parameters:
    //      - double forward: The given forward velocity.
    //      - double strafe: The given strafe velocity.
    //      - double turn: The given turn velocity.
    public void setVelocityDrive(double forward, double strafe, double turn) {
        if (Math.abs(forward) <= MAX_STOP_VELOCITY && Math.abs(strafe) <= MAX_STOP_VELOCITY &&
                Math.abs(turn) <= MAX_STOP_VELOCITY) {
            setDriveState(DriveState.STOPPED);
            return;
        }
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        setDriveState(DriveState.VELOCITY_DRIVE);
    }

    // Behavior: Sets the position for all the motors and the velocity to drive to those positions.
    // Parameters:
    //      - int backLeftTarget: The target for the back left motor in counts.
    //      - int backRightTarget: The target for the back right motor in counts.
    //      - int frontLeftTarget: The target for the front left motor in counts.
    //      - int frontRightTarget: The target for the front right motor in counts.
    public void setPositionDrive(int backLeftTarget, int backRightTarget,
                                 int frontLeftTarget, int frontRightTarget) {
        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);
        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        setDriveState(DriveState.POSITION_DRIVE);
    }

    // Behavior: An overloaded method of setPositionDrive that sets the position of the motors
    //           based on given forward, strafe, and turn count values. Directions follow the
    //           unit circle.
    // Parameters:
    //      - double forward: The number of forward counts to move. It is a double so it is not
    //                        truncated before all the calculations are complete.
    //      - double strafe: The number of strafe counts to move. It is a double so it is not
    //                        truncated before all the calculations are complete.
    //      - double turn: The number of turn counts to move. It is a double so it is not
    //                        truncated before all the calculations are complete.
    public void setPositionDrive(double forward, double strafe, double turn) {
        setPositionDrive(backLeft.getCurrentPosition() + (int)(forward + strafe - turn),
                backRight.getCurrentPosition() + (int)(forward - strafe + turn),
                frontLeft.getCurrentPosition() + (int)(forward - strafe - turn),
                frontRight.getCurrentPosition() + (int)(forward + strafe + turn));
        countsAffectedByTurn = (int)turn;
    }

    // Behavior: An overloaded method of setPositionDrive that sets the motor targets based on
    //           a direction (relative to the robot), a distance (in counts), and a turn (in counts).
    // Parameters:
    //      - int distance: The distance to drive, in counts.
    //      - double direction: The direction relative to the robot to drive in, in degrees.
    //                          Forward is 0 degrees, left is positive, right is negative.
    public void setPositionDrive(int distance, double direction, double turn) {
        double forwardCounts = distance * Math.cos(Math.toRadians(direction));
        double strafeCounts = distance * Math.sin(Math.toRadians(direction)) * STRAFE_TO_FORWARD_RATIO;
        setPositionDrive(forwardCounts, strafeCounts, turn);
    }

    public void setPositionDrive(int distance, double direction) {
        setPositionDrive(distance, direction, 0);
    }

    // Behavior: Gets the number of counts the robot needs to move forward to finish the current
    //           position drive.
    // Returns: The number of forward counts remaining in the current position drive as an int.
    public int getForwardCountsLeft() {
        if (currentDriveState == DriveState.POSITION_DRIVE) {
            return ((getBackLeftTarget() + countsAffectedByTurn - getBackLeftPosition()) +
                    (getBackRightTarget() - countsAffectedByTurn - getBackRightPosition()) +
                    (getFrontLeftTarget() + countsAffectedByTurn - getFrontLeftPosition()) +
                    (getFrontRightTarget() - countsAffectedByTurn - getFrontRightPosition())) / 4;
        }
        return 0;
    }

    // Behavior: Gets the number of counts the robot needs to strafe to finish the current
    //           position drive.
    // Returns: The number of strafe counts remaining in the current position drive as an int.
    public int getStrafeCountsLeft() {
        if (currentDriveState == DriveState.POSITION_DRIVE) {
            return ((getBackLeftTarget() + countsAffectedByTurn - getBackLeftPosition()) -
                    (getBackRightTarget() - countsAffectedByTurn - getBackRightPosition()) -
                    (getFrontLeftTarget() + countsAffectedByTurn - getFrontLeftPosition()) +
                    (getFrontRightTarget() - countsAffectedByTurn - getFrontRightPosition())) / 4;
        }
        return 0;
    }

    // Behavior: Gets the number of counts the robot needs to turn to finish the current
    //           position drive.
    // Returns: The number of turn counts remaining in the current position drive as an int.
    public int getTurnCountsLeft() {
        if (currentDriveState == DriveState.POSITION_DRIVE) {
            return ((getBackLeftTarget() - getBackLeftPosition()) -
                    (getBackRightTarget() - getBackRightPosition()) +
                    (getFrontLeftTarget() - getFrontLeftPosition()) -
                    (getFrontRightTarget() - getFrontRightPosition())) / 4;
        }
        return 0;
    }

    // Behavior: Gets the distance from the destination left in the current position drive, in
    //           counts.
    // Returns: A double containing the distance to the destination in counts.
    public int getPositionDriveDistanceLeft() {
        return (int)Math.hypot(getForwardCountsLeft(), getStrafeCountsLeft() / STRAFE_TO_FORWARD_RATIO);
    }

    // Behavior: Stops the robot by setting the motor velocities to 0.
    public void stop() {
        forward = 0;
        strafe = 0;
        turn = 0;
        setDriveState(DriveState.STOPPED);
    }

    // Behavior: Sets the drive state of the robot.
    // Parameters:
    //      - DriveState driveState: The drive state to set the robot to.
    protected void setDriveState(DriveState driveState) {
        switch (driveState) {
            case POSITION_DRIVE:
                setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;

            case VELOCITY_DRIVE:
            case STOPPED:
                setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
        }
        currentDriveState = driveState;
    }

    // Behavior: Sets the velocity of the robot for position drive.
    // Parameters:
    //      - int velocity: The velocity to set the robot to, in counts per second.
    public void setVelocity(int velocity) {
        forward = velocity;
        strafe = 0;
        turn = 0;
    }

    // Behavior: Returns whether the robot is currently driving
    // Returns: A boolean that is true if the robot is currently driving, false otherwise.
    public boolean isDriving() {
        return frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy();
    }

    // Behavior: Gets the current drive state of the robot.
    // Returns: A DriveState object containing the current drive state.
    protected DriveState getDriveState(){
        return currentDriveState;
    }

    // Behavior: Gets the target position of the back left motor.
    // Exceptions: Throws an IllegalStateException if the current drive state is not POSITION_DRIVE
    // Returns: The back left motor target position as an integer in counts.
    public int getBackLeftTarget() throws IllegalStateException {
        if (currentDriveState != DriveState.POSITION_DRIVE) {
            throw new IllegalStateException(
                    "Must be in POSITION_DRIVE state to access motor target positions!");
        }
        return backLeft.getTargetPosition();
    }

    // Behavior: Gets the target position of the back left motor.
    // Exceptions: Throws an IllegalStateException if the current drive state is not POSITION_DRIVE
    // Returns: The back right motor target position as an integer in counts.
    public int getBackRightTarget() throws IllegalStateException {
        if (currentDriveState != DriveState.POSITION_DRIVE) {
            throw new IllegalStateException(
                    "Must be in POSITION_DRIVE state to access motor target positions!");
        }
        return backRight.getTargetPosition();
    }

    // Behavior: Gets the target position of the back left motor.
    // Exceptions: Throws an IllegalStateException if the current drive state is not POSITION_DRIVE
    // Returns: The front left motor target position as an integer in counts.
    public int getFrontLeftTarget() throws IllegalStateException {
        if (currentDriveState != DriveState.POSITION_DRIVE) {
            throw new IllegalStateException(
                    "Must be in POSITION_DRIVE state to access motor target positions!");
        }
        return frontLeft.getTargetPosition();
    }

    // Behavior: Gets the target position of the back left motor.
    // Exceptions: Throws an IllegalStateException if the current drive state is not POSITION_DRIVE
    // Returns: The front right motor target position as an integer in counts.
    public int getFrontRightTarget() throws IllegalStateException {
        if (currentDriveState != DriveState.POSITION_DRIVE) {
            throw new IllegalStateException(
                    "Must be in POSITION_DRIVE state to access motor target positions!");
        }
        return frontRight.getTargetPosition();
    }

    public int getBackLeftPosition() {
        return backLeft.getCurrentPosition();
    }

    public int getBackRightPosition() {
        return backRight.getCurrentPosition();
    }

    public int getFrontLeftPosition() {
        return frontLeft.getCurrentPosition();
    }

    public int getFrontRightPosition() {
        return frontRight.getCurrentPosition();
    }
}
