package org.firstinspires.ftc.teamcode.system.DifferentialSwerve;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;

public class BasicDifferentialSwerve {
    public static final double MAX_STOP_VELOCITY = 1e-2;
    protected final DifferentialSwerveModule[] modules;
    protected double forward;
    protected double strafe;
    protected double turn;
    protected double maxVel;
    protected DriveState currentDriveState;
    public enum DriveState {
        POSITION_DRIVE,
        VELOCITY_DRIVE,
        STOPPED
    }

    public BasicDifferentialSwerve(DifferentialSwerveModule[] modules) {
        this.modules = modules;
    }

    private void setModuleStates(double[] wheelTangentialVelocities, double[] steeringAngles) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(wheelTangentialVelocities[i], steeringAngles[i]);
        }
    }

    private void stopModules() {
        for (DifferentialSwerveModule module : modules) {
            module.stop();
        }
    }

    private void updateModules() {
        for (DifferentialSwerveModule module : modules) {
            module.update();
        }
    }

    public void drive() {
        switch (currentDriveState) {
            case STOPPED:
                stopModules();
                break;

            case POSITION_DRIVE:
            case VELOCITY_DRIVE:
                double[] wheelTangentialVelocities = new double[modules.length];
                double[] steeringAngles = new double[modules.length];
                for (int i = 0; i < modules.length; i++) {
                    double turnRadPerSec = turn * Math.PI / 180.0;
                    double[] finalVelocityVector = {forward - turnRadPerSec * modules[i].getYOffsetIn(),
                                                    strafe + turnRadPerSec * modules[i].getXOffsetIn()};
                    wheelTangentialVelocities[i] = Math.hypot(finalVelocityVector[0], finalVelocityVector[1]);
                    steeringAngles[i] = Math.toDegrees(
                            Math.atan2(
                                    finalVelocityVector[1],
                                    finalVelocityVector[0]
                            )
                    );
                }
                setModuleStates(wheelTangentialVelocities, steeringAngles);
                break;
        }

        updateModules();
    }

    // forward and strafe are in in/s, turn is in deg/s
    public void setVelocityDrive(double forward, double strafe, double turn) {
        if (Math.abs(forward) <= MAX_STOP_VELOCITY && Math.abs(strafe) <= MAX_STOP_VELOCITY &&
                Math.abs(turn) <= MAX_STOP_VELOCITY) {
            currentDriveState = DriveState.STOPPED;
            return;
        }
        double magnitude = Math.hypot(forward, strafe);
        if (magnitude > maxVel) {
            double scale = maxVel / magnitude;
            forward *= scale;
            strafe *= scale;
        }
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        if (currentDriveState != DriveState.POSITION_DRIVE) {
            currentDriveState = DriveState.VELOCITY_DRIVE;
        }
    }

    public void setVelocity(double vel) {
        this.maxVel = vel;
    }

    public void resetPIDF() {
        for (DifferentialSwerveModule module : modules) {
            module.resetPIDF();
        }
    }

    public void updatePIDFValues() {
        for (DifferentialSwerveModule module : modules) {
            module.updatePIDFValues();
        }
    }
}
