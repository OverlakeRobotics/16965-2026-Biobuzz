package org.firstinspires.ftc.teamcode.code.helpers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.code.parts.Turret;
import org.firstinspires.ftc.teamcode.system.OdometryHolonomicDrivetrain;

import java.util.ArrayList;
import java.util.List;



@Config
public class AutoAligner {
    public static final double g = 386.08858; // in/s^2
    public static final double rhinoWheelRadius = 1.88976; // in
    // TODO: Tune these values
    public static final double motorTicksPerRev = 28d;
    public static double kSlip = 0;
    public static double hoodAngle = 0;
    public static double aprilAlignOffset = 0;
    public static double kSlipTurretRotationConstant = 0.01; // 0.01;
    public static double autoAlignBuffer = 5; // degrees
    public static double randomMultiplier = 1;
    public static double randomMultiplierPerp = 1;

    public static double farShooterTolerance = 80;
    public static double closeShooterTolerance = 150;

    public static double closeDist = 110;
    public static double closeTurretTolerance = 6;
    public static double farTurretTolerance = 2;

    public static double maxShooterVelocity = 2800;
    public static double goalX;
    public static double goalY;
    public static double redGoalX = 70;
    public static double redGoalY = -70;
    public static double blueGoalX = 70;
    public static double blueGoalY = 70;
    public static double aprilX;
    public static double aprilY;
    public static double goalDZ = 28 + 2;

    public static double launchDelay = 0;
    public double angleOffset = 0;
    private int targetAprilID;
    private int sideFlipMultiplier;
    private final OdometryHolonomicDrivetrain driveTrain;
    private final Turret turret;
    private final Limelight3A limelight;

    public boolean useShootMove = true;
    public boolean limelightEnabled = true;

    public double lastVelPar;
    public double lastVelPerp;

    // 1. kSlip, 2. hoodAngle, 3.
    public static class PointValues {
        public double x, y;
        public double[] v;

        public PointValues(double x, double y, double[] v) {
            this.x = x;
            this.y = y;
            this.v = v;
        }
    }

    private final PointValues[] interpolationPoints = {
//              new PointValues(-45, 0, new double[]{0.47, 50, 1.5}),
//              new PointValues(-54, 18, new double[]{0.47, 50, 1}),
//              new PointValues(-54, -18, new double[]{0.47, 50, 1}),
//              new PointValues(-63, 33, new double[]{0.47, 50, 3}),
            // NEW POINTS
              new PointValues(-63.3, 12, new double[]{0.473, 50, 4}),
              new PointValues(-60, -12, new double[]{0.47, 50, 1}),
              new PointValues(-60, -12, new double[]{0.47, 50, 1}),
              new PointValues(-60, -27, new double[]{0.46, 50, 2}),
              new PointValues(-57, 0, new double[]{0.47, 50, 3}),
            // OLD
              new PointValues(-63.3, 33, new double[]{0.473, 50, 4}),
//              new PointValues(-64, 33, new double[]{0.47, 50, 5}),
              new PointValues(-54, 18, new double[]{0.475, 50, 3.5 - 0.5}),
              new PointValues(-48, -0, new double[]{0.472, 50, 3 - 0.5}),
              new PointValues(-63.3, 0, new double[]{0.47, 50, 2.5}),
              new PointValues(-54, -18, new double[]{0.467, 50, 1.5}),
              new PointValues(-63.3, -30, new double[]{0.457, 50, 0.5}),


              // Close points
              new PointValues(0, 0, new double[]{0.5, 50, 0}),
              new PointValues(21, 21, new double[]{0.525, 50, 0}),
              new PointValues(33, 33, new double[]{0.51, 35, 0}),
              new PointValues(45, 45, new double[]{0.495, 23, 0}),
              new PointValues(7, 18, new double[]{0.51, 50, 0}),
              new PointValues(39, 15, new double[]{0.5, 40, -1}),
//              new PointValues(0, 0, new double[]{0.47, 45, 0}),
//              new PointValues(39, 39, new double[]{0.47, 25, 0}),
//              new PointValues(24, 24, new double[]{0.47, 35, 0}),
//              new PointValues(48, 0, new double[]{0.46, 37, -2}),
//              new PointValues(24, 0, new double[]{0.47, 40, 1}),
//              new PointValues(48, 24, new double[]{0.465, 28, -1}),
//              new PointValues(30, 42, new double[]{0.475, 27, 1}),
//              new PointValues(6, 18, new double[]{0.48, 42, 1}),
//              new PointValues(24, -24, new double[]{0.46, 45, -2}),
//              new PointValues(39, -39, new double[]{0.46, 45, -1}),
//              new PointValues(48, -24, new double[]{0.47, 45, -1}),

              // Far points
//              new PointValues(-54, 18, new double[]{0.455, 45, 3.5}),
//              new PointValues(-63, 33, new double[]{0.45, 45, 3}),
//              new PointValues(-48, 0, new double[]{0.46, 45, 1.5}),
//              new PointValues(-54, -18, new double[]{0.455, 45, 1}),
//              new PointValues(-63, -33, new double[]{0.445, 45, 1}),
//              new PointValues(-63, 0, new double[]{0.45, 45, 2})
    };

    public AutoAligner(OdometryHolonomicDrivetrain driveTrain, Turret turret, Limelight3A limelight, boolean isRed) {
        this.driveTrain = driveTrain;
        this.turret = turret;
        this.limelight = limelight;

        if (isRed) {
            setRed();
        } else {
            setBlue();
        }
    }

    public void setBlue() {
        goalX = blueGoalX;
        goalY = blueGoalY;
        aprilX = 60;
        aprilY = 54;
        targetAprilID = 20;
        sideFlipMultiplier = 1;
    }

    public void setRed() {
        goalX = redGoalX;
        goalY = redGoalY;
        aprilX = 60;
        aprilY = -54;
        targetAprilID = 24;
        sideFlipMultiplier = -1;
    }

    public double getDistanceToGoal() {
//        LLResult result = limelight.getLatestResult();
        Pose2D pos = driveTrain.getPosition();
//        if (result.isValid()) {
//            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
//            for (LLResultTypes.FiducialResult tag : aprilTags) {
//                if (tag.getFiducialId() == targetAprilID) {
//                    Position robotPose = tag.getRobotPoseFieldSpace().getPosition();
////                    Log.d("Limelight", "using limelight dist");
////                    if (Math.hypot(robotPose.x * 39.37 + pos.getX(DistanceUnit.INCH), robotPose.y * 39.37 + pos.getY(DistanceUnit.INCH)) > 30) {
////                        break;
////                    }
//                    return Math.hypot(goalY + robotPose.y * 39.37, goalX + robotPose.x * 39.37);
//                }
//            }
//        }
//        Log.d("Limelight", "Pinpoint dist");
        return Math.hypot(goalX - pos.getX(DistanceUnit.INCH), goalY - pos.getY(DistanceUnit.INCH));
    }

    public double getDistanceToApril() {
        Pose2D pos = driveTrain.getPosition();
        return Math.hypot(aprilX - pos.getX(DistanceUnit.INCH), aprilY - pos.getY(DistanceUnit.INCH));
    }

    public boolean readyToShoot() {
        double dist = getDistanceToGoal();
        double shooterTolerance = dist < closeDist ? closeShooterTolerance : farShooterTolerance;
        double turretTolerance = dist < closeDist ? closeTurretTolerance : farTurretTolerance;
        double shooterError = Math.abs(turret.getShooter1Velocity() - turret.getShooterTargetVelocity());
        double turretError = Math.abs(turret.getTurretTargetAngle() - turret.getTurretCurrentAngle());
//        Log.d("Tolerance", String.format("Dist: %.4f; Shooter: %f; Turret: %f", dist, shooterTolerance, turretTolerance));
        return (shooterError <= shooterTolerance) && (turretError <= turretTolerance);
    }

    public void resetPinpointPositionFromLimelight() {
        if (!limelightEnabled) return;
        Pose2D pos = driveTrain.getPosition();
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : aprilTags) {
                if (tag.getFiducialId() == targetAprilID) {
                    Position robotPose = tag.getRobotPoseFieldSpace().getPosition();
                    driveTrain.setPosition(new Pose2D(DistanceUnit.METER, -robotPose.x, -robotPose.y, AngleUnit.DEGREES, pos.getHeading(AngleUnit.DEGREES)));
                }
            }
        }
    }

    public double getRawTurretAutoAlignAngle() {
        Pose2D pos = driveTrain.getPosition();
        if (limelightEnabled) {
            double turretAngle = turret.getTurretCurrentAngle();
            LLResult result = limelight.getLatestResult();
            //        double pinpointAngle = normalize(180 + Math.toDegrees(Math.atan2(
            //                goalY - pos.getY(DistanceUnit.INCH),
            //                goalX - pos.getX(DistanceUnit.INCH)
            //        )) - pos.getHeading(AngleUnit.DEGREES));
            //        Log.d("Turret Debug", "Pinpoint wanted: " + pinpointAngle);
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
                for (LLResultTypes.FiducialResult tag : aprilTags) {
                    if (tag.getFiducialId() == targetAprilID) {
                        // Testing making it aim not directly at apriltag, doesnt completely work
                        //                    Position robotPose = tag.getRobotPoseFieldSpace().getPosition();
                        //                    Log.d("Limelight", "See tag");
                        //                    Log.d("Limelight", "Pose: " + robotPose);
                        ////                    driveTrain.setPosition(new Pose2D(DistanceUnit.METER, -robotPose.x, -robotPose.y, AngleUnit.DEGREES, pos.getHeading(AngleUnit.DEGREES)));
                        //                    if (Math.hypot(robotPose.x * 39.37 + pos.getX(DistanceUnit.INCH), robotPose.y * 39.37 + pos.getY(DistanceUnit.INCH)) > 30) {
                        ////                        Log.d("Limelight", "Pos off");
                        //                        break;
                        //                    }
                        //                    return normalize(180 + Math.toDegrees(Math.atan2(goalY + robotPose.y * 39.37, goalX + robotPose.x * 39.37)) - pos.getHeading(AngleUnit.DEGREES));

                        // Return limelight angle if it sees the tag
                        //                    double limelightAngle = turretAngle - tag.getTargetXDegrees();
                        //                    Log.d("Turret Debug", "Limelight wanted: " + limelightAngle);
                        //                    return limelightAngle;
//                        Log.d("Turret", "Using limelight");
                        return turretAngle - tag.getTargetXDegrees() + aprilAlignOffset + (useShootMove ? angleOffset : 0);
                    }
                }
            }
        }
//        Log.d("Turret", "Using pinpoint");
//        Log.d("Limelight", "Pinpoint fallback");/
        // Fallback on using pinpoint
        return normalize(180 + Math.toDegrees(Math.atan2(
                        goalY - pos.getY(DistanceUnit.INCH),
                        goalX - pos.getX(DistanceUnit.INCH)
                )) - pos.getHeading(AngleUnit.DEGREES));
//        return pinpointAngle;
    }

    public double getDrivetrainAutoAlignAngleNoTurret() {
        Pose2D pos = driveTrain.getPosition();
        if (limelightEnabled) {
            double drivetrainAngle = pos.getHeading(AngleUnit.DEGREES);
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
                for (LLResultTypes.FiducialResult tag : aprilTags) {
                    if (tag.getFiducialId() == targetAprilID) {
                        return drivetrainAngle - tag.getTargetXDegrees() + aprilAlignOffset + (useShootMove ? angleOffset : 0);
                    }
                }
            }
        }
        // Fallback on using pinpoint
        return normalize(180 + Math.toDegrees(Math.atan2(
                goalY - pos.getY(DistanceUnit.INCH),
                goalX - pos.getX(DistanceUnit.INCH)
        )));
    }

    public double getTurretAutoAlignAngle() {
        double rawAngle = getRawTurretAutoAlignAngle();
        return Range.clip(rawAngle, turret.MIN_ANGLE_LIMIT, turret.MAX_ANGLE_LIMIT);
    }

    public double getDrivetrainAutoAlignAngleWithTurret() {
        double rawTurretAngle = getRawTurretAutoAlignAngle();
        double drivetrainAngle = driveTrain.getPosition().getHeading(AngleUnit.DEGREES);
        if (rawTurretAngle >= turret.MIN_ANGLE_LIMIT && rawTurretAngle <= turret.MAX_ANGLE_LIMIT) {
            return drivetrainAngle;
        }
        if (rawTurretAngle < turret.MIN_ANGLE_LIMIT) {
            return normalize(drivetrainAngle + rawTurretAngle - turret.MIN_ANGLE_LIMIT - autoAlignBuffer);
        } else {
            return normalize(drivetrainAngle + rawTurretAngle - turret.MAX_ANGLE_LIMIT + autoAlignBuffer);
        }
    }

    public void updateInterpolation(double x, double y) {
        int dims = interpolationPoints[0].v.length;       // number of value components
        double[] weighted = new double[dims];
        double totalWeight = 0;
        double eps = 1e-9;

        for (PointValues p : interpolationPoints) {
            double dx = x - p.x;
            double dy = y - p.y * sideFlipMultiplier;
            double distSq = dx * dx + dy * dy;

            // Exact match
            if (distSq < 1e-12) {
                kSlip = p.v[0];
                hoodAngle = p.v[1];
                aprilAlignOffset = p.v[2] * sideFlipMultiplier;
            }

            double w = 1.0 / (distSq + eps); // weight = 1/d^2

            for (int i = 0; i < dims; i++) {
                weighted[i] += w * p.v[i];
            }
            totalWeight += w;
        }

        // Normalize by weight sum
        for (int i = 0; i < dims; i++) {
            weighted[i] /= totalWeight;
        }

        kSlip = weighted[0];
        hoodAngle = weighted[1];
        aprilAlignOffset = weighted[2] * sideFlipMultiplier;
    }

    private static double solveTimeForShot(
            double d, double z,
            double thetaRad,
            double velPar, double velPerp,
            double g,
            double tMin, double tMax)
    {
        final double sinT = Math.sin(thetaRad);
        final double cosT = Math.cos(thetaRad);

        if (Math.abs(cosT) < 1e-6) return Double.NaN;

        // Define f(t) we need to find roots for
        java.util.function.DoubleUnaryOperator f = (t) -> {
            if (t <= 0) return Double.NaN;

//            double V = (z + 0.5 * g * t * t) / (t * cosT);
//            double A = V * sinT;
//
//            double term = (d / t) - velPar;
//            return (A * A) - (velPerp * velPerp) - (term * term);
            return t * (Math.cos(thetaRad) / Math.sin(thetaRad)) * Math.hypot(velPerp, (d / t) - velPar) - (g * t * t / 2) - z;
        };

        // Log-spaced scan to find all sign-change brackets
        int N = 160; // small + robust
        double ratio = Math.pow(tMax / tMin, 1.0 / N);

        double tPrev = tMin;
        double fPrev = f.applyAsDouble(tPrev);
        List<double[]> brackets = new ArrayList<>();

        for (int i = 1; i <= N; i++) {
            double tCur = tPrev * ratio;
            double fCur = f.applyAsDouble(tCur);

            if (Double.isFinite(fPrev) && Double.isFinite(fCur)) {
                if (fPrev == 0.0) {
                    brackets.add(new double[]{tPrev, tPrev});
                } else if (fPrev * fCur < 0.0) {
                    brackets.add(new double[]{tPrev, tCur});
                }
            }

            tPrev = tCur;
            fPrev = fCur;
        }

        if (brackets.isEmpty()) return Double.NaN;

        // Bisection refine each bracket
        // keep the valid root with smallest required V
        double bestT = Double.POSITIVE_INFINITY;
//        double bestV = Double.POSITIVE_INFINITY;

        for (double[] br : brackets) {
            double rootT = (br[0] == br[1]) ? br[0] : bisectRoot(f, br[0], br[1], 1e-6, 80);
            if (!Double.isFinite(rootT) || rootT <= 0) continue;

            double V = (z + 0.5 * g * rootT * rootT) / (rootT * cosT);
            if (!(V > 0) || !Double.isFinite(V)) continue;

            double A = V * sinT;
            if (!Double.isFinite(A) || Math.abs(A) < 1e-9) continue;

            // Need abs(velPerp) <= abs(A) so angle offset works
            if (Math.abs(velPerp) > Math.abs(A) + 1e-6) continue;

            // Choose the root that minimizes required speed V
            if (rootT < bestT) {
//                bestV = V;
                bestT = rootT;
            }
        }

        return bestT;
    }

    private static double bisectRoot(
            java.util.function.DoubleUnaryOperator f,
            double lo, double hi,
            double tol, int maxIter)
    {
        double flo = f.applyAsDouble(lo);
        double fhi = f.applyAsDouble(hi);

        if (!Double.isFinite(flo) || !Double.isFinite(fhi)) return Double.NaN;

        if (flo * fhi > 0) return Double.NaN;

        for (int i = 0; i < maxIter; i++) {
            double mid = 0.5 * (lo + hi);
            double fmid = f.applyAsDouble(mid);
            if (!Double.isFinite(fmid)) return Double.NaN;

            if (Math.abs(fmid) < tol || (hi - lo) < tol) return mid;

            if (flo * fmid <= 0.0) {
                hi = mid;
                fhi = fmid;
            } else {
                lo = mid;
                flo = fmid;
            }
        }
        return 0.5 * (lo + hi);
    }

    // TODO: Check angle and velocity calculations
    // Distances should be passed in as inches due to FTC standard units
    // theta is the launch angle in degrees, where launching straight up is 0 degrees (hood flat) and straight forward is 90 degrees (hood vertical)
    // Returns -1 if no valid solution (i.e. not possible given the angle)
    public double getShooterVelocityFromAngle(double theta) {
        double horizontalDist = getDistanceToGoal();

        double adjustedKSlip = kSlip + (horizontalDist > closeDist ? kSlipTurretRotationConstant * Math.abs(turret.getTurretCurrentAngle()) / 180 : 0);
        // Convert theta to radians
        theta = Math.toRadians(theta);

        Pose2D pos = driveTrain.getPosition();

        double vx = driveTrain.getXVelocity();
        double vy = driveTrain.getYVelocity();

        double dx = goalX - pos.getX(DistanceUnit.INCH) + vx * launchDelay;
        double dy = goalY - pos.getY(DistanceUnit.INCH) + vy * launchDelay;
        double dist = Math.hypot(dx, dy);

        // towards/away from goal
        double velPar = 0;

        // sideways/strafing around goal
        double velPerp = 0;

        if (dist != 0) {
            double ux = dx / dist;
            double uy = dy / dist;

            velPar = vx * ux + vy * uy;
            velPerp = vy * ux - vx * uy;
//            Log.d("Move Launch", "velPar: " + velPar);
//            Log.d("Move Launch", "velPerp: " + velPerp);
        }

        lastVelPar = velPar;
        lastVelPerp = velPerp;

        double t = solveTimeForShot(
                horizontalDist,
                goalDZ,
                theta,
                velPar * randomMultiplier,
                velPerp * randomMultiplierPerp,
                g,
                0.02,
                4.0
        );
//        Log.d("Move Launch", "Expected Flight Time: " + t);
        if (!Double.isFinite(t) || t <= 0) return -1;

        double v = (goalDZ + (g * t * t) / 2) / (t * Math.cos(theta));
//        Log.d("Move Launch", "Wanted Exit Velocity: " + v);

        double t_test = solveTimeForShot(
                horizontalDist,
                goalDZ,
                theta,
                0,
                0,
                g,
                0.02,
                4.0
        );
        double v_test = (goalDZ + (g * t_test * t_test) / 2) / (t_test * Math.cos(theta));
//        double tps_test = (v_test * 60) / (2 * Math.PI * rhinoWheelRadius * adjustedKSlip) * motorTicksPerRev / 60;

        angleOffset = Math.toDegrees(Math.atan2(-velPerp, (horizontalDist / t) - velPar));
//        Log.d("Move Launch", "Angle Offset: " + angleOffset);

        // Calculate RPM from linear velocity
        double rpm = (v * 60) / (2 * Math.PI * rhinoWheelRadius * adjustedKSlip);
        // Scale to motor ticks per second
        // TODO: Check ticks/sec calculation
        double rawTPS = rpm * motorTicksPerRev / 60;
//        if (rawTPS > maxShooterVelocity) {
//            Log.d("Shooter Velocity", "raw vel: " + rawTPS);
//        }
//        Log.d("Shooter Velocity", "Wanted: " + Math.round(rawTPS) + "WantedStationary: " + Math.round(tps_test) + "Actual: " + turret.getShooterVelocity() + "Diff: " + Math.round(rawTPS - tps_test) + "AccDiff: " + Math.round(rawTPS - turret.getShooterVelocity()) + "Power: " + turret.getShooterPower());
//        Log.d("Shooter", String.format("Horizontal Dist: %.5f, K-Slip: %.5f, RPM: %f, RawTPS: %f, Hood Angle: %f", horizontalDist, adjustedKSlip, rpm, rawTPS, theta));
        return Range.clip(rawTPS, 0, maxShooterVelocity);
    }

    public double getOptimalHoodAngle() {
        return hoodAngle;
    }

    // Work in progress
    public double getHoodAngleFromVelocity(double v, boolean useNegSol) {
        // All inputs must be in inches and inches/second
        double x = getDistanceToGoal();
        double xSquared = x * x;
        double vSquared = v * v;
        double gTerm = g * xSquared / (2 * vSquared);

        double discriminant = xSquared - (4 * gTerm * (goalDZ + gTerm));

        if (discriminant < 0) {
            return Double.NaN;
        }

        double sol = useNegSol ? (x - Math.sqrt(discriminant)) / (2 * gTerm) : (x + Math.sqrt(discriminant)) / (2 * gTerm);

        return 90 - Math.toDegrees(Math.atan(sol));
    }

    public double getHoodAngleFromVelocity(double v) {
        return getHoodAngleFromVelocity(v, false);
    }

    // Work in progress
    public double getOptimalShooterVelocity() {
        return Math.min(900 + (getDistanceToGoal() / 156) * 900, 1800);
    }

    public void toggleLimelight() {
        limelightEnabled = !limelightEnabled;
    }

    private double normalize(double angle) {
        return (angle + 180) % 360 - 180;
    }
}
