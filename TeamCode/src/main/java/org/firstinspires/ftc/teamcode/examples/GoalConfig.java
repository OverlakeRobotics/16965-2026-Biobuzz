package org.firstinspires.ftc.teamcode.examples;


import com.acmerobotics.dashboard.config.Config;


// Global goal locations used by every shooter TeleOp (single shooter, dual shooter and dual turret,
// red and blue). Change them here or live in FTC Dashboard under "GoalConfig".
//
// Field coordinates are the Pinpoint's: inches, +x forward, +y left. Each alliance has two goals
// and the robot aims at whichever is closer.
@Config
public class GoalConfig {
    // TODO: Set these to the real goal locations. The defaults are placeholders: goal 1 is the old
    //  AutoAligner goal and goal 2 is its mirror across the field's y axis.
    public static double redGoal1X = 70;
    public static double redGoal1Y = -70;
    public static double redGoal2X = -70;
    public static double redGoal2Y = -70;
    public static double blueGoal1X = 70;
    public static double blueGoal1Y = 70;
    public static double blueGoal2X = -70;
    public static double blueGoal2Y = 70;

    // Height of the goal's scoring point above the shooter's exit point (inches). Same meaning as
    // AutoAligner.goalDZ.
    public static double goalHeight = 30;
}
