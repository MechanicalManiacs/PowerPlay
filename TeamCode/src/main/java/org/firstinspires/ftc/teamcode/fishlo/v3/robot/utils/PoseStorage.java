package org.firstinspires.ftc.teamcode.fishlo.v3.robot.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class PoseStorage {
    public enum Alliance {
        NONE(new Vector2d()),
        RED(new Vector2d()),
        BLUE(new Vector2d());

        public Vector2d highJunctionPosition = new Vector2d();
        Alliance(Vector2d highJunctionPosition) {
            this.highJunctionPosition = highJunctionPosition;
        }
    }
    public static Pose2d currentPose = new Pose2d();
    public static Alliance alliance = Alliance.BLUE;
}
