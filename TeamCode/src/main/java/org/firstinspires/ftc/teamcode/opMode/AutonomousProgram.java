package org.firstinspires.ftc.teamcode.opMode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * An Autonomous program.
 * NOTE: Unlike Driver-controlled programs, it does NOT track time remaining. The driver station does that.
 * Usage:
 * This class should be extended by an abstract class representing an autonomous program for the robot.
 * @Author Jaxon Brown
 */
public abstract class AutonomousProgram extends LinearOpMode {
    private Robot robot;

    /**
     * Build a robot. This should be overridden by your abstract class and there marked final.
     * Construct your robot and cache its subcomponents as protected for use in the actual programs themselves.
     * @return Robot this program controls.
     */
    protected abstract Robot buildRobot();

    /**
     * Allow these methods to be overridden in the Autonomous program.
     * From there, you can code just like a linear opmode.
     */

    /**
     * The main method is for code after start is pressed
     * The preMain method is for code during the init phase
     */
    public abstract void preMain();
    public abstract void main();

    @Override
    public final void runOpMode() throws InterruptedException {
        robot = buildRobot();
        telemetry.setAutoClear(false);

        try {
            robot.init();
        } catch(Exception ex) {
            telemetry.addData("ERROR!!!", ex.getMessage());
        }

        try {
            preMain();
        } catch(Exception ex) {
            telemetry.addData("ERROR!!!", ex.getMessage());
            ex.printStackTrace();
        }

        waitForStart();

        try {
            idle();
        } catch(Exception ex) {
            telemetry.addData("ERROR!!!", ex.getMessage());
            ex.printStackTrace();
        }

        try {
            main();
        } catch(Exception ex) {
            telemetry.addData("ERROR!!!", ex.getMessage());
        }

    }

    /**
     * Gets the robot. If you properly cached your subcomponents in buildRobot(), you probably don't need this.
     * @return the robot.
     */
    protected final Robot getRobot() {
        return robot;
    }


}
