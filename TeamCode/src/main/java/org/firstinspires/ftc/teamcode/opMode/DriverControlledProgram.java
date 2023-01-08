package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * A driver controlled program.
 * NOTE: This class will automatically stop itself two minutes after starting!
 * Usage:
 * This class should be extended by a single program in which you MUST:
 *  - Override #buildRobot()
 * and MAY:
 *  - Override #onStart()
 *  - Override #onUpdate()
 *  - Override #onStop()
 * @Author Jaxon Brown
 */
public abstract class DriverControlledProgram extends OpMode {
    public static Robot robot;
    /**
     * Build a robot. This should be overridden by your Program.
     * Construct your robot and make any necessary changes to the subsystems.
     * @return Robot this program controls.
     */
    protected abstract Robot buildRobot();

    /**
     * Called when the the program is started.
     */
    protected void onStart() {telemetry.setAutoClear(true);}

    /**
     * Called when the loop finishes.
     */
    protected void onUpdate() {}

    /**
     * Called when the robot is stopped.
     */
    protected void onStop() {}

    @Override
    public final void init() {
        robot = buildRobot();
        try {
            robot.init();
        } catch(Exception ex) {
            telemetry.addData("ERROR!!!", ex.getMessage());
        }
    }

    @Override
    public final void start() {
        onStart();
    }

    @Override
    public final void loop() {
        robot.driverControlledUpdate();
        onUpdate();
    }

    @Override
    public final void stop() {
//        timer.stopThread();
        onStop();
    }

    /**
     * Gets the robot. If you properly cached your subcomponents in buildRobot(), you probably don't need this.
     * @return
     */
    protected final Robot getRobot() {
        return robot;
    }

}
