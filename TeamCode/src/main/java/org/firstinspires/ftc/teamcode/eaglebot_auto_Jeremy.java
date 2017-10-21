package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Eaglebot: Center Ball", group="Eaglebot")
public class eaglebot_auto_Jeremy extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareSampleBot     robot   = new HardwareSampleBot();
    private ElapsedTime     runtime = new ElapsedTime();



    @Override
    public void runOpMode() {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.stopMoving();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 8)) {
            telemetry.addData("Stopped", runtime.seconds());
            telemetry.update();
        }
        robot.forward(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Moving Forward", runtime.seconds());
            telemetry.update();
        }
        robot.stop();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}