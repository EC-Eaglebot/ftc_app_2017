package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
public class eaglebot_auto_Josh extends LinearOpMode {

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
        // When the start button is pressed, the rest of the code will execute.
        // You want to put the actions under here. At the end of the file are
        // functions that will allow the robot to perform certain actions.
        // If you want a new action, talk to Cassidy, Corey, or Mr. Steen
        // and we will give you a hand.
        // Right now the code is executing "Eaglebot: Center Ball". You'll want
        // to make the code between // START and // END whatever you want the
        // robot to do.

        // START

        // START



        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        {

        }
        robot.stop();

        // END

        // Below here is where the robot will stop and go to sleep. Make
        // sure you have the last line above as "robot.stop();" !

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
// TOOLBOX OF FUNCTIONS
// *********************
/*
    Totally end the robot's function:       robot.stop();
    Make the motors stop moving:            robot.stopMoving();
    Make the robot move forward:            robot.forward(<insert speed here>);
    Display a message to the screen:        telemetry.addData("Message","value");
    Update the screen for the message:      telemetry.update();
    Insert this after a robot.function:     runtime.reset();
    Determine how long the code has run:    runtime.seconds();


*/