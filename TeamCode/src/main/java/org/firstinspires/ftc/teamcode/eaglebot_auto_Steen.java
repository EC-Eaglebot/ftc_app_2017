package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
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

@Autonomous(name="Eaglebot: Steen", group="Eaglebot")
public class eaglebot_auto_Steen extends LinearOpMode {

    // Declare OpMode members.
    HardwareClawbot     robot   = new HardwareClawbot();
    private ElapsedTime     runtime = new ElapsedTime();



//this stuff gets the light sensor ready, taken from the sample SensorColor opMode
    NormalizedColorSensor colorSensor;
    float[] hsvValues = new float[3];
    final float values[] = hsvValues;

    // Get a reference to our sensor object.
   // colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

    // If possible, turn the light on in the beginning (it might already be on anyway,
    // we just make sure it is if we can).


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        double HUE_THRESHOLD = 500;
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        //turns the light on color sensor
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        robot.stopMoving();
        runtime.reset();

        //moves the arm to the down position and reads the
        robot.arm.setPosition(robot.MID_SERVO - .5);
        // Read the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);


        telemetry.addLine()
                .addData("H", "%.3f", hsvValues[0])
                .addData("S", "%.3f", hsvValues[1])
                .addData("V", "%.3f", hsvValues[2]);
        telemetry.update();
        sleep(3000);


        if (hsvValues[0] > HUE_THRESHOLD){
            robot.strafeLeft(.5,.2, runtime);
            telemetry.addLine("BLUES CLUES");
            telemetry.update();
            sleep(3000);
        }
        else {
            robot.strafeRight(.5 , .2, runtime);
            telemetry.addLine("REDRUM REDRUM");
            telemetry.update();
            sleep(3000);
        }
        robot.stopMoving();
        robot.forward(0.5, 1, runtime);
        robot.stopMoving();
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