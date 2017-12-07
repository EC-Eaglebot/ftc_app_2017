package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;




@Autonomous(name="Eaglebot: Steen", group="Eaglebot")
public class eaglebot_auto_Steen extends LinearOpMode {

    // Declare OpMode members.
    HardwareClawbot     robot   = new HardwareClawbot();
    private ElapsedTime     runtime = new ElapsedTime();



//this stuff gets the light sensor ready, taken from the sample SensorColor opMode
    //ColorSensor color_sensor;
    //float[] hsvValues = new float[3];
    //final float values[] = hsvValues;

    // Get a reference to our sensor object.
    //color_sensor = hardwareMap.ColorSensor.get("color_sensor");
    
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        double RED_FLOOR_THRESHOLD = 300;
        double RED_CEILING_THRESHOLD = 60;
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        robot.stopMoving();
        runtime.reset();

        //moves the arm to the down position and reads the
        robot.arm.setPosition(robot.MID_SERVO - .5);
        // Read the sensor
        //robot.color.red();   // Red channel value
        //robot.color_sensor.green(); // Green channel value
        //robot.color_sensor.blue();  // Blue channel value

        //robot.color_sensor.alpha(); // Total luminosity
        //robot.color_sensor.argb();  // Combined color value



        telemetry.addLine()
                .addData("R", "%.3f", robot.color.red())
                .addData("G", "%.3f", robot.color.green())
                .addData("B", "%.3f", robot.color.blue());
        telemetry.update();
        sleep(3000);

        // if blue
        if (robot.color.red() < RED_FLOOR_THRESHOLD || robot.color.red() > RED_CEILING_THRESHOLD ){
            robot.strafeLeft(.5,.2, runtime);
            telemetry.addLine("BLUES CLUES");
            telemetry.update();
            sleep(3000);
        } // if red
        else {
            robot.strafeRight(.5 , .2, runtime);
            telemetry.addLine("REDRUM REDRUM");
            telemetry.update();
            sleep(3000);
        }
        robot.stopMoving();
        //robot.forward(0.5, 1, runtime);
        //robot.stopMoving();
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