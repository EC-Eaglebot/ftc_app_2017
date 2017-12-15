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




@Autonomous(name="Red Team Right Stone JEWEL ONLY", group="Competition")
public class auto_red_right extends LinearOpMode {

    // Declare OpMode members.
    HardwareClawbot     robot   = new HardwareClawbot();
    private ElapsedTime     runtime = new ElapsedTime();



//this stuff gets the light sensor ready, taken from the sample concept_SensorColor opMode
    //ColorSensor color_sensor;
    //float[] hsvValues = new float[3];
    //final float values[] = hsvValues;

    // Get a reference to our sensor object.
    //color_sensor = hardwareMap.ColorSensor.get("color_sensor");
    
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
       // double RED_FLOOR_THRESHOLD = 300;
        // double RED_CEILING_THRESHOLD = 60;
        double red_qualifier = 30;
        double arm_down = .9;
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //robot.stopMoving();
        //runtime.reset();

        //moves the arm to the down position
        robot.arm.setPosition(.5);
        //close the claw here before the robot ever moves
        robot.closeClaws(runtime);
        //pause for a small time to slow the arm down
        sleep(500);
        robot.arm.setPosition(arm_down);


//read the sensor
    runtime.reset();
      while (runtime.seconds() < 2) {
            telemetry.addLine()
                    .addData("R", "%d", robot.color.red())
                    .addData("G", "%d", robot.color.green())
                    .addData("B", "%d", robot.color.blue());
            telemetry.update();
        }


        double power = .5;
        double twitchtime = 2;

        // Phase I: Read jewel color, knock off correct one
        if (robot.color.red() < red_qualifier){
            robot.strafeLeft(power,twitchtime, runtime);
            telemetry.addLine("BLUES CLUES");
            telemetry.update();
            sleep(2000);
        } // if red
        else {
            robot.strafeRight(power,twitchtime, runtime);
            telemetry.addLine("REDRUM REDRUM");
            telemetry.update();
            sleep(2000);
        }

        power = 1;
        // Phase II: Grab glyph, turn toward glyph wall, and move toward it
        //robot.closeClaws(runtime);
        //robot.forward(power,2,runtime);
        //robot.turnRight(runtime);
        power = .25;
        //robot.forward(power,2.5,runtime);

        // End
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