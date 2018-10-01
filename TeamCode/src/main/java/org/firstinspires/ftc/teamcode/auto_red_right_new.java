package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Red Team Right Stone JEWEL ONLY with edit", group="Competition")
public class auto_red_right_new extends LinearOpMode {

    // Declare OpMode members.
    HardwareClawbot     robot   = new HardwareClawbot();
    private ElapsedTime     runtime = new ElapsedTime();
    private boolean MoveOn = false;
    private int stop = 500;


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
        double red_qualifier = 140;
        double arm_down = .85;
        double arm_middlepos = 0.42;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //robot.stopMoving();
        //runtime.reset();
        robot.paddle.setPosition(arm_middlepos);
        robot.arm.setPosition(0);
        //close the claw here before the robot ever moves
        robot.closeClaws(runtime);
        //pause for a small time to slow the arm down
        sleep(1000);
        robot.arm.setPosition(.6);
        sleep(1000);
        robot.arm.setPosition(.7);
        sleep(1000);
        robot.arm.setPosition(.8);
        sleep(1000);
        robot.arm.setPosition(arm_down);
        MoveOn = true;


//read the sensor
    runtime.reset();
      while (runtime.seconds() < 3 && MoveOn) {
            telemetry.addLine()
                    .addData("R", "%d", robot.color.red())
                    .addData("G", "%d", robot.color.green())
                    .addData("B", "%d", robot.color.blue());
            telemetry.update();
        }


        double power = .5;
        double twitchtime = 1.5;

        // Phase I: Read jewel color, knock off correct one
        // if blue
        if (robot.color.red() < red_qualifier){
            robot.paddle.setPosition(1.0);
            telemetry.addLine("BIG BLUE");
            telemetry.update();
            sleep(2000);
        } // if red
        else {
            robot.paddle.setPosition(-1.0);
            telemetry.addLine("RED DEAD REDEMPTION");
            telemetry.update();
            sleep(2000);
        }
        robot.paddle.setPosition(arm_middlepos);
        robot.arm.setPosition(0.5);
        robot.arm.setPosition(0.2);
        robot.liftDrive.setPower(0.3);
       // runtime.reset();
       /* robot.forward(0.2, 1.4, runtime);
        sleep(stop);
        //runtime.reset();
        robot.spinRight(0.2, 1.1, runtime);
        sleep(stop);
        //runtime.reset();
        robot.forward(0.2, 1, runtime);
        sleep(stop);
        robot.spinRight(0.2, 1,runtime);
        sleep(stop);
        robot.forward(0.2,1, runtime);
        //runtime.reset();*/

       /* robot.spinRight(0.2, 1, runtime);
        //sleep(stop);
        //runtime.reset();
        robot.forward(0.2, 1, runtime);
        sleep(stop);
        //runtime.reset();
        robot.spinRight(0.05, 0.01, runtime);
        sleep(stop);

        // runtime.reset();
        robot.forward(0.2, 0.8, runtime);
        sleep(stop); */
        //runtime.reset();

       /* robot.openClaws(runtime); */

       //robot.forward(0.01, 0.5, runtime);

       /* robot.backward(0.1, 1.5, runtime); */

        power = 1;
        // Phase II: Grab glyph, turn toward glyph wall, and move toward it
        //robot.closeClaws(runtime);
        //robot.forward(power,2,runtime);
        //robot.turnRight(runtime);
        power = .25;
        //robot.forward(power,2.5,runtime);

        // End
        robot.arm.setPosition(.2);
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