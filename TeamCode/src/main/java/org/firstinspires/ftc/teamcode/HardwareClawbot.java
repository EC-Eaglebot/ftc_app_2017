/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareClawbot
{
    /* Public OpMode members. */
    public DcMotor  frontleftDrive  = null;
    public DcMotor  frontrightDrive = null;
    public DcMotor  backleftDrive   = null;
    public DcMotor  backrightDrive  = null;
    public DcMotor  liftDrive       = null;
    public Servo arm = null;
    public Servo rightClaw = null;
    public Servo leftClaw = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;
    public static final double standardSpeed = 0.5;
    //public static final double ARM_UP_POWER    =  0.45 ;
    //public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime runtime  = new ElapsedTime();

    /* Constructor */
    public HardwareClawbot(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontleftDrive  = hwMap.get(DcMotor.class, "frontleft_drive");
        frontrightDrive = hwMap.get(DcMotor.class, "frontright_drive");
        backleftDrive  = hwMap.get(DcMotor.class, "backleft_drive");
        backrightDrive = hwMap.get(DcMotor.class, "backright_drive");
        liftDrive = hwMap.get(DcMotor.class, "lift");

        arm            = hwMap.get(Servo.class,"arm");
        rightClaw      = hwMap.get(Servo.class,"claw_right");
        leftClaw       = hwMap.get(Servo.class,"claw_left");


        frontleftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontrightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backleftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
        liftDrive.setPower(0);
        //leftClaw.setPower(0);
        //rightClaw.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);
        arm.setPosition(MID_SERVO);
    }

    // Function "forward"
    // make robot move forward at specified speed
    void forward(double speed, double secondsToRun) {
        double end = runtime.seconds() + secondsToRun;
        frontleftDrive.setPower(speed);
        frontrightDrive.setPower(speed);
        backrightDrive.setPower(speed);
        backleftDrive.setPower(speed);
        while (end > runtime.seconds()){ } // wait
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backrightDrive.setPower(0);
        backleftDrive.setPower(0);
    }

    // Function "stopMoving"
    // make the movement motors on the robot turn off

    void stopMoving() {
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backrightDrive.setPower(0);
        backleftDrive.setPower(0);
    }

    void backward(double speed, double secondsToRun){
        this.forward(-speed, secondsToRun);
    }


    // Function "turnDegree"
    // make the robot turn a specified degree

    void turnDegree(double degree){
        double end = (degree / 0.5) + runtime.seconds();
        // since angle = angular velocity * time,
        // time = angle / velocity
        if (degree > 0) {
            frontrightDrive.setPower(-0.5);
            backrightDrive.setPower(-0.5);
        } else if (degree < 0) {
            frontleftDrive.setPower(-0.5);
            backleftDrive.setPower(-0.5);
        }

        while (end > runtime.seconds()) { } // let the runtime go

        frontrightDrive.setPower(0);
        backrightDrive.setPower(0);
        frontleftDrive.setPower(0);
        backleftDrive.setPower(0);
    }

    void liftDrive_up () {
       runtime.reset();
       while (runtime.seconds() < 5) {
           liftDrive.setPower(0.5);
       }
    }
    void liftDrive_down () {
        runtime.reset();
        while (runtime.seconds() < 5) {
            liftDrive.setPower(-0.5);
        }
    }
    void liftDrive_test () {
        liftDrive.setPower(0.5);
    }
    void turnAround() {
        turnDegree(180);
    }
    void turnRight(){
        turnDegree(90);
    }
    void turnleft(){
        turnDegree(-90);
    }

    void stopAndWait(double secondsToWait) {
        stopMoving();
        double endTime = runtime.seconds() + secondsToWait;
        while (runtime.seconds() < endTime) { }
    }

}



