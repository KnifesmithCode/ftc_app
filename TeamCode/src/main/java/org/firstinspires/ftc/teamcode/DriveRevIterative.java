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

import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/* *
 * Drive REV
 * Used for testing our robots
 * This program started as a basic iterative OpMode which uses tank steering
 * It also controls the grabber on our robot
 * It also turns on the torch (flashlight) on the robot controller phone so it is obvious when the robot is running
 * (but only if the option is enabled in code)
*/

@TeleOp(name = "Drive REV", group = "Manual")
//@Disabled

@SuppressWarnings("unused")
public class DriveRevIterative extends OpMode {

    //#region Global Variables

    //Use a global variable for torch control so that it's easy to toggle
    private boolean ACTIVATE_TORCH = false;

    //Default OpMode variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private DcMotor clawHeight = null;

    private Servo leftClawServo = null;
    private Servo rightClawServo = null;

    //Custom variables to enable speed control and torch control
    private float speedMultiplier = 1.0f;
    private Camera camera;
    private Camera.Parameters p;
    //#endregion

    //#region Initialization
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        clawHeight = hardwareMap.get(DcMotor.class, "clawMotor");

        leftClawServo  = hardwareMap.get(Servo.class, "leftClaw");
        rightClawServo = hardwareMap.get(Servo.class, "rightClaw");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftClawServo.setDirection(Servo.Direction.FORWARD);
        rightClawServo.setDirection(Servo.Direction.REVERSE);

        //Set encoder mode
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set zero power mode to brake
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Ready", "Init finished");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }
    //#endregion

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //Use a global variable for torch control so that it's easy to toggle
        if(ACTIVATE_TORCH) {
            //On the ZTE phones, this will occasionally throw an error
            //Use a try/catch statement so that the whole program doesn't crash
            //if the torch won't work
            try {
                //Turn on the torch when the robot is ready
                camera = Camera.open();
                p = camera.getParameters();
                p.setFlashMode(Camera.Parameters.FLASH_MODE_TORCH);
                camera.setParameters(p);
                camera.startPreview();
            } catch (RuntimeException e) {
                e.printStackTrace();
            }
        }

        runtime.reset();
    }

    // run until the end of the match (driver presses STOP)
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Setup a variable for the height of the claw to save for telemetry
        double clawPower = gamepad1.left_trigger - gamepad1.right_trigger;

        clawHeight.setPower(clawPower / 2);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        leftPower = gamepad1.left_stick_y * speedMultiplier;
        rightPower = gamepad1.right_stick_y * speedMultiplier;

        if (gamepad1.dpad_up && speedMultiplier + 0.0005 <= 1) {
            speedMultiplier += 0.0005;
        } else if (gamepad1.dpad_down && speedMultiplier - 0.0005 >= 0) {
            speedMultiplier -= 0.0005;
        }

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Set claw position based on which buttons are pressed on the gamepad
        if(gamepad1.a) {
            setClaw(1.0);
        } else if(gamepad1.b) {
            setClaw((double)0);
        } else if(gamepad1.y) {
            setClaw(0.75);
        }

        double leftPosition = leftClawServo.getPosition();
        double rightPosition = rightClawServo.getPosition();

        // Show the elapsed game time and wheel power
        // Also show the speed multiplier and servo positions
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "Left (%.2f), Right (%.2f)",
                leftPower, rightPower);
        telemetry.addData("Speed", Float.toString(speedMultiplier));
        telemetry.addData("Claw Pwr", "(%.2f)", clawPower);
        telemetry.addData("Servos", "Left (%.2f), Right (%.2f)",
                leftPosition, rightPosition);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //Use a global variable for torch control so that it's easy to toggle
        if(ACTIVATE_TORCH) {
            //On the ZTE phones, this will occasionally throw an error
            //Use a try/catch statement so that the whole program doesn't crash
            //if the torch won't work
            try {
                //Turn off the torch when the robot is done
                camera = Camera.open();
                p = camera.getParameters();
                p.setFlashMode(Camera.Parameters.FLASH_MODE_OFF);
                camera.setParameters(p);
                camera.stopPreview();
            } catch (RuntimeException e) {
                e.printStackTrace();
            }
        }
    }

    // Helper function so that opening and closing the claw is easier
    private void setClaw(Double position) {
        leftClawServo.setPosition(position);
        rightClawServo.setPosition(position);
    }
}