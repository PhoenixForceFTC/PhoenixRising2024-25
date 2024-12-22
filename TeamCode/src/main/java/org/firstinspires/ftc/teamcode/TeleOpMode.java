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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Teleop Mode", group = "Concept")
public class TeleOpMode extends LinearOpMode {

    private MecanumWheels mecanumWheels;    // for mecanumWheels

    DcMotor arm; // motor for arm
    Servo servoWJ; // servo for wrist joint
    CRServo servoAI; // servo for wrist joint

    // for arm
    static final double COUNTS_PER_MOTOR_REV = 3895.9; // setting for 43 RPM motor
    static final int CYCLE_MS = 50; // period of each cycle, set to 50 milliseconds
    int targetPosition = 0; // To store the current target position for arm

    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position for wrist joint
    double power = 0; // power for active intake

    static final double INCREMENT = 0.03;     // amount to slew wrist joint each CYCLE_MS cycle
    static final double MAX_POS = 1;     // Maximum rotational position (wrist joint)
    static final double MIN_POS = 0;       // Minimum rotational position (wrist joint)

    @Override
    public void runOpMode() {
        // mecanum wheels program
        mecanumWheels = new MecanumWheels(this);

        // map arm and servos
        arm = hardwareMap.get(DcMotor.class, "AR");
        servoWJ = hardwareMap.get(Servo.class, "WJ");
        servoAI = (CRServo) hardwareMap.get(CRServo.class, "AI");

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Teleop.");
        telemetry.update();
        waitForStart();

        // setup arm
        // Set arm direction, mode, and behavior
        arm.setDirection(DcMotor.Direction.FORWARD); // Adjust this as needed
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Define the target positions in encoder counts for arm
        double targetRotationLowBasket = 0.4; // Set appropriately
        double targetRotationSpecimen = 0.45;  // Set appropriately
        double targetPickUp = 0.72;            // Set appropriately
        double targetZero = 0;
        double targetEnter = 0.6;

        while (opModeIsActive()) {
            // move mecanum wheels
            mecanumWheels.move();

            // Check which button is pressed and set the target position of arm
            if (gamepad2.dpad_left) {
                targetPosition = (int) (COUNTS_PER_MOTOR_REV * targetRotationLowBasket);
            } else if (gamepad2.dpad_up) {
                targetPosition = (int) (COUNTS_PER_MOTOR_REV * targetRotationSpecimen);
            } else if (gamepad2.dpad_down) {
                targetPosition = (int) (COUNTS_PER_MOTOR_REV * targetPickUp);
            } else if (gamepad2.a) {
                targetPosition = (int) (COUNTS_PER_MOTOR_REV * targetZero);
            } else if (gamepad2.dpad_right) {
                targetPosition = (int) (COUNTS_PER_MOTOR_REV * targetEnter);
            }

            // Check if the arm is not moving or the target position has changed
            if (!arm.isBusy() || arm.getTargetPosition() != targetPosition) {
                arm.setTargetPosition(targetPosition);
                arm.setPower(1); // Move to the target position
            }

            // Stop the arm once it has reached the target position
            if (!arm.isBusy()) {
                arm.setPower(0.2); // Stop the motor
            }

            if (gamepad2.x) {                   // Moves AI (counter)clockwise
                power = 1;
            } else if (gamepad2.y) {                   // Stops AI
                power = 0;
            } else if (gamepad2.b) {                   // Moves AI (counter)clockwise
                power = -1;
            }

            // slew the servo, according to the rampUp (direction) variable.
            if (gamepad2.right_stick_x > 0.05) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT;
                if (position >= MAX_POS) {
                    position = MAX_POS;
                }
            } else if (gamepad2.right_stick_x < -0.05) {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT;
                if (position <= MIN_POS) {
                    position = MIN_POS;
                }
            } else if (gamepad2.right_stick_button) {
                position = (MAX_POS - MIN_POS) / 2;
            }

            // Set the servo to the new position and pause;
            servoWJ.setPosition(position);
            servoAI.setPower(power);

            sleep(CYCLE_MS); // Pause for 50 milliseconds
            idle();
        }
    }
}