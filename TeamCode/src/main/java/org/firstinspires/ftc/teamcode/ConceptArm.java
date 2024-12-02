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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a Robot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Concept: Arm", group = "Concept")
public class ConceptArm extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    static final double     COUNTS_PER_MOTOR_REV    = 537.7;

    // Define class members
    DcMotor motor;
    double  power   = 0;
    boolean rampUp  = true;


    @Override
    public void runOpMode() {

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        motor = hardwareMap.get(DcMotor.class, "AR");

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        waitForStart();

        motor.setDirection(DcMotor.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setPower(1);

       double targetRotationLowBasket = 0.45;

       double targetRotationSpecimen = 0.49;

       double targetPickUp = 0.65;

        int newArmPosition = motor.getCurrentPosition() +
                (int)(COUNTS_PER_MOTOR_REV * targetRotationLowBasket);

        motor.setTargetPosition(newArmPosition);


        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {

            if(gamepad1.x){
                 newArmPosition = (int)(COUNTS_PER_MOTOR_REV * targetRotationLowBasket);

                motor.setTargetPosition(newArmPosition);


                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if(gamepad1.y){
                newArmPosition = (int)(COUNTS_PER_MOTOR_REV * targetRotationSpecimen);

                motor.setTargetPosition(newArmPosition);

                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }else if(gamepad1.b){
                newArmPosition = (int)(COUNTS_PER_MOTOR_REV * targetPickUp);

                motor.setTargetPosition(newArmPosition);


                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            // Display the current value
            telemetry.addData("Target Position", newArmPosition);
            telemetry.addData("Current Position at",  "%7d",
                    motor.getCurrentPosition());
            telemetry.update();

            // Set the motor to the new power and pause;
            sleep(CYCLE_MS);
            idle();
        }

        // Turn off motor and signal done;
        motor.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
