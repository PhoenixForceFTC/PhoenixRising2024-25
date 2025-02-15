package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Concept: Slide", group = "Concept")
public class ConceptSlide extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 3895.9; // setting for 43 RPM motor
    static final int CYCLE_MS = 50; // period of each cycle, set to 50 milliseconds
    DcMotor motor;
    int targetPosition = 0; // To store the current target position

    @Override
    public void runOpMode() {
        // Initialize motor
        motor = hardwareMap.get(DcMotor.class, "SL");

        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors.");
        telemetry.update();
        waitForStart();

        // Set motor direction, mode, and behavior
        motor.setDirection(DcMotor.Direction.FORWARD); // Adjust this as needed
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        // Define the target positions in encoder counts
        double targetZero = 0;      // Set as needed
        double targetExtend = 0.6;  // Set as needed

        // Main loop
        while (opModeIsActive()) {
            // Check which button is pressed and set the target position
            if (gamepad2.left_bumper){
                targetPosition = (int) (COUNTS_PER_MOTOR_REV * targetZero);
            } else if (gamepad2.right_bumper){
                targetPosition=(int) (COUNTS_PER_MOTOR_REV * targetExtend);
            }

            // Check if the motor is not moving or the target position has changed
            if (!motor.isBusy() || motor.getTargetPosition() != targetPosition) {
                motor.setTargetPosition(targetPosition);
                motor.setPower(1); // Move to the target position
            }

            // Stop the motor once it has reached the target position
            if (!motor.isBusy()) {
                motor.setPower(0.2); // Stop the motor
            }

            // Display data for debugging
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.update();

            sleep(CYCLE_MS); // Pause for 50 milliseconds
            idle();
        }

        motor.setPower(0); // Stop the motor at the end
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
