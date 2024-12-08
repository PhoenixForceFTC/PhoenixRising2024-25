package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Concept: Arm", group = "Concept")
public class ConceptArm extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 3895.9; // setting for 43 RPM motor
    static final int CYCLE_MS = 50; // period of each cycle, set to 50 milliseconds
    DcMotor motor;
    int targetPosition = 0; // To store the current target position

    @Override
    public void runOpMode() {
        // Initialize motor
        motor = hardwareMap.get(DcMotor.class, "AR");

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
        double targetRotationLowBasket = 0.4; // Set appropriately
        double targetRotationSpecimen = 0.45;  // Set appropriately
        double targetPickUp = 0.72;            // Set appropriately
        double targetZero = 0;
        double targetEnter = 0.55;

        // Main loop
        while (opModeIsActive()) {
            // Check which button is pressed and set the target position
            if (gamepad2.dpad_left) {
                targetPosition = (int) (COUNTS_PER_MOTOR_REV * targetRotationLowBasket);
            } else if (gamepad2.dpad_up) {
                targetPosition = (int) (COUNTS_PER_MOTOR_REV * targetRotationSpecimen);
            } else if (gamepad2.dpad_down) {
                targetPosition = (int) (COUNTS_PER_MOTOR_REV * targetPickUp);
            } else if (gamepad2.a){
                targetPosition = (int) (COUNTS_PER_MOTOR_REV * targetZero);
            } else if (gamepad2.dpad_right){
                targetPosition=(int) (COUNTS_PER_MOTOR_REV * targetEnter);
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
