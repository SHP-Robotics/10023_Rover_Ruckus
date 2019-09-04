package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class BaseRobot extends OpMode {
    public DcMotor leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor, climbMotor, flipMotor, liftMotor, bucketMotor;
    public Servo marker_servo, intake_servo;
    public ElapsedTime timer = new ElapsedTime();


    @Override
    public void init() {
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftBackDriveMotor");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightBackDriveMotor");
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFrontDriveMotor");
        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFrontDriveMotor");
        climbMotor = hardwareMap.get(DcMotor.class, "climbMotor");

        flipMotor = hardwareMap.get(DcMotor.class, "flipMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        bucketMotor = hardwareMap.get(DcMotor.class, "bucketMotor");

        marker_servo = hardwareMap.get(Servo.class, "marker_servo");
        intake_servo = hardwareMap.get(Servo.class, "intake_servo");

        set_marker_servo(ConstantVariables.K_MARKER_SERVO_UP);
        set_intake_servo(ConstantVariables.K_INTAKE_SERVO_IN);
    }

    @Override
    public void start() {
        timer.reset();
        reset_drive_encoders();
        reset_climb_encoders();
        reset_intake_outtake_encoders();
    }

    @Override
    public void loop() {

        telemetry.addData("D00 Left Front Drive Motor Enc: ", get_left_front_drive_motor_enc());
        telemetry.addData("D01 Right Front Drive Motor Enc: ", get_right_front_drive_motor_enc());
        telemetry.addData("D02 Left Back Drive Motor Enc: ", get_left_back_drive_motor_enc());
        telemetry.addData("D03 Right Back Drive Motor Enc: ", get_right_back_drive_motor_enc());

        telemetry.addData("D04 Climb Motor Enc: ", get_climb_motor_enc());
        telemetry.addData("D05 Flip Motor Enc: ", get_flip_motor_enc());
        telemetry.addData("D06 Lift Motor Enc: ", get_lift_motor_enc());
        telemetry.addData("D07 Bucket Motor Enc: ", get_bucket_motor_enc());

        telemetry.addData("D08 Marker Servo Pos: ", marker_servo.getPosition());
        telemetry.addData("D09 Intake Servo Pos: ", intake_servo.getPosition());
    }

    public void climb(double power) {
        double speed = Range.clip(power, -1, 1);
        climbMotor.setPower(speed);
    }
    public void flip(double power) {
        double speed = Range.clip(power, -1, 1);
        flipMotor.setPower(speed);
    }

    public void lift(double power) {
        double speed = Range.clip(power, -1, 1);
        liftMotor.setPower(speed);
    }

    public void bucket(double power) {
        double speed = Range.clip(power, -1, 1);
        bucketMotor.setPower(speed);
    }

    public boolean auto_drive(double power, double inches) {
        double TARGET_ENC = ConstantVariables.K_PPIN_DRIVE * inches;
        telemetry.addData("Target_enc: ", TARGET_ENC);
        double left_speed = -power;
        double right_speed = power;
        /*double error = -get_left_front_drive_motor_enc() - get_right_front_drive_motor_enc();

        error /= ConstantVariables.K_DRIVE_ERROR_P;
        left_speed += error;
        right_speed -= error;*/

        left_speed = Range.clip(left_speed, -1, 1);
        right_speed = Range.clip(right_speed, -1, 1);
        leftFrontDriveMotor.setPower(left_speed);
        leftBackDriveMotor.setPower(left_speed);
        rightFrontDriveMotor.setPower(right_speed);
        rightBackDriveMotor.setPower(right_speed);

        if (Math.abs(get_right_front_drive_motor_enc()) >= TARGET_ENC) {
            leftFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
            return true;
        }
        return false;
    }

    /**
     * @param power:   the speed to turn at. Negative for left.
     * @param degrees: the number of degrees to turn.
     * @return Whether the target angle has been reached.
     */
    public boolean auto_turn(double power, double degrees) {
        double TARGET_ENC = Math.abs(ConstantVariables.K_PPDEG_DRIVE * degrees);
        telemetry.addData("D99 TURNING TO ENC: ", TARGET_ENC);

        double speed = Range.clip(power, -1, 1);
        leftFrontDriveMotor.setPower(-speed);
        leftBackDriveMotor.setPower(-speed);
        rightFrontDriveMotor.setPower(-speed);
        rightBackDriveMotor.setPower(-speed);

        if (Math.abs(get_right_front_drive_motor_enc()) >= TARGET_ENC) {
            leftFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
            return true;
        } else {
            return false;
        }
    }

    //positive for right, negative for left
    public boolean auto_mecanum(double power, double inches) {
        double TARGET_ENC = ConstantVariables.K_PPIN_DRIVE * inches;
        telemetry.addData("Target_enc: ", TARGET_ENC);

        double leftFrontPower = Range.clip(0 - power, -1.0, 1.0);
        double leftBackPower = Range.clip(0 + power, -1.0, 1.0);
        double rightFrontPower = Range.clip(0 - power, -1.0, 1.0);
        double rightBackPower = Range.clip(0 + power, -1.0, 1.0);

        leftFrontDriveMotor.setPower(leftFrontPower);
        leftBackDriveMotor.setPower(leftBackPower);
        rightFrontDriveMotor.setPower(rightFrontPower);
        rightBackDriveMotor.setPower(rightBackPower);

        if (Math.abs(get_right_front_drive_motor_enc()) >= TARGET_ENC) {
            leftFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
            return true;
        } else {
            return false;
        }
    }

    public void tankanum_drive(double rightPwr, double leftPwr, double lateralpwr) {
        rightPwr *= -1;

        double leftFrontPower = Range.clip(leftPwr - lateralpwr, -1.0, 1.0);
        double leftBackPower = Range.clip(leftPwr + lateralpwr, -1.0, 1.0);
        double rightFrontPower = Range.clip(rightPwr - lateralpwr, -1.0, 1.0);
        double rightBackPower = Range.clip(rightPwr + lateralpwr, -1.0, 1.0);

        leftFrontDriveMotor.setPower(leftFrontPower);
        leftBackDriveMotor.setPower(leftBackPower);
        rightFrontDriveMotor.setPower(rightFrontPower);
        rightBackDriveMotor.setPower(rightBackPower);
    }

    public void tank_drive(double leftPwr, double rightPwr) {
        double leftPower = Range.clip(leftPwr, -1.0, 1.0);
        double rightPower = Range.clip(rightPwr, -1.0, 1.0);

        leftFrontDriveMotor.setPower(leftPower);
        leftBackDriveMotor.setPower(leftPower);
        rightFrontDriveMotor.setPower(-rightPower);
        rightBackDriveMotor.setPower(-rightPower);
    }

    public void set_marker_servo(double pos) {
        double position = Range.clip(pos,0,1.0);
        marker_servo.setPosition(position);
    }

    public void set_intake_servo(double pos) {
        double position = Range.clip(pos,0,1.0);
        intake_servo.setPosition(position);
    }

    public void reset_drive_encoders() {
        leftFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void reset_climb_encoders() {
        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void reset_intake_outtake_encoders() {
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bucketMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int get_left_front_drive_motor_enc() {
        if (leftFrontDriveMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return leftFrontDriveMotor.getCurrentPosition();
    }

    public int get_right_front_drive_motor_enc() {
        if (rightFrontDriveMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return rightFrontDriveMotor.getCurrentPosition();
    }

    public int get_left_back_drive_motor_enc() {
        if (leftBackDriveMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return leftBackDriveMotor.getCurrentPosition();
    }

    public int get_right_back_drive_motor_enc() {
        if (rightBackDriveMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return rightBackDriveMotor.getCurrentPosition();
    }

    public int get_climb_motor_enc() {
        if (climbMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return climbMotor.getCurrentPosition();
    }

    public int get_flip_motor_enc() {
        if (flipMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return flipMotor.getCurrentPosition();
    }

    public int get_lift_motor_enc() {
        if (liftMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return liftMotor.getCurrentPosition();
    }

    public int get_bucket_motor_enc() {
        if (bucketMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            bucketMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return bucketMotor.getCurrentPosition();
    }
}