package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * Created by Chun on 1/26/19 for 10023.
 */

@Autonomous
//@Disabled

public class DepotNoParkAutonomous extends BaseRobot {
    private int stage = 0;
    private GoldAlignDetector detector;
    private double turnMult = 0;

    @Override
    public void init() {
        super.init();

        //170 445 -280
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 200; //50 How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; //0 How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral
        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.
        telemetry.addData("Case: " , stage); // Current Case.

        switch (stage) {
            case 0:
                //find cube
                if (detector.getAligned() || timer.time()>3) {
                    turnMult=0;
                    timer.reset();
                    stage++;
                }
                if (detector.getXPosition()<100 && detector.getXPosition() != 0 && turnMult == 0) {
                    turnMult = 1;
                    stage++;
                } else if (detector.getXPosition()>460 && detector.getXPosition() != 0 && turnMult == 0) {
                    turnMult = -1;
                    stage++;
                }
                break;
            case 1:
                //detach climber
                if (Math.abs(get_climb_motor_enc())>ConstantVariables.K_MAX_CLIBER) {
                    climb(0);
                    stage++;
                } else {
                    climb(-1);
                }
                break;
            case 2:
                //turn to face minerals
                if (auto_turn(-1, 115)) { //turns left
                    reset_drive_encoders();
                    stage++;
                }
                break;

            case 3:
                //turn to cube
                if (turnMult == 0) {
                    reset_drive_encoders();
                    stage++;
                } else if(turnMult == 1) {
                    if (auto_turn(-0.2, 75)) { //turns left
                        reset_drive_encoders();
                        timer.reset();
                        stage++;
                    }
                } else if (turnMult == -1) {
                    if (auto_turn(0.2, 23)) { //turns right
                        reset_drive_encoders();
                        timer.reset();
                        stage++;
                    }
                }
                break;
            case 4:
                //knock gold block off
                if (turnMult == 0) {
                    if (auto_drive(1, 20)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else {
                    if (auto_drive(1, 30)) {
                        reset_drive_encoders();
                        stage++;
                    }
                }
                break;
            case 5:
                //turn to face depot
                if (turnMult == 0) {
                    reset_drive_encoders();
                    stage++;
                } else if (turnMult == 1) {
                    if (auto_turn(0.5, 75)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else if (turnMult == -1){
                    if (auto_turn(-0.5, 95)) {
                        reset_drive_encoders();
                        stage++;
                    }
                }
                break;
            case 6:
                //drive to depot
                if (auto_drive(1, 25)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 7:
                //turn so marker faces depot
                if (turnMult == 1) {
                    if (auto_turn(0.6, 45)) {
                        reset_drive_encoders();
                        timer.reset();
                        stage++;
                    }
                } else if (turnMult == 0) {
                    if (auto_turn(0.6, 120)) {
                        reset_drive_encoders();
                        timer.reset();
                        stage++;
                    }
                } else {
                    if (auto_turn(0.6, 160)) {
                        reset_drive_encoders();
                        timer.reset();
                        stage++;
                    }
                }
                break;

            case 8:
                //drop marker
                if(timer.time()>1) {
                    timer.reset();
                    stage++;
                }
                set_marker_servo(ConstantVariables.K_MARKER_SERVO_DOWN);
                break;
            case 9:
                //raise marker servo
                if(timer.time()>1) {
                    stage++;
                }
                set_marker_servo(ConstantVariables.K_MARKER_SERVO_UP);
                break;
            /*case 10:
                //drive to side wall
                if (turnMult == 1) {
                    if (auto_drive(-1, 3)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else if (turnMult == 0) {
                    if (auto_drive(-1, 10)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else {
                    if (auto_drive(-1, 20)) {
                        reset_drive_encoders();
                        stage++;
                    }
                }
                break;
            case 11:
                //turn to face crater
                if (auto_turn(-0.5, 40)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 12:
                //mechanum to wall
                if (auto_mecanum(-1, 20)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 13:
                //drive to crater
                if (auto_drive(-1, 10)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;*/
            default:
                break;
        }
    }

    @Override
    public void stop() {
        detector.disable();
    }
}
