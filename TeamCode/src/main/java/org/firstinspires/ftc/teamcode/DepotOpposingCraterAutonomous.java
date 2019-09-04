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

public class DepotOpposingCraterAutonomous extends BaseRobot {
    private int stage = 0;
    private GoldAlignDetector detector;
    private double turnMult = 0;
    private boolean ready = false;

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
        telemetry.addData("Current case: " , stage); // Current Case.

        switch (stage) {
            case 0:
                if(!ready) {
                    detector.enable();
                    ready = true;
                }
                if(timer.time()>3) {
                    stage++;
                    timer.reset();
                }
                break;
            case 1:
                //find cube
                if (detector.getAligned()) {
                    turnMult=0;
                    stage++;
                }
                if (detector.getXPosition()<100 && detector.getXPosition() != 0 && turnMult == 0) {
                    turnMult = 1;
                    stage++;
                } else if ((detector.getXPosition()>460 && turnMult == 0) || timer.time()>3) {
                    turnMult = -1;
                    stage++;
                }
                break;
            case 2:
                //detach climber
                if (Math.abs(get_climb_motor_enc())>ConstantVariables.K_MAX_CLIBER) {
                    climb(0);
                    stage++;
                } else {
                    climb(-1);
                }
                break;
            case 3:
                //turn to face minerals
                if (auto_turn(-0.8, 110)) { //turns left
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 4:
                //turn to gold cube
                if (turnMult == 0) {
                    reset_drive_encoders();
                    stage++;
                } else if(turnMult == 1) {
                    if (auto_turn(-0.2, 70)) { //turns left
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
            case 5:
                //knock gold cube off
                if (turnMult == 0) {
                    if (auto_drive(1, 16)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else if (turnMult == -1) {
                    if (auto_drive(1, 27)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else {
                    if (auto_drive(1, 32)) {
                        reset_drive_encoders();
                        stage++;
                    }
                }

                break;
            case 6:
                //turn to face depot
                if (turnMult == 0) {
                    reset_drive_encoders();
                    stage++;
                } else if (turnMult == 1) {
                    if (auto_turn(0.5, 80)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else if (turnMult == -1){
                    if (auto_turn(-0.5, 110)) {
                        reset_drive_encoders();
                        stage++;
                    }
                }
                break;
            case 7:
                //drive to depot
                if (auto_drive(1, 20)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 8:
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
                    if (auto_turn(0.6, 140)) {
                        reset_drive_encoders();
                        timer.reset();
                        stage++;
                    }
                }
                break;

            case 9:
                //drop marker
                if(timer.time()>1) {
                    timer.reset();
                    stage++;
                }
                set_marker_servo(ConstantVariables.K_MARKER_SERVO_DOWN);
                break;
            case 10:
                //raise marker servo
                if(timer.time()>1) {
                    stage++;
                }
                set_marker_servo(ConstantVariables.K_MARKER_SERVO_UP);
                break;
            case 11:
                //drive to side wall
                if (turnMult == 1) {
                    if (auto_drive(-1, 8)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else if (turnMult == 0) {
                    if (auto_drive(-1, 10)) {
                        reset_drive_encoders();
                        stage++;
                    }
                } else {
                    if (auto_drive(-1, 15)) {
                        reset_drive_encoders();
                        stage++;
                    }
                }
                break;
            case 12:
                //turn to face crater
                if (auto_turn(-0.5, 40)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 13:
                //mechanum to wall
                if (auto_mecanum(-1, 20)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 14:
                //drive to crater
                if (auto_drive(-1, 38)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 15:
                //mechanum to wall
                if (auto_mecanum(-1, 5)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 16:
                //drive to crater
                if (auto_drive(-1, 30)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            default:
                break;
        }
    }

    @Override
    public void stop() {
        detector.disable();
    }
}
