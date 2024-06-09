package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Salute")
public class Salute extends LinearOpMode {
    // initialize motors and servos to null.
    DcMotor fm = null;
    DcMotor bm = null;
    Servo elbow = null;
    Servo hand = null;

    // Convert encoder ticks on the HD hex encoder motors to degrees
    /**
     * Cookie Crumb 1 - Encoders.
     *
     * Imagine you had a wheel with a bunch of dashes around the outside.  If you know the number of
     * dashes around the outside of the wheel, and can count the number of dashes that pass as the
     * wheel spins, you can figure out how much the wheel has spun.  By dividing the number of dashes
     * you have counted by the number of dashes on the wheel you can calculate the number of
     * revolutions the wheel has completed.  This is essentially how an encoder works.  The encoder
     * counts the number of dashes (called counts) that passes it, and then the programmer can use
     * the number of counts on the wheel of the encoder to convert to a measurement of how far the
     * wheel has travelled.  Here, we convert these encoder counts into degrees, since we want the
     * arm to rotate to a specific degree.
    **/

    // The motors we use have integrated encoders that have 28 counts on the wheel.
    static final double COUNTS_PER_MOTOR_REV = 28;
    // Since this encoder is on the motor, we need to convert the number of times the motor turns
    // to the number of times the gear in the shoulder turns.  This number should be 100.
    static final double DRIVE_GEAR_REDUCTION = 100;
    // Now we can calculate one big number that relates the number of encoder counts to the number
    // of degrees the gear on the motor should turn.  If we multiply the desired number of degrees
    // by this number, we can send this number to the motor to know how much to rotate.
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (360);

    /**
     * This is the constants section.  These are the values that represent the positions the arm
     * should go to, as well as the speed at which the arm should go to these positions.  Feel free
     * to adjust these numbers if necessary.  Just make a change to the number, and while connected
     * control hub, press the play button or the replay button at the top middle of the window to
     * update your robot to the new values.
     * Remember - these values below are the ones that worked with our team!  It is perfectly fine
     * if you need to change them, feel free to!
     */

    //  The angle at which the front motor will go to to raise the arm up
    final static double FM_UP_ANGLE = 180;
    // The angle at which the back motor will go to to raise the arm up.  Ideally,
    // FM_UP_ANGLE - BM_UP_ANGLE should be 180
    final static double BM_UP_ANGLE = 0;
    //  The angle at which the front motor will go to to lower the arm
    final static double FM_DOWN_ANGLE = 0;
    //  The angle at which the back motor will go to to lower the arm
    final static double BM_DOWN_ANGLE = 0;
    final static double FM_LOW_FIVE = 45;
    final static double BM_LOW_FIVE = -45;
    /**
     * Cookie Crumb 2 - Servos.
     * The elbow and hand actuators are servos, which work a bit differently from the motors used to
     * power the shoulder.  Servos do not need encoders to go to a specific position, instead using
     * something called a Pulse Width Modulation (PWM) signal to determine the position.  This makes
     * servos much easier to control, but they typically not as strong as normal motors and means a
     * servo cannot rotate forever like a normal motor, instead going between two set positions For our
     * code, all we need to know is a value of 0.0 means all the way to the left, a value of 1.0
     * means all the way to the right, and a value of 0.5 is right in the middle.
     */
    final static double ELBOW_UP = 0.5;
    final static double ELBOW_RELAXED = 1.0;
    final static double ELBOW_WAVE_OFFSET = 0.2;
    final static double HAND_SALUTE = 1.0;
    final static double HAND_OPEN = 0.5;
    final static double UP_SPEED = 1.0;
    final static double DOWN_SPEED = 0.5;
    final static double ELBOW_LOW_FIVE = 0.75;
    // Initializing all used variables
    double speed = 0;
    double fm_target = 0;
    double bm_target = 0;
    double elbow_target = 1.0;
    double hand_target = 0.5;
    boolean up = false;
    ElapsedTime timer = new ElapsedTime();
    boolean end = false;
    ElapsedTime end_timer = new ElapsedTime();

    /**
     * Future Senior Project groups should add more cookie crumbs for an educational experience maybe
     */
    // Initialize the teleoperated op mode
    public void runOpMode(){
        // initialize the first motor
        fm = hardwareMap.dcMotor.get("fm");
        fm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initialize the second motor
        bm = hardwareMap.dcMotor.get("bm");
        bm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initialize the third motor
        elbow = hardwareMap.servo.get("elbow");

        // initialize the fourth motor
        hand = hardwareMap.servo.get("hand");

        // waiting until start button is pressed
        waitForStart();
        while(opModeIsActive()){
            // make the arm go up to the salute position with the controller
            if(gamepad1.y && !gamepad1.a && !gamepad1.right_bumper && !end){
                speed = UP_SPEED;
                fm_target = (FM_UP_ANGLE*COUNTS_PER_DEGREE);
                bm_target = (-BM_UP_ANGLE*COUNTS_PER_DEGREE);
                elbow_target = ELBOW_UP;
                hand_target = HAND_SALUTE;
                up = true;
            }

            // make the arm go down to the rest position with the controller
            if(gamepad1.a && !gamepad1.y && !gamepad1.right_bumper && !end){
                speed = DOWN_SPEED;
                fm_target = FM_DOWN_ANGLE;
                bm_target = BM_DOWN_ANGLE;
                elbow_target = ELBOW_RELAXED;
                hand_target = HAND_OPEN;
                up = false;
            }

            // make the arm wave
            if(gamepad1.x && up && !gamepad1.a && !gamepad1.y && !gamepad1.right_bumper && !end){
                hand_target = HAND_OPEN;
                if(timer.milliseconds() >= 750){
                    elbow_target = ELBOW_UP + ELBOW_WAVE_OFFSET;
                }
                if (timer.milliseconds() >=1500) {
                    elbow_target = ELBOW_UP - ELBOW_WAVE_OFFSET;
                    timer.reset();
                }
            }

            // make the arm go to the low five position
            if(gamepad1.right_bumper && !gamepad1.a && !gamepad1.y && !gamepad1.x && !end){
                speed = DOWN_SPEED;
                fm_target = (FM_LOW_FIVE*COUNTS_PER_DEGREE);
                bm_target = (-BM_LOW_FIVE*COUNTS_PER_DEGREE);
                elbow_target = ELBOW_LOW_FIVE;
                hand_target = HAND_OPEN;
                up = false;
            }

            // reset hand to salute position
            if(up && !gamepad1.x && !end){
                elbow_target = ELBOW_UP;
                hand_target = HAND_OPEN;
            }

            // make the arm go to the rest position for disable button
            if(gamepad1.b){
                speed = DOWN_SPEED;
                fm_target = FM_DOWN_ANGLE;
                bm_target = BM_UP_ANGLE;
                elbow_target = ELBOW_RELAXED;
                hand_target = HAND_OPEN;
                up = false;
                end = true;
                end_timer.reset();
            }
            if(end && end_timer.milliseconds() >= 1000){
                break;
            }

            // set positions and speeds for motors, and positions for servos
            fm.setTargetPosition((int)fm_target);
            fm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fm.setPower(Math.abs(speed));
            bm.setTargetPosition((int)bm_target);
            bm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bm.setPower(Math.abs(speed));
            elbow.setPosition(elbow_target);
            hand.setPosition(hand_target);
        }
    }
}
