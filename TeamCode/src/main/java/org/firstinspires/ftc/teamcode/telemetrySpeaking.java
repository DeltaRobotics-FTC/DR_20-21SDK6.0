package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="telemetrySpeaking" ,group = "")
public class telemetrySpeaking extends LinearOpMode {

    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {
            while (!gamepad1.left_bumper) {
                if (gamepad1.a) {
                    telemetry.speak("hi");
                    telemetry.update();
                }
                if (gamepad1.b) {
                    telemetry.speak("hello");
                    telemetry.update();
                }
                if (gamepad1.y) {
                    telemetry.speak("is it me");
                    telemetry.update();
                }
                if (gamepad1.x) {
                    telemetry.speak("your looking for");
                    telemetry.update();
                }
                if (gamepad1.dpad_up) {
                    telemetry.speak("rawr");
                    telemetry.update();
                }
                if (gamepad1.dpad_down) {
                    telemetry.speak("good job");
                    telemetry.update();
                }
                if (gamepad1.dpad_left) {
                    telemetry.speak("hows your day");
                    telemetry.update();
                }
                if (gamepad1.dpad_right) {
                    telemetry.speak("you're the best");
                    telemetry.update();
                }
            }

            while (gamepad1.left_bumper) {
                if (gamepad1.a) {
                    telemetry.speak("press init");
                    telemetry.update();
                }
                if (gamepad1.b) {
                    telemetry.speak("the code broke again");
                    telemetry.update();
                }
                if (gamepad1.y) {
                    telemetry.speak("bla bla bla");
                    telemetry.update();
                }
                if (gamepad1.x) {
                    telemetry.speak("error 404");
                    telemetry.update();
                }
                if (gamepad1.dpad_up) {
                    telemetry.speak("ouch");
                    telemetry.update();
                }
                if (gamepad1.dpad_down) {
                    telemetry.speak("full charge ahead");
                    telemetry.update();
                }
                if (gamepad1.dpad_left) {
                    telemetry.speak("this is battle bots");
                    telemetry.update();
                }
                if (gamepad1.dpad_right) {
                    telemetry.speak("im working right now");
                    telemetry.update();
                }
            }

        }
    }
}
