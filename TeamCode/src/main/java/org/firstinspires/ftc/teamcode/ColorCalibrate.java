/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@TeleOp
public class ColorCalibrate extends LinearOpMode {
  ColorSensor colorSensor;    // Hardware Device Object
  int Rvals[] = {0,0};
  int Bvals[] = {0,0};
  int R;
  int B;
  int Rfin;
  int Bfin;
  int LoopT;
  int SetT = 10; //Accuracy (seconds)
  int TimeD = 1000; //Put Delay time here
  ElapsedTime delay = new ElapsedTime();
  @Override
  public void runOpMode() {
    colorSensor = hardwareMap.colorSensor.get("sensor_color");
    colorSensor.enableLed(true);
    waitForStart();
    while (opModeIsActive()) {
      if (gamepad1.a) {
        while (LoopT != SetT) {
          R = colorSensor.red();
          B = colorSensor.blue();
          Rfin = Rfin + R;
          Bfin = Bfin + B;
          LoopT++;
          //try {
          //  Thread.sleep(TimeD);
          //} catch (InterruptedException e) {
          //  e.printStackTrace();
          //}
        }
        Rfin = Math.round(Rfin / LoopT);
        Bfin = Math.round(Bfin / LoopT);
        Bvals[0] = Rfin;
        Bvals[1] = Bfin;
      }
      if (gamepad1.b) {
        while (LoopT != SetT) {
          R = colorSensor.red();
          B = colorSensor.blue();
          Rfin = Rfin + R;
          Bfin = Bfin + B;
          LoopT++;
          //try {
          //  Thread.sleep(TimeD);
          //} catch (InterruptedException e) {
          //  e.printStackTrace();
          //}
        }
        Rfin = Math.round(Rfin / LoopT);
        Bfin = Math.round(Bfin / LoopT);
        Rvals[0] = Rfin;
        Rvals[1] = Bfin;
      }
      telemetry.addData("RedValues:", Rvals[0] + " " + Rvals[1]);
      telemetry.addData("BlueValues: ", Bvals[0] + " " + Bvals[1]);
      telemetry.update();
    }
  }
}
