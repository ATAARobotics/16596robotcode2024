package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestAutoDriveTo",group = "")
public class Test_autoDriveTo extends AutoOpMode{

    public void runOpMode() {

// After "start" do this - test driveTo:
        driveTrain.driveTo(.5,20,20);

    }//end of runOpMode
}//end of class
