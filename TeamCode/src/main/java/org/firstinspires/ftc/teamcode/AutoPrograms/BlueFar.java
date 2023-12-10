package org.firstinspires.ftc.teamcode.AutoPrograms;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "BlueFar", group = "Auto")
public class BlueFar extends AutoOpMode {
    //@Override
    public void runOpmode() {
        super.runOpMode();
        int zone = cam.detectElement();

        switch (zone) {
            case 1:


                //Strafe left.
                // Deposit}
        } // end of switch
    }// end of OpMode
}// end of class
