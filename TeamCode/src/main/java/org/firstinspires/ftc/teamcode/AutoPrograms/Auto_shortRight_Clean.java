package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Airplane;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;

@Autonomous(name = "shortRight",group = "")

public class Auto_shortRight_Clean extends LinearOpMode {
  private DriveTrain driveTrain;
  private Arm arm;
  private Airplane drone;

  public void runOpMode(){
      driveTrain = new DriveTrain(hardwareMap);
      arm = new Arm(hardwareMap);
      drone = new Airplane(hardwareMap);

      driveTrain.init();
      arm.init();
      drone.init();
      waitForStart();
  }
  //step 1 strafe right for 38 inches
    s]
}
