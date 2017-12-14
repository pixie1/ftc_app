package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class JewelGlyphParkAutoRedPerimeter extends MainOpMode {
    @Override
    public void runOpMode() {
        int color = 1; //1 for red -1 for blue
        initAll();
        JewelGlyphParkAutoPerimeter(color);
    }
}
