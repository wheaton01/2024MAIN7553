// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class limelight {
NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

    public double getLimelightTX(){
        return table.getEntry("tx").getDouble(0);
    }
    public double getLimelightTY(){
        return table.getEntry("ty").getDouble(0);
    }
    public double getLimelightTA(){
        return table.getEntry("ta").getDouble(0);
    }
}
