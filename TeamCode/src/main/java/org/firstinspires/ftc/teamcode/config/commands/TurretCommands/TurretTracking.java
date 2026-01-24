package org.firstinspires.ftc.teamcode.config.commands.TurretCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.config.subsystem.TurretSubsystem;

public class TurretTracking extends CommandBase {
    private final TurretSubsystem turret;

    public TurretTracking(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.TurretTracking();
    }
}
