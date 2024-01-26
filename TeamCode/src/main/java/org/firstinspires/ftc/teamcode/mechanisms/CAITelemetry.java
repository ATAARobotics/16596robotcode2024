package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CAITelemetry {
    Telemetry dashboardTelemetry;
    Telemetry ftcTelemetry;
    private boolean dashboardEnabled;
    private boolean telemetryEnabled;

    public CAITelemetry(Telemetry ftcTelemetry, Telemetry dashboardTelemetry) {
        this.dashboardTelemetry = dashboardTelemetry;
        this.ftcTelemetry = ftcTelemetry;
        dashboardEnabled = Constants.DASHBOARD_ENABLED;
        telemetryEnabled = Constants.TELEMETRY_ENABLED;
    }

    public void update() {
        if(dashboardEnabled) {
            dashboardTelemetry.update();
        }
        if(telemetryEnabled) {
            ftcTelemetry.update();
        }
    }

    public void addData(String caption, Object value) {
        if(dashboardEnabled) {
            dashboardTelemetry.addData(caption,value);
        }
        if(telemetryEnabled) {
            ftcTelemetry.addData(caption,value);
        }
    }

    public void setDashboardEnabled(boolean dashboardEnabled) {
        this.dashboardEnabled = dashboardEnabled;
    }

    public void setTelemetryEnabled(boolean telemetryEnabled) {
        this.telemetryEnabled = telemetryEnabled;
    }
}
