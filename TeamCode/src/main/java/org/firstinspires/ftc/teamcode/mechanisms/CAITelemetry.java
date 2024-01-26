package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CAITelemetry implements Telemetry {
    Telemetry dashboardTelemetry;
    Telemetry ftcTelemetry;
    FtcDashboard dashboard;
    private boolean dashboardEnabled = Constants.DASHBOARD_ENABLED;
    private boolean telemetryEnabled = Constants.TELEMETRY_ENABLED;

    public CAITelemetry(Telemetry ftcTelemetry) {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        this.ftcTelemetry = ftcTelemetry;
    }

    public boolean update() {
        boolean bReturn = false;
        if (dashboardEnabled) {
            bReturn = dashboardTelemetry.update();
        }
        if (telemetryEnabled) {
            bReturn = ftcTelemetry.update() || bReturn;
        }
        return bReturn;
    }

    @Override
    public Line addLine() {
        Line iReturn = null;
        if (dashboardEnabled) {
            iReturn = dashboardTelemetry.addLine();
        }
        if (telemetryEnabled) {
            ftcTelemetry.addLine();
        }
        return iReturn;
    }

    @Override
    public Line addLine(String lineCaption) {
        Line iReturn = null;
        if (dashboardEnabled) {
            iReturn = dashboardTelemetry.addLine(lineCaption);
        }
        if (telemetryEnabled) {
            ftcTelemetry.addLine(lineCaption);
        }
        return iReturn;
    }

    @Override
    public boolean removeLine(Line line) {
        boolean bReturn = false;
        if (dashboardEnabled) {
            bReturn = dashboardTelemetry.removeLine(line);
        }
        if (telemetryEnabled) {
            bReturn = ftcTelemetry.removeLine(line) || bReturn;
        }
        return bReturn;
    }

    @Override
    public boolean isAutoClear() {
        boolean bReturn = false;
        if (dashboardEnabled) {
            bReturn = dashboardTelemetry.isAutoClear();
        }
        if (telemetryEnabled) {
            bReturn = ftcTelemetry.isAutoClear() || bReturn;
        }
        return bReturn;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        if (dashboardEnabled) {
            dashboardTelemetry.setAutoClear(autoClear);
        }
        if (telemetryEnabled) {
            ftcTelemetry.setAutoClear(autoClear);
        }
    }

    @Override
    public int getMsTransmissionInterval() {
        int interval = 0;
        if (dashboardEnabled) {
            interval = dashboardTelemetry.getMsTransmissionInterval();
        }
        if (telemetryEnabled) {
            interval = Math.max(ftcTelemetry.getMsTransmissionInterval(), interval);
        }
        return interval;
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        if (dashboardEnabled) {
            dashboardTelemetry.setMsTransmissionInterval(msTransmissionInterval);
        }
        if (telemetryEnabled) {
            ftcTelemetry.setMsTransmissionInterval(msTransmissionInterval);
        }
    }

    @Override
    public String getItemSeparator() {
        String separator = null;
        if (dashboardEnabled) {
            separator = dashboardTelemetry.getItemSeparator();
        } else if (telemetryEnabled) {
            separator = ftcTelemetry.getItemSeparator();
        }
        return separator;
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        if (dashboardEnabled) {
            dashboardTelemetry.setItemSeparator(itemSeparator);
        }
        if (telemetryEnabled) {
            ftcTelemetry.setItemSeparator(itemSeparator);
        }
    }

    @Override
    public String getCaptionValueSeparator() {
        String sReturn = null;
        if (dashboardEnabled) {
            sReturn = dashboardTelemetry.getCaptionValueSeparator();
        } else if (telemetryEnabled) {
            sReturn = ftcTelemetry.getCaptionValueSeparator();
        }
        return sReturn;
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        if (dashboardEnabled) {
            dashboardTelemetry.setCaptionValueSeparator(captionValueSeparator);
        }
        if (telemetryEnabled) {
            ftcTelemetry.setCaptionValueSeparator(captionValueSeparator);
        }
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        if (dashboardEnabled) {
            dashboardTelemetry.setDisplayFormat(displayFormat);
        }
        if (telemetryEnabled) {
            ftcTelemetry.setDisplayFormat(displayFormat);
        }
    }

    @Override
    public Log log() {
        Log lReturn = null;
        if (dashboardEnabled) {
            lReturn = dashboardTelemetry.log();
        } else if (telemetryEnabled) {
            lReturn = ftcTelemetry.log();
        }
        return lReturn;
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        Item iReturn = null;
        if (dashboardEnabled) {
            iReturn = dashboardTelemetry.addData(caption, format, args);
        }
        if (telemetryEnabled) {
            ftcTelemetry.addData(caption, format, args);
        }
        return iReturn;
    }

    public Item addData(String caption, Object value) {
        Item iReturn = null;
        if (dashboardEnabled) {
            iReturn = dashboardTelemetry.addData(caption, value);
        }
        if (telemetryEnabled) {
            ftcTelemetry.addData(caption, value);
        }
        return iReturn;
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        Item iReturn = null;
        if (dashboardEnabled) {
            iReturn = dashboardTelemetry.addData(caption, valueProducer);
        }
        if (telemetryEnabled) {
            ftcTelemetry.addData(caption, valueProducer);
        }
        return iReturn;
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        Item iReturn = null;
        if (dashboardEnabled) {
            iReturn = dashboardTelemetry.addData(caption, format, valueProducer);
        }
        if (telemetryEnabled) {
            ftcTelemetry.addData(caption, format, valueProducer);
        }
        return iReturn;
    }

    @Override
    public boolean removeItem(Item item) {
        Boolean bReturn = false;
        if (dashboardEnabled) {
            bReturn = dashboardTelemetry.removeItem(item);
        }
        if (telemetryEnabled) {
            bReturn = ftcTelemetry.removeItem(item) || bReturn;
        }
        return bReturn;
    }

    @Override
    public void clear() {
        if (dashboardEnabled) {
            dashboardTelemetry.clear();
        }
        if (telemetryEnabled) {
            ftcTelemetry.clear();
        }
    }

    @Override
    public void clearAll() {
        if (dashboardEnabled) {
            dashboardTelemetry.clearAll();
        }
        if (telemetryEnabled) {
            ftcTelemetry.clearAll();
        }
    }

    @Override
    public Object addAction(Runnable action) {
        Object oReturn = null;
        if (dashboardEnabled) {
            oReturn = dashboardTelemetry.addAction(action);
        }
        if (telemetryEnabled) {
            ftcTelemetry.addAction(action);
        }
        return oReturn;
    }

    @Override
    public boolean removeAction(Object token) {
        boolean bReturn = false;
        if (dashboardEnabled) {
            bReturn = dashboardTelemetry.removeAction(token);
        }
        if (telemetryEnabled) {
            ftcTelemetry.removeAction(token);
        }
        return bReturn;
    }

    @Override
    public void speak(String text) {
        if (dashboardEnabled) {
            dashboardTelemetry.speak(text);
        }
        if (telemetryEnabled) {
            ftcTelemetry.speak(text);
        }
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        if (dashboardEnabled) {
            dashboardTelemetry.speak(text, languageCode, countryCode);
        }
        if (telemetryEnabled) {
            ftcTelemetry.speak(text, languageCode, countryCode);
        }
    }

    public void setDashboardEnabled(boolean dashboardEnabled) {
        this.dashboardEnabled = dashboardEnabled;
    }

    public void setTelemetryEnabled(boolean telemetryEnabled) {
        this.telemetryEnabled = telemetryEnabled;
    }
}
