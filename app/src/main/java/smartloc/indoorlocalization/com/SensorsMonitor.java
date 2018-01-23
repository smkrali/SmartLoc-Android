package smartloc.indoorlocalization.com;

import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.SystemClock;
import android.util.Log;

/**
 * Created by spider on 7/16/17.
 */

public class SensorsMonitor implements Runnable {

    private int interval;
    private boolean running;
    private long startTime;
    private SmartLocMain mainActivity;
    private android.os.Handler handler;

    private SensorManager sensorManager;
    private SensorInputListener sensorInputListener;
    private Sensor sensorStepCounter;
    //private Sensor sensorStepDetector; % TODO: Detect step. If useful.

    private int mPreviousCounterSteps;
    private int sensedSteps;
    private double stepChange;
    private int prevStepsTaken;
    private int initialSteps = -1;

    public SensorsMonitor(int tWait, SensorManager sm, SmartLocMain mac)
    {
        interval = tWait;
        sensorManager = sm;
        mainActivity = mac;

        running = false;
        handler = new android.os.Handler();
        sensorInputListener = new SensorInputListener();

        // Get default sensor types here.
        sensorStepCounter = sensorManager.getDefaultSensor(Sensor.TYPE_STEP_COUNTER);
        //sensorStepCounter = sensorManager.getDefaultSensor(Sensor.TYPE_STEP_DETECTOR);
    }

    public void start(){
        if(!running)
        {
            running = true;
            // Register sensors here.
            sensorManager.registerListener(sensorInputListener, sensorStepCounter,SensorManager.SENSOR_DELAY_GAME);
            //sensorManager.registerListener(sensorInputListener, sensorStepDetector,SensorManager.SENSOR_DELAY_FASTEST);

            startTime = SystemClock.uptimeMillis();
            handler.postDelayed(this, interval);
        }
    }

    public int getStepsTaken(){
        return prevStepsTaken;
    }

    public double getStepChange(){
        return stepChange;
    }

    @Override
    public void run(){
        if(running)
        {
            sensedSteps = sensorInputListener.getStepTaken();

            // Step Detection.
            if (initialSteps < 0){
                initialSteps = sensedSteps;
                prevStepsTaken = initialSteps;
            }
            else {
                stepChange = (1000*(double)(sensedSteps - prevStepsTaken))/interval;
                prevStepsTaken = sensedSteps + initialSteps;
            }

            handler.postDelayed(this, interval);
        }
    }

    public void stop(){
        if(running)
        {
            running = false;
            sensorManager.unregisterListener(sensorInputListener);
        }
    }

    public SensorInputListener getSensorListener()
    {
        return sensorInputListener;
    }
}
