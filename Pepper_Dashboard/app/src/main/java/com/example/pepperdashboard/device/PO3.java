package com.example.pepperdashboard.device;

import android.Manifest;
import android.app.Application;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.example.pepperdashboard.R;
import com.example.pepperdashboard.activity.MisActivity;
import com.example.pepperdashboard.logic.RosMotion;
import com.example.pepperdashboard.logic.RosTTS;
import com.ihealth.communication.control.HsProfile;
import com.ihealth.communication.control.Po3Control;
import com.ihealth.communication.control.PoProfile;
import com.ihealth.communication.manager.DiscoveryTypeEnum;
import com.ihealth.communication.manager.iHealthDevicesCallback;
import com.ihealth.communication.manager.iHealthDevicesManager;
import com.yabu.livechart.model.DataPoint;
import com.yabu.livechart.model.Dataset;
import com.yabu.livechart.view.LiveChart;
import com.yabu.livechart.view.LiveChartStyle;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import kotlin.jvm.internal.markers.KMutableList;

public class PO3 extends AppCompatActivity {
    private static String TAG = PO3.class.getName();
    private iHealth iHealth;
    private PO3 po3;
    private int callbackId;
    private String mac;
    private String deviceType = "PO3";
    private boolean done = false;

    private TextView connect;
    private TextView battery;
    private Button backBtn;
    private Button startButton;

    private boolean is_connected;
    private iHealth app;
    private Po3Control control;

    private ListView resultList;
    private List<String> itemArray;
    private ArrayAdapter<String> adapter;

    private LiveChart liveChart;
    private Dataset dataset;
    private  ArrayList<DataPoint> dataPoints;

    private TextView infoText;
    private Runnable a;

    /**
     * The getDataPoint function returns the dataPoints ArrayList.
     *
     *
     *
     * @return An arraylist of datapoints
     *
     * @docauthor Lorenzo Rossi
     */
    public synchronized ArrayList<DataPoint> getDataPoint(){
        return dataPoints;
    }

    /**
     * The setDataPoint function adds a DataPoint object to the dataPoints ArrayList.
     *
     *
     * @param DataPoint dataPoint Add a new datapoint object to the arraylist datapoints
     *
     * @return Void
     *
     * @docauthor Lorenzo Rossi
     */
    public synchronized void setDataPoint(DataPoint dataPoint){
        dataPoints.add(dataPoint);    }

    /**
     * The onCreate function is called when the activity is created.
     * It sets up the layout of the activity, and initializes all variables.

     *
     * @param Bundle savedInstanceState Save the state of the activity

     *
     * @return A view
     *
     * @docauthor Lorenzo Rossi
     */

    RosMotion motion;
    RosTTS tts;
    Thread thread;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.pulse_activity);
        connect = findViewById(R.id.onConnect);
        battery = findViewById(R.id.battery);
        backBtn = findViewById(R.id.bacKBtn);
        startButton = findViewById(R.id.startBtn);
        resultList = findViewById(R.id.listMis);
        liveChart = findViewById(R.id.live_chart);
        infoText = findViewById(R.id.infoTextView);
        motion = new RosMotion(getIntent().getStringExtra("host"), 9999, this, getAssets());
        tts = new RosTTS(getIntent().getStringExtra("host"), 9090, this, getAssets());
        motion.setContent(motion.get_arm_up());
        motion.execute();
        // Setup dataset for livechart
        dataPoints = new ArrayList<>();

        // Draw the chart at runtime in an async task

        // Disable the click on the button until the device is connected
        startButton.setEnabled(false);
        // Set up listView items
        itemArray = Collections.synchronizedList(new ArrayList<String>());
        itemArray.clear();
        adapter = new ArrayAdapter<String>(this,
                android.R.layout.simple_list_item_1, itemArray);
        resultList.setAdapter(adapter);

        app = (iHealth) getApplication();
        app.discover(this);
//        Po3Control control = app.getControl();
        Log.e(TAG, "control: " + control);
        startButton.setOnClickListener(view -> {
            //control.getBattery();
//            Po3Control control = iHealthDevicesManager.getInstance().getPo3Control(app.getMac());

            control.startMeasure();
            // disable click button
            startButton.setEnabled(false);

        });

        backBtn.setOnClickListener(view -> {
            finish();
        });

        a = new Runnable() {
            @Override
            public void run() {

                int prec_size=0;
                LiveChart.OnTouchCallback datasetListener = new LiveChart.OnTouchCallback() {
                    /**
                     * The onTouchFinished function is called when the user has finished touching the screen.
                     * This function will be used to determine if a swipe was made, and if so, what direction it was in.

                     *
                     *
                     * @return A boolean value
                     *
                     * @docauthor Lorenzo Rossi
                     */
                    @Override
                    public void onTouchFinished() {

                    }

                    @Override
                    public void onTouchCallback(@NonNull DataPoint dataPoint) {
                        infoText.setText(":\t" + dataPoint.getY());
                    }
                };
                LiveChartStyle chartStyle = new LiveChartStyle();
                chartStyle.setOverlayLineColor(Color.BLUE);
                chartStyle.setOverlayCircleDiameter(32f);
                chartStyle.setOverlayCircleColor(Color.GREEN);
                chartStyle.setTextHeight(20.0f);
                liveChart.setDataset(new Dataset(getDataPoint()))
                        .setLiveChartStyle(chartStyle)
                        // Draws the Y Axis bounds with Text data points.
                        .drawYBounds()
                        // Draws a customizable base line from the first point of the dataset or manually set a data point
                        .drawBaseline()
                        // Set manually the data point from where the baseline draws,
                        //.setBaselineManually(1.5f)
                        // draws the color of the path and fill conditional to being above/below the baseline datapoint
                        //.drawBaselineConditionalColor()
                        // Draw Guidelines in the background
                        .drawVerticalGuidelines(4)
                        .drawHorizontalGuidelines(4)
                        // Draw smooth path
                        .drawSmoothPath()
                        .setOnTouchCallbackListener(datasetListener)
                        // Draw last point tag label
                        .drawLastPointLabel();
                while (true) {
                    try {
                        // Create a new DataPoint with the received bloodOxygen value
                        Log.e(TAG, "dataPoints: " + dataPoints.size());
                        // Update the LiveChart to reflect the new data
                        if (dataPoints.size()>=2 && dataPoints.size() != prec_size) {

                            liveChart.drawDataset();
                            prec_size = dataPoints.size();
                        }
                        else {
                            Thread.sleep(1000);

                        }

                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }};
        thread = new Thread(a);
        thread.start();
    }

    /**
     * The finish function is called when the user presses the back button.
     * It destroys all instances of iHealthDevicesManager, and returns to the previous activity.

     *
     *
     * @return The following:
     *
     * @docauthor Lorenzo Rossi
     */
    public void finish() {
        iHealthDevicesManager.getInstance().destroy();
        // Destroy Runnable object

        startButton.setVisibility(Button.VISIBLE);
        Intent backIntent = new Intent(PO3.this, MisActivity.class);
        backIntent.putExtra("host", getIntent().getStringExtra("host"));
        startActivity(backIntent);
    }

    /**
     * The onStart function is called when the activity enters the Started state.
     * This happens when the activity becomes visible to the user, either because it was just launched or because it had been stopped and is now being restarted by another activity.
     * The onStart function performs basic application startup logic that should happen after any application initialization but before anything interactive happens in your app (such as user input).

     *
     *
     * @return A boolean value
     *
     * @docauthor Lorenzo Rossi
     */
    @Override
    protected void onStart() {
        super.onStart();
//        boolean connected = iHealthDevicesManager.getInstance().connectDevice(mac, deviceType);
//        if (connected) {
//            Log.e(TAG, "connected");
//        } else {
//            Log.e(TAG, "disconnected");
//        }

    }


    public String getMac() {
        return mac;
    }

    public void setMac(String mac) {
        this.mac = mac;
    }

    public String getDeviceType() {
        return deviceType;
    }

    public void setDeviceType(String deviceType) {
        this.deviceType = deviceType;
    }

    /**
     * The enableMeasure function is called when the user clicks on the &quot;ON&quot; button.
     * It enables the startButton and changes its color to green, indicating that it is now enabled.
     * It also calls getBattery() from PO3Control class in order to get battery information from device.

     *
     *
     * @return A boolean, but it doesn't seem to be used anywhere
     *
     * @docauthor Lorenzo Rossi
     */
    public void enableMeasure() {
        control = iHealthDevicesManager.getInstance().getPo3Control(app.getMac());
        Handler handler = new Handler();
        handler.postDelayed(new Runnable() {
      /**
             * The run function is called when the thread is started.
             * It sets the startButton to enabled, and changes the text of connect to &quot;ON&quot;
             * and its color to green. Then it calls getBattery() from control class.

             *
             *
             * @return A void, so you can't return anything from it
             *
             * @docauthor Lorenzo Rossi
             */
                  @Override
            public void run() {
                startButton.setEnabled(true);
                connect.setText("ON");
                connect.setTextColor(getResources().getColor(R.color.green));
                control.getBattery();
//
            }
        },7000);


    }

    /**
     * The setBattery function sets the battery level of the device.
     *
     *
     * @param int battery Set the battery level
     *
     * @return Void
     *
     * @docauthor Lorenzo Rossi
     */
    public void setBattery(int battery) {
        this.battery.setText(battery + "%");
    }

    /**
     * The setResultList function is called by the MainActivity class when a new set of results are available.
     * The function takes in four parameters: bloodOxygen, pulseRate, pulseStrength and pi. These values are then
     * added to an array which is used to populate the ListView on the ResultsFragment page. This function also calls
     * notifyDataSetChanged() so that any changes made to itemArray will be reflected in the ListView on screen.

     *
     * @param int bloodOxygen Store the blood oxygen value, which is then displayed in the listview
     * @param int pulseRate Set the pulse rate value in the textview
    public void setpulserate(int pulserate) {

            this
     * @param int pulseStrength Set the pulse strength value in the list view
     * @param int pi Calculate the perfusion index
     * @param JSONArray pulseWave Store the pulse wave data
     *
     * @return A jsonarray which i want to use in the following function:
     *
     * @docauthor Lorenzo Rossi
     */

    private String generateTTSReport(int bloodOxygen, int pulseRate){
        String report = "Il tuo livello di ossigeno nel sangue è al " + bloodOxygen + "%  e il tuo battito è " + pulseRate + " battiti al minuto.";
        return report;
    }
    public void setResultList(int bloodOxygen, int pulseRate, int pulseStrength, int pi, JSONArray pulseWave) {

        String[] values = new String[] { "Blood Oxygen: " + bloodOxygen + "%\t", "Pulse Rate: " + pulseRate + "bpm\t", "Pulse Strength: " + pulseStrength, "\tPi: " + pi};
        tts.setContent(generateTTSReport(bloodOxygen, pulseRate));
        itemArray.add(0, Arrays.toString(values));
        Log.e(TAG, "itemArray: " + itemArray);
        adapter.notifyDataSetChanged();
        tts.execute();
        motion.setContent(motion.get_arm_down());
        motion.execute();
    }

    /**
     * The setLiveResult function is called by the LiveResultActivity class when a new blood oxygen value is received from the
     * Bluetooth device. The function updates the chart with this new data point and then calls drawChart to update the chart.

     *
     * @param int bloodOxygen Get the current blood oxygen level from the main activity
     *
     * @return The value of the bloodoxygen variable
     *
     * @docauthor Lorenzo Rossi
     */
    public void setLiveResult(int bloodOxygen) {

        drawChart(bloodOxygen);


    }

    /**
     * The drawChart function takes in a bloodOxygen value and adds it to the dataPoints list.
     * It then updates the graph with this new data point.

     *
     * @param int bloodOxygen Store the blood oxygen value
    &lt;/code&gt;

     *
     * @return A datapoint object
     *
     * @docauthor Lorenzo Rossi
     */
    private void drawChart(int bloodOxygen) {
        DataPoint dataPoint = new DataPoint(dataPoints.size() + 1, bloodOxygen);
        // Add the data point to your dataPoints list
        setDataPoint(dataPoint);
        Log.e("DataPoint", "dataPoint: " + getDataPoint().size());

    }

    // On back pressed destroy the activity and the runnable
    @Override
    public void onBackPressed() {
        super.onBackPressed();
        thread.interrupt();
        motion.setContent(motion.get_arm_down());
        motion.execute();
        finish();
    }
}





