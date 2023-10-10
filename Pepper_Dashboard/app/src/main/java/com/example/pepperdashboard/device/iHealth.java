package com.example.pepperdashboard.device;

import android.Manifest;
import android.app.Activity;
import android.app.Application;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Handler;
import android.os.Message;
import android.util.Log;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.example.pepperdashboard.R;
import com.example.pepperdashboard.activity.MisActivity;
import com.ihealth.communication.control.HsProfile;
import com.ihealth.communication.control.Po3Control;
import com.ihealth.communication.control.PoProfile;
import com.ihealth.communication.control.UpgradeProfile;
import com.ihealth.communication.manager.DiscoveryTypeEnum;
import com.ihealth.communication.manager.iHealthDevicesCallback;
import com.ihealth.communication.manager.iHealthDevicesManager;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONTokener;

import java.io.IOException;
import java.io.InputStream;
import java.util.Map;

public class iHealth extends Application {

    private static String TAG = iHealth.class.getName();

    private static  iHealth mInstance;
    public static Context applicationContext;
    private String mac;

    /**
     * The getPo3 function returns the PO3 object.
     *
     *
     *
     * @return The po3 object
     *
     * @docauthor Lorenzo Rossi
     */
    public PO3 getPo3() {
        return po3;
    }

    /**
     * The setPo3 function sets the value of the po3 variable.
     *
     *
     * @param PO3 po3 Set the value of the po3 variable
     *
     * @return Void
     *
     * @docauthor Lorenzo Rossi
     */
    public void setPo3(PO3 po3) {
        this.po3 = po3;
    }

    private PO3 po3;


    /**
     * The getControl function returns the control object for this class.
     *
     *
     *
     * @return The control object, which is the po3control class
     *
     * @docauthor Lorenzo Rossi
     */
    public Po3Control getControl() {
        return control;
    }

    /**
     * The setControl function is used to set the control variable of the class.
     *
     *
     * @param Po3Control control Set the control variable to a new value
     *
     * @return Nothing, so the return type is void
     *
     * @docauthor Lorenzo Rossi
     */
    public void setControl(Po3Control control) {
        this.control = control;
    }

    private Po3Control control;

    /**
     * The getMac function returns the mac address of the device.
     *
     *
     *
     * @return The mac address of the device
     *
     * @docauthor Lorenzo Rossi
     */
    public String getMac() {
        return mac;
    }

    /**
     * The setMac function sets the mac address of a device.
     *
     *
     * @param String mac Set the mac address of the device
     *
     * @return Nothing, so it is a void function
     *
     * @docauthor Lorenzo Rossi
     */
    public void setMac(String mac) {
        this.mac = mac;
    }

    /**
     * The getDeviceType function returns the deviceType variable.
     *
     *
     *
     * @return The device type
     *
     * @docauthor Lorenzo Rossi
     */
    public String getDeviceType() {
        return deviceType;
    }

    /**
     * The setDeviceType function sets the deviceType variable to a new value.
     *
     *
     * @param String deviceType Set the devicetype instance variable
     *
     * @return Nothing
     *
     * @docauthor Lorenzo Rossi
     */
    public void setDeviceType(String deviceType) {
        this.deviceType = deviceType;
    }

    private String deviceType;
    private iHealthDevicesCallback miHealthDevicesCallback = new iHealthDevicesCallback() {




        /**
         * The onDeviceNotify function is called when the device sends a notification to the app.
         *
         *
         * @param String mac Identify the device
         * @param String deviceType Determine which device is being used
         * @param String action Determine which action is being performed
         * @param String message Display the data in a textview
         *
         * @return A string, what is this string?
         *
         * @docauthor Lorenzo Rossi
         */
        @Override
        public void onDeviceNotify(String mac, String deviceType, String action, String message) {
            if (PoProfile.ACTION_BATTERY_PO.equals(action)) {
                try {
                    JSONObject obj = new JSONObject(message);
                    int battery = obj.getInt(PoProfile.BATTERY_PO);
                    Log.e(this.getClass().getName(), "battery: " + battery);
                    getPo3().setBattery(battery);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
            }
            if (PoProfile.ACTION_LIVEDA_PO.equals(action)) {
                try {
                    JSONObject obj = new JSONObject(message);

                    int bloodOxygen = obj.getInt(PoProfile.BLOOD_OXYGEN_PO);
                    int pulseRate = obj.getInt(PoProfile.PULSE_RATE_PO);
                    int pulseStrength = obj.getInt(PoProfile.PULSE_STRENGTH_PO);
                    int pi = obj.getInt(PoProfile.PI_PO);
                    JSONArray pulseWave = obj.getJSONArray(PoProfile.PULSE_WAVE_PO);
                    po3.setLiveResult(pulseRate);


                } catch (JSONException e) {
                    e.printStackTrace();
                }
            } else if (PoProfile.ACTION_RESULTDATA_PO.equals(action)) {
                try {
                    JSONObject obj = new JSONObject(message);

                    int bloodOxygen = obj.getInt(PoProfile.BLOOD_OXYGEN_PO);
                    int pulseRate = obj.getInt(PoProfile.PULSE_RATE_PO);
                    int pulseStrength = obj.getInt(PoProfile.PULSE_STRENGTH_PO);
                    int pi = obj.getInt(PoProfile.PI_PO);
                    JSONArray pulseWave = obj.getJSONArray(PoProfile.PULSE_WAVE_PO);
                    // wait 1 sec

                    po3.setResultList(bloodOxygen, pulseRate, pulseStrength, pi, pulseWave);
//                    po3.setResultList(bloodOxygen, pulseRate, pulseStrength, pi, pulseWave);
                } catch (JSONException e) {
                    e.printStackTrace();
                }
            } else if (PoProfile.ACTION_OFFLINEDATA_PO.equals(action)) {
                try {
                    JSONTokener jsonTokener = new JSONTokener(message);

                    JSONObject object = (JSONObject) jsonTokener.nextValue();
                    JSONArray jsonArray = object.getJSONArray(PoProfile.OFFLINEDATA_PO);

                } catch (JSONException e) {
                    e.printStackTrace();
                }
            }
        }

        /**
         * The onScanDevice function is called when a device is found during the scan process.
         *
         *
         * @param String mac Set the mac address of the device
         * @param String deviceType Identify the type of device that is being scanned
         * @param int rssi Determine the strength of the signal
         * @param Map manufacturerData Get the mac suffix of the device
         *
         * @return The device type and the mac address of the device
         *
         * @docauthor Lorenzo Rossi
         */
        @Override
        public void onScanDevice(String mac, String deviceType, int rssi, Map manufacturerData) {
            Log.i(TAG, "onScanDevice - mac:" + mac + " - deviceType:" + deviceType + " - rssi:" + rssi + " - manufacturerData:" + manufacturerData);
            if ((manufacturerData != null) && (manufacturerData.get("isBound") != null)) {
                boolean isBound = (boolean) manufacturerData.get("isBound");
                Log.i(TAG, "isBound AM6 - " + isBound);
            }
            setMac(mac);
            setDeviceType(deviceType);

            if (manufacturerData != null) {
                Log.d(TAG, "onScanDevice mac suffix = " + manufacturerData.get(HsProfile.SCALE_WIFI_MAC_SUFFIX));
            }


        }

        /**
         * The onDeviceConnectionStateChange function is called when the connection state of a device changes.
         *
         *
         * @param String mac Identify the device
         * @param String deviceType Identify the type of device
         * @param int status Determine whether the device is connected or not
         * @param int errorID Determine the error code
         * @param Map manufacturerData Return the manufacturer data of the device
         *
         * @return The device's mac address, the type of device,
         *
         * @docauthor Lorenzo Rossi
         */
        @Override
        public void onDeviceConnectionStateChange(String mac, String deviceType, int status, int errorID, Map manufacturerData) {
            Log.e(TAG, "mac:" + mac + " deviceType:" + deviceType + " status:" + status + " errorId:" + errorID + " - manufacturerData:" + manufacturerData);
            iHealthDevicesManager.getInstance().stopDiscovery();
//            iHealthDevicesManager.getInstance().unRegisterClientCallback(callbackId);
            Log.e(TAG, "CONNECTED");
            setControl(iHealthDevicesManager.getInstance().getPo3Control(getMac()));

            getPo3().enableMeasure();


        }

        /**
         * Callback indicating an error happened during discovery.
         *
         * @param reason A string for the reason why discovery failed.
         */
        @Override
        public void onScanError(String reason, long latency) {
            Log.e(TAG, reason);
            Log.e(TAG, "please wait for " + latency + " ms");

        }

        /**
         * The onScanFinish function is called when the scan for devices has finished.
         * It then connects to the device with a given MAC address and type.

         *
         *
         * @return The device's mac address and type
         *
         * @docauthor Lorenzo Rossi
         */
        @Override
        public void onScanFinish() {
            super.onScanFinish();
            iHealthDevicesManager.getInstance().connectDevice(getMac(), getDeviceType());
        }

        /**
         * The onSDKStatus function is called when the SDK has a status update.
         *
         *
         * @param int statusId Determine the status of the sdk
         * @param String statusMessage Display the status of the sdk
         *
         * @return The status of the sdk
         *
         * @docauthor Lorenzo Rossi
         */
        @Override
        public void onSDKStatus(int statusId, String statusMessage) {
            Log.e(TAG, "statusId: " + statusId);
            Log.e(TAG, "statusMessage: " + statusMessage);
        }


    };
    private int callbackId;


    /**
     * The onCreate function is called when the application is first created.
     * It initializes the application context and sets up a few other things.

     *
     *
     * @return A void type
     *
     * @docauthor Lorenzo Rossi
     */
    @Override
    public void onCreate() {
        super.onCreate();
        mInstance = this;
        applicationContext = this;
        init();
    }

    /**
     * The init function initializes the iHealthDevicesManager.
     *
     *
     *
     * @return A boolean
     *
     * @docauthor Lorenzo Rossi
     */
    public void init(){
        try {
            iHealthDevicesManager.getInstance().init(mInstance, Log.VERBOSE, Log.VERBOSE);
            auth();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * The auth function is used to authenticate the SDK.
     *
     *
     *
     * @return A boolean value
     *
     * @docauthor Lorenzo Rossi
     */
    public boolean auth(){
        try {
            InputStream is = applicationContext.getAssets().open("com_example_pepperdashboard_android.pem");
            int size = is.available();
            byte[] buffer = new byte[size];
            is.read(buffer);
            is.close();
            boolean isPass = iHealthDevicesManager.getInstance().sdkAuthWithLicense(buffer);
            return isPass;

        } catch (IOException e) {
            e.printStackTrace();
        }

        return false;
    }

    /**
     * The discover function is used to discover the iHealth PO3 device.
     *
     *
     * @param Activity activity Get the context of the activity
     *
     * @return The following:
     *
     * @docauthor Lorenzo Rossi
     */
    public void discover(Activity activity){
        setPo3((PO3) activity);
        checkPermission(activity);
        callbackId = iHealthDevicesManager.getInstance().registerClientCallback(miHealthDevicesCallback);
        iHealthDevicesManager.getInstance().addCallbackFilterForDeviceType(callbackId, iHealthDevicesManager.TYPE_PO3);
        iHealthDevicesManager.getInstance().addCallbackFilterForAddress(callbackId,"6C79B841CB87");
        iHealthDevicesManager.getInstance().startDiscovery(DiscoveryTypeEnum.valueOf("PO3"));
    }

    /**
     * The checkPermission function checks the version of Android that is running on the device and then requests
     * permissions for location, audio recording, and external storage. The function also checks to see if these
     * permissions have already been granted. If they have not been granted, it will request them from the user.

     *
     * @param Activity activity Request the permissions
     *
     * @return A boolean value
     *
     * @docauthor Lorenzo Rossi
     */
    private void checkPermission(Activity activity) {
        int version = Build.VERSION.SDK_INT;
        if (version > 30) {
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(activity, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE,
                        Manifest.permission.ACCESS_COARSE_LOCATION,
                        Manifest.permission.ACCESS_FINE_LOCATION,
                        Manifest.permission.RECORD_AUDIO,
                        Manifest.permission.BLUETOOTH_SCAN,
                        Manifest.permission.BLUETOOTH_CONNECT,
                        Manifest.permission.BLUETOOTH_ADVERTISE}, 1);
            }
        } else {
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED ||
                    ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(activity, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE, Manifest.permission.ACCESS_COARSE_LOCATION, Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.RECORD_AUDIO}, 1);
            }
        }

    }
}
