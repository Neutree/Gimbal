package com.neucrack.gimbal;

import android.content.Context;
import android.net.Uri;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;


public class FragmentStatus extends Fragment {
    private static final String ARG_PARAM1 = "param1";
    private static final String ARG_PARAM2 = "param2";

    private String mParam1;
    private String mParam2;


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (getArguments() != null) {
            mParam1 = getArguments().getString(ARG_PARAM1);
            mParam2 = getArguments().getString(ARG_PARAM2);
        }
    }


    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        return inflater.inflate(R.layout.fragment_fragment_status, container, false);
    }

    public void showAngle(double roll,double pitch,double yaw){
        Log.v("a","showAngle:"+roll+"\t"+pitch+"\t"+yaw);
    }


    public void showArm(boolean armed) {

    }

    public void showRawData(double acc_x, double acc_y, double acc_z, int gryo_x, int gryo_y, int gryo_z, int mag_x, int mag_y, int mag_z) {

    }

    public void showRCData(double rc_roll, double rc_pitch, double rc_yaw) {
    }

    public void showVoltage(double voltage) {
    }

    public void showMotor(int motor_roll, int motor_pitch, int motor_yaw) {
    }
}
