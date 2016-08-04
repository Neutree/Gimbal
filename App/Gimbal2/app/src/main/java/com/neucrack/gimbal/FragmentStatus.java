package com.neucrack.gimbal;

import android.content.Context;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;


public class FragmentStatus extends Fragment {
    private static final String ARG_PARAM1 = "param1";
    private static final String ARG_PARAM2 = "param2";

    private String mParam1;
    private String mParam2;
    private TextView mAngleRoll,mAnglePitch,mAngleYaw, mAccX,mAccY,mAccZ, mGyroX,mGyroY,mGyroZ, mMagX,mMagY,mMagZ, mMotorRoll,mMotorPitch,mMotorYaw, mVoltage;


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
        View view = inflater.inflate(R.layout.fragment_fragment_status, container, false);
        mAngleRoll = (TextView) view.findViewById(R.id.textView_angle_roll);
        mAnglePitch = (TextView) view.findViewById(R.id.textView_angle_pitch);
        mAccX = (TextView) view.findViewById(R.id.textView_acc_x);
        mAccY = (TextView) view.findViewById(R.id.textView_acc_y);
        mAccZ = (TextView) view.findViewById(R.id.textView_acc_z);
        mGyroX = (TextView) view.findViewById(R.id.textView_gyro_x);
        mGyroY = (TextView) view.findViewById(R.id.textView_gyro_y);
        mGyroZ = (TextView) view.findViewById(R.id.textView_gyro_z);
        mMagX = (TextView) view.findViewById(R.id.textView_mag_x);
        mMagY = (TextView) view.findViewById(R.id.textView_mag_y);
        mMagZ = (TextView) view.findViewById(R.id.textView_mag_z);
        mMotorRoll = (TextView) view.findViewById(R.id.textView_motor_roll);
        mMotorPitch = (TextView) view.findViewById(R.id.textView_motor_pitch);
        mMotorYaw = (TextView) view.findViewById(R.id.textView_motor_yaw);
        mVoltage = (TextView) view.findViewById(R.id.textView_voltage);

        return view;
    }

    public void showAngle(double roll,double pitch,double yaw){
        Bundle data = new Bundle();
        data.putDouble("roll",roll);
        data.putDouble("pitch",pitch);
        data.putDouble("yaw",yaw);
        Message msg = new Message();
        msg.setData(data);
        setAngle.sendMessage(msg);
    }

    Handler setAngle = new Handler(){
        @Override
        public void handleMessage(Message msg) {
            super.handleMessage(msg);
            Bundle data = msg.getData();
            double roll = data.getDouble("roll",0);
            double pitch = data.getDouble("pitch",0);
            double yaw = data.getDouble("yaw",0);
            mAngleRoll.setText(roll+"°");
            mAnglePitch.setText(pitch+"°");
            mAngleYaw.setText(yaw+"°");
        }
    };

    public void showRawData(double acc_x, double acc_y, double acc_z, int gryo_x, int gryo_y, int gryo_z, int mag_x, int mag_y, int mag_z) {
        Bundle data = new Bundle();
        data.putDouble("acc_x",acc_x);
        data.putDouble("acc_y",acc_y);
        data.putDouble("acc_z",acc_z);
        data.putDouble("gyro_x",gryo_x);
        data.putDouble("gyro_y",gryo_y);
        data.putDouble("gyro_z",gryo_z);
        data.putDouble("mag_x",mag_x);
        data.putDouble("mag_y",mag_y);
        data.putDouble("mag_z",mag_z);
        Message msg = new Message();
        msg.setData(data);
        setRawData.sendMessage(msg);
    }
    Handler setRawData = new Handler(){
        @Override
        public void handleMessage(Message msg) {
            super.handleMessage(msg);
            Bundle data = msg.getData();
            double acc_x = data.getDouble("acc_x",0);
            double acc_y = data.getDouble("acc_y",0);
            double acc_z = data.getDouble("acc_z",0);
            int gyro_x = data.getInt("gyro_x",0);
            int gyro_y = data.getInt("gyro_y",0);
            int gyro_z = data.getInt("gyro_z",0);
            int mag_x = data.getInt("mag_x",0);
            int mag_y = data.getInt("mag_y",0);
            int mag_z = data.getInt("mag_z",0);
            mAccX.setText(acc_x+"");
            mAccY.setText(acc_y+"");
            mAccZ.setText(acc_z+"");
            mGyroX.setText(gyro_x+"");
            mGyroY.setText(gyro_y+"");
            mGyroZ.setText(gyro_z+"");
            mMagX.setText(mag_x+"");
            mMagY.setText(mag_y+"");
            mMagZ.setText(mag_z+"");
        }
    };



    public void showRCData(double rc_roll, double rc_pitch, double rc_yaw) {
    }

    public void showVoltage(double voltage) {
        Bundle data = new Bundle();
        data.putDouble("voltage",voltage);
        Message msg = new Message();
        msg.setData(data);
        setVoltage.sendMessage(msg);
    }
    Handler setVoltage = new Handler(){
        @Override
        public void handleMessage(Message msg) {
            super.handleMessage(msg);
            Bundle data = msg.getData();
            double voltage = data.getDouble("voltage",0);
            mVoltage.setText(voltage+"V");
        }
    };
    public void showMotor(int motor_roll, int motor_pitch, int motor_yaw) {
        Bundle data = new Bundle();
        data.putInt("roll",motor_roll);
        data.putInt("pitch",motor_pitch);
        data.putInt("yaw",motor_yaw);
        Message msg = new Message();
        msg.setData(data);
        setRawData.sendMessage(msg);
    }

    Handler setMotor = new Handler(){
        @Override
        public void handleMessage(Message msg) {
            super.handleMessage(msg);
            Bundle data = msg.getData();
            int roll = data.getInt("roll",0);
            int pitch = data.getInt("pitch",0);
            int yaw = data.getInt("yaw",0);
            mMotorRoll.setText(roll+"");
            mMotorPitch.setText(pitch+"");
            mMotorYaw.setText(yaw+"");
        }
    };
}
